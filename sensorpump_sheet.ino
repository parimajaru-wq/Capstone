/*
  =====================================================
  EC + Ultrasonic + NETPIE + Google Sheet
  + Peristaltic Pump A/B via L298N
  =====================================================
  รับจาก NETPIE (@msg/config):
    {"ecTarget":2000}
    {"distTarget":15.0}

  ส่งขึ้น NETPIE shadow:
    ec, mv, temp, ecTarget, dist, distTarget, pwmA, pwmB

  Peristaltic Pump:
    ปั๊ม A (ปุ๋ย A) — PWM=GPIO25 → L298N ENA → OUT1/2
    ปั๊ม B (ปุ๋ย B) — PWM=GPIO27 → L298N ENB → OUT3/4
    อัตราส่วน A:B = 4:1 (40ml:10ml ต่อน้ำ 1000ml)
    PWM 0-255 แปรตาม ecTarget - EC (เมื่อ EC < ecTarget)
    ไม่ใช้ relay — L298N ตัดไฟได้เองตอน PWM=0
  =====================================================
*/

#include <SPI.h>
#include <TFT_22_ILI9225.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==========================================
// WiFi
// ==========================================
const char* ssid     = "EETAR3";
const char* password = "EETARNET";

// ==========================================
// Google Sheet
// ==========================================
const char* scriptURL = "https://script.google.com/macros/s/AKfycbz5vPyy7ZPPOjNMF3sTzzm4Sda6s96z0pwCtgkrWaQQZkipiktWZxg7DVPHRd8HPSYE/exec";

// ==========================================
// NETPIE 2020
// ==========================================
const char* mqtt_server = "broker.netpie.io";
const int   mqtt_port   = 1883;
const char* client_id   = "7e7249ed-78ff-46c7-a78b-febf5ba8ebb3";
const char* token       = "6cdv5bKihQjybaqzw4XqDQUKNxtLHbke";
const char* secret      = "L1K5bbVZZUpeyAncJKE7XPWr9hqdTH9Q";

const char* TOPIC_CONFIG = "@msg/config";
const char* TOPIC_SHADOW = "@shadow/data/update";

// ==========================================
// TFT ILI9225
// ==========================================
#define TFT_RST 4
#define TFT_RS  2
#define TFT_CS  5
#define TFT_SDI 23
#define TFT_CLK 18
#define TFT_LED 0
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED);

// ==========================================
// DS18B20
// ==========================================
#define ONE_WIRE_BUS 15
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float currentTemp  = 25.0;
unsigned long tempTime = 0;

// ==========================================
// EC Sensor
// ==========================================
#define EC_PIN 34

const byte    NUM_MA = 10;
unsigned int  ecReadings[NUM_MA];
byte          ecIdx        = 0;
unsigned long ecTotal      = 0;
unsigned long ecSampleTime = 0;

// ==========================================
// Ultrasonic HC-SR04
// ==========================================
#define TRIG_PIN 13
#define ECHO_PIN 12

// ==========================================
// Peristaltic Pump A/B via L298N
// ==========================================
#define PWM_A_PIN    25   // ENA ของ L298N channel 1 → ปั๊ม A
#define PWM_B_PIN    27   // ENB ของ L298N channel 2 → ปั๊ม B

#define PWM_FREQ     1000 // 1kHz
#define PWM_RES      8    // 8-bit = 0-255

// อัตราส่วน A:B = 4:1
const float RATIO_A  = 4.0f;
const float RATIO_B  = 1.0f;

// หน่วง pump หลังจาก EC ถึงเป้า (ms) ป้องกัน overshoot
const unsigned long PUMP_STOP_DELAY = 5000;
unsigned long pumpStopTime = 0;
bool pumpRunning = false;

int currentPWM_A = 0;
int currentPWM_B = 0;

// ==========================================
// CONFIG
// ==========================================
float ecTarget   = 2000.0;
float distTarget = 15.0;

// ==========================================
// กราฟ TFT
// ==========================================
int startX = 25, endX = 220;
int topY   = 90, bottomY = 165;
int graphX = startX;
int lastYec = 165;

// ==========================================
// เวลา
// ==========================================
unsigned long printTime = 0;
const unsigned int printInterval = 1000;

// ==========================================
// MQTT
// ==========================================
WiFiClient   espClient;
PubSubClient mqttClient(espClient);

// ==========================================
// MQTT Callback
// ==========================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("NETPIE: " + msg);

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, msg)) return;

  if (doc.containsKey("ecTarget")) {
    ecTarget = doc["ecTarget"].as<float>();
    Serial.println("ecTarget: " + String(ecTarget));
  }
  if (doc.containsKey("distTarget")) {
    distTarget = doc["distTarget"].as<float>();
    Serial.println("distTarget: " + String(distTarget));
  }
}

// ==========================================
// MQTT reconnect
// ==========================================
void reconnectMQTT() {
  int retry = 0;
  while (!mqttClient.connected() && retry < 3) {
    Serial.print("MQTT connecting...");
    if (mqttClient.connect(client_id, token, secret)) {
      Serial.println("OK");
      mqttClient.subscribe(TOPIC_CONFIG);
    } else {
      Serial.println("fail rc=" + String(mqttClient.state()));
      delay(2000);
      retry++;
    }
  }
}

// ==========================================
// FreeRTOS HTTP Task — Google Sheet
// ==========================================
String pendingURL  = "";
bool   sendPending = false;
SemaphoreHandle_t urlMutex;

void sendTask(void* param) {
  while (true) {
    if (sendPending) {
      String url;
      if (xSemaphoreTake(urlMutex, portMAX_DELAY)) {
        url = pendingURL;
        sendPending = false;
        xSemaphoreGive(urlMutex);
      }
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(url);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.GET();
        http.end();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ==========================================
// Moving Average EC
// ==========================================
void updateMA() {
  ecTotal -= ecReadings[ecIdx];
  ecReadings[ecIdx] = analogRead(EC_PIN);
  ecTotal += ecReadings[ecIdx];
  ecIdx = (ecIdx + 1) % NUM_MA;
}

// ==========================================
// คำนวณ EC — สมการ cubic
// y = -0.0007x^3 - 0.1415x^2 + 17.1793x + 416.9638
// หาร coef ชดเชยอุณหภูมิ
// ==========================================
float calcEC(int MV, float coef) {
  float x = (float)MV;
  float raw = (-0.0007f * x * x * x)
            + (-0.1415f * x * x)
            + ( 17.1793f * x)
            +   416.9638f;
  return raw / coef;
}

// ==========================================
// ส่ง Google Sheet
// ==========================================
void sendToSheet(float temp, int MV, float EC,
                 float ecTgt, float dist, float distTgt) {
  if (WiFi.status() != WL_CONNECTED) return;
  String url = String(scriptURL)
    + "?temp="       + String(temp,   1)
    + "&mv="         + String(MV)
    + "&ec="         + String(EC,     2)
    + "&ecTarget="   + String(ecTgt,  1)
    + "&dist="       + String(dist,   1)
    + "&distTarget=" + String(distTgt,1);
  if (xSemaphoreTake(urlMutex, portMAX_DELAY)) {
    pendingURL  = url;
    sendPending = true;
    xSemaphoreGive(urlMutex);
  }
}

// ==========================================
// ส่ง NETPIE Shadow
// ==========================================
void sendToNetpie(float temp, int MV, float EC,
                  float ecTgt, float dist, float distTgt) {
  if (!mqttClient.connected()) return;

  String payload = "{\"data\":{"
    "\"ec\":"         + String(EC,     2) + ","
    "\"mv\":"         + String(MV)        + ","
    "\"temp\":"       + String(temp,   1) + ","
    "\"ecTarget\":"   + String(ecTgt,  1) + ","
    "\"dist\":"       + String(dist,   1) + ","
    "\"distTarget\":" + String(distTgt,1) + ","
    "\"pwmA\":"       + String(currentPWM_A) + ","
    "\"pwmB\":"       + String(currentPWM_B) +
    "}}";
  mqttClient.publish(TOPIC_SHADOW, payload.c_str());

  mqttClient.publish("@feed/ec",   String(EC,   2).c_str());
  mqttClient.publish("@feed/temp", String(temp, 1).c_str());
  mqttClient.publish("@feed/dist", String(dist, 1).c_str());

  Serial.println(payload);
}

// ==========================================
// แสดงผล TFT
// ==========================================
void drawScreen(int MV, float EC, float temp,
                float ecTgt, float dist, float distTgt) {
  tft.fillRectangle(45, 0, 220, 84, COLOR_BLACK);

  // แถว 1: MV
  tft.drawText(45, 2, String(MV) + " mV", COLOR_YELLOW);

  // แถว 2: Temp
  tft.drawText(45, 16, String(temp, 1) + " C", COLOR_ORANGE);

  // แถว 3: EC + สถานะ
  float ecDiff = EC - ecTgt;
  unsigned int ecColor;
  String ecStatus;
  if (abs(ecDiff) <= ecTgt * 0.05) {
    ecColor = COLOR_GREEN; ecStatus = "OK";
  } else if (ecDiff > 0) {
    ecColor = COLOR_RED;   ecStatus = "HI";
  } else {
    ecColor = COLOR_CYAN;  ecStatus = "LO";
  }
  tft.drawText(45,  32, String(EC, 1), ecColor);
  tft.drawText(150, 32, ecStatus,      ecColor);

  // แถว 4: EC target
  tft.drawText(45, 48, ">" + String(ecTgt, 0), COLOR_WHITE);

  // แถว 5: ระยะ sensor→น้ำ + สถานะ
  unsigned int dColor;
  String dStatus;
  if (dist < 0) {
    dColor = COLOR_RED; dStatus = "ERR";
    tft.drawText(45, 64, "--- cm", dColor);
  } else {
    float dDiff = dist - distTgt;
    if (abs(dDiff) <= distTgt * 0.05) {
      dColor = COLOR_GREEN; dStatus = "OK";
    } else if (dDiff < 0) {
      dColor = COLOR_CYAN; dStatus = "HI";
    } else {
      dColor = COLOR_RED;  dStatus = "LO";
    }
    tft.drawText(45,  64, String(dist, 1) + " cm", dColor);
    tft.drawText(150, 64, dStatus,                 dColor);
  }
}

// ==========================================
// ควบคุม Peristaltic Pump A/B
// ==========================================
void setPeriPumps(int pwmA, int pwmB) {
  pwmA = constrain(pwmA, 0, 255);
  pwmB = constrain(pwmB, 0, 255);
  currentPWM_A = pwmA;
  currentPWM_B = pwmB;
  ledcWrite(PWM_A_PIN, pwmA);
  ledcWrite(PWM_B_PIN, pwmB);
}

// คำนวณ PWM จาก EC error และอัตราส่วน A:B
// error > 200  → เต็มสปีด 255
// error <= 200 → proportional แต่ไม่ต่ำกว่า 170
// error = 0    → หยุด
void updatePeriPumps(float EC, float ecTgt) {
  float err = ecTgt - EC;

  if (err <= 0) {
    if (pumpRunning) {
      pumpRunning  = false;
      pumpStopTime = millis();
    }
    setPeriPumps(0, 0);
    return;
  }

  int pwmBase;
  if (err > 200.0f) {
    pwmBase = 255;
  } else {
    pwmBase = (int)constrain((err / 200.0f) * 255.0f, 170, 255);
  }

  float total = RATIO_A + RATIO_B;
  int   pwmA  = (int)(pwmBase * RATIO_A / total);
  int   pwmB  = (int)(pwmBase * RATIO_B / total);

  pumpRunning = true;
  setPeriPumps(pwmA, pwmB);

  Serial.printf("PeriPump err:%.1f pwmBase:%d pwmA:%d pwmB:%d\n", err, pwmBase, pwmA, pwmB);
}

// ==========================================
// อ่าน Ultrasonic HC-SR04
// ==========================================
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1.0;
  return (float)dur / 29.0 / 2.0;
}

// ==========================================
// Setup
// ==========================================
void setup() {
  Serial.begin(9600);
  analogReadResolution(12);

  sensors.begin();

  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);

  // Peristaltic Pump A/B via L298N (ESP32 Core 3.x API)
  ledcAttach(PWM_A_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(PWM_B_PIN, PWM_FREQ, PWM_RES);
  ledcWrite(PWM_A_PIN, 0);
  ledcWrite(PWM_B_PIN, 0);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println(" OK: " + WiFi.localIP().toString());

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(256);
  reconnectMQTT();

  urlMutex = xSemaphoreCreateMutex();
  xTaskCreate(sendTask, "sendTask", 8192, NULL, 1, NULL);

  tft.begin();
  tft.setOrientation(1);
  tft.clear();
  tft.setFont(Terminal6x8);

  // Label คงที่
  tft.drawText(2,  2,  "MV:",  COLOR_WHITE);
  tft.drawText(2,  16, "T:",   COLOR_WHITE);
  tft.drawText(2,  32, "EC:",  COLOR_WHITE);
  tft.drawText(2,  48, "TGT:", COLOR_WHITE);
  tft.drawText(2,  64, "Dst:", COLOR_WHITE);

  // กรอบกราฟ
  tft.drawLine(startX, topY,    startX, bottomY, COLOR_WHITE);
  tft.drawLine(startX, bottomY, endX,   bottomY, COLOR_WHITE);
  tft.drawText(2,   topY - 4,    "8000", COLOR_YELLOW);
  tft.drawText(2,   bottomY - 4, " 500", COLOR_YELLOW);
  tft.drawText(180, bottomY + 4, "200s", COLOR_YELLOW);

  memset(ecReadings, 0, sizeof(ecReadings));
}

// ==========================================
// Loop
// ==========================================
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
  }

  unsigned long now = millis();

  // อ่านอุณหภูมิ DS18B20 ทุก 5 วิ
  if (now - tempTime >= 5000) {
    tempTime = now;
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) currentTemp = t;
  }

  // MA ทุก 100ms
  if (now - ecSampleTime >= 100) {
    ecSampleTime = now;
    updateMA();
  }

  // ประมวลผลทุก 1 วิ
  if (now - printTime >= printInterval) {
    printTime = now;

    float coef = 1.0f + 0.0185f * (currentTemp - 25.0f);
    int   MV   = (int)((ecTotal / NUM_MA) * 3300.0f / 4095.0f);
    float EC   = calcEC(MV, coef);
    float dist = readDistance();

    // TFT
    drawScreen(MV, EC, currentTemp, ecTarget, dist, distTarget);

    // กราฟ
    int yEC = map(constrain((int)EC, 500, 8000), 500, 8000, bottomY, topY);
    if (graphX == startX) lastYec = yEC;
    tft.drawLine(graphX - 1, lastYec, graphX, yEC, COLOR_RED);
    lastYec = yEC;
    graphX++;
    if (graphX > endX) {
      tft.fillRectangle(startX + 1, topY, endX, bottomY - 1, COLOR_BLACK);
      graphX = startX;
    }

    // Serial
    Serial.print(currentTemp, 1); Serial.print(",");
    Serial.print(MV);             Serial.print(",");
    Serial.print(EC, 2);          Serial.print(",");
    Serial.print(ecTarget, 1);    Serial.print(",");
    Serial.print(dist, 1);        Serial.print(",");
    Serial.println(distTarget, 1);

    // ส่งข้อมูล
    sendToSheet(currentTemp, MV, EC, ecTarget, dist, distTarget);
    sendToNetpie(currentTemp, MV, EC, ecTarget, dist, distTarget);

    // ควบคุม Peristaltic Pump
    updatePeriPumps(EC, ecTarget);
  }
}
