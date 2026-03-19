/*
  =====================================================
  EC + Ultrasonic + NETPIE + Google Sheet
  =====================================================
  รับจาก NETPIE (@msg/config):
    {"ecTarget":2000}
    {"distTarget":15.0}  ระยะ sensor->น้ำ เป้าหมาย cm

  ส่งขึ้น NETPIE shadow:
    eq3, mV3, temp, ecTarget, dist, distTarget

  TFT แสดง:
    mV3 | Temp | EC+status | ECTarget
    Dist | status | กราฟ EC
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
// Google Sheet กรอกเพิ่ม
// ==========================================
const char* scriptURL = "  ";

// ==========================================
// NETPIE 2020 กรอกเพิ่ม
// ==========================================
const char* mqtt_server = "broker.netpie.io";
const int   mqtt_port   = 1883;
const char* client_id   = "  ";
const char* token       = "  ";
const char* secret      = "  ";

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
float currentTemp    = 25.0;
unsigned long tempTime = 0;

// ==========================================
// EC Sensor
// ==========================================
#define EC_PIN 34

const byte    NUM_MA = 10;
unsigned int  ecReadings[NUM_MA];
byte          ecIdx       = 0;
unsigned long ecTotal     = 0;
unsigned long ecSampleTime = 0;

// ==========================================
// Ultrasonic HC-SR04
// ==========================================
#define TRIG_PIN 13
#define ECHO_PIN 12

// ==========================================
// CONFIG
// ==========================================
float ecTarget   = 2000.0; // ปรับได้จาก NETPIE
float distTarget = 15.0;   // cm sensor→น้ำ เป้าหมาย ปรับได้จาก NETPIE

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
String pendingURL = "";
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
// คำนวณ EC Eq3
// ==========================================
float calcEq3(int mV, float coef) {
  return ((5.2254f * mV) + 408.59f) / coef;
}

// ==========================================
// ส่ง Google Sheet
// ==========================================
void sendToSheet(float temp, int mV3, float eq3,
                 float ecTgt, float dist, float distTgt) {
  if (WiFi.status() != WL_CONNECTED) return;
  String url = String(scriptURL)
    + "?temp="       + String(temp,    1)
    + "&mv3="        + String(mV3)
    + "&eq3="        + String(eq3,     2)
    + "&ecTarget="   + String(ecTgt,   1)
    + "&dist="       + String(dist,    1)
    + "&distTarget=" + String(distTgt, 1);
  if (xSemaphoreTake(urlMutex, portMAX_DELAY)) {
    pendingURL  = url;
    sendPending = true;
    xSemaphoreGive(urlMutex);
  }
}

// ==========================================
// ส่ง NETPIE Shadow
// ==========================================
void sendToNetpie(float temp, int mV3, float eq3,
                  float ecTgt, float dist, float distTgt) {
  if (!mqttClient.connected()) return;
  String payload = "{\"data\":{"
    "\"eq3\":"        + String(eq3,     2) + ","
    "\"mV3\":"        + String(mV3)        + ","
    "\"temp\":"       + String(temp,    1) + ","
    "\"ecTarget\":"   + String(ecTgt,   1) + ","
    "\"dist\":"       + String(dist,    1) + ","
    "\"distTarget\":" + String(distTgt, 1) +
    "}}";
  mqttClient.publish(TOPIC_SHADOW, payload.c_str());
  Serial.println(payload);
}

// ==========================================
// แสดงผล TFT
// ==========================================
void drawScreen(int mV3, float eq3, float temp,
                float ecTgt, float dist, float distTgt) {
  tft.fillRectangle(45, 0, 220, 84, COLOR_BLACK);

  // แถว 1: mV3
  tft.drawText(45, 2, String(mV3) + " mV", COLOR_YELLOW);

  // แถว 2: Temp
  tft.drawText(45, 16, String(temp, 1) + " C", COLOR_ORANGE);

  // แถว 3: EC + สถานะ
  float ecDiff = eq3 - ecTgt;
  unsigned int ecColor;
  String ecStatus;
  if (abs(ecDiff) <= ecTgt * 0.05) {
    ecColor = COLOR_GREEN; ecStatus = "OK";
  } else if (ecDiff > 0) {
    ecColor = COLOR_RED;   ecStatus = "HI";
  } else {
    ecColor = COLOR_CYAN;  ecStatus = "LO";
  }
  tft.drawText(45,  32, String(eq3, 1), ecColor);
  tft.drawText(150, 32, ecStatus,       ecColor);

  // แถว 4: EC target
  tft.drawText(45, 48, ">" + String(ecTgt, 0), COLOR_WHITE);

  // แถว 5: ระยะ sensor→น้ำ + สถานะ
  // dist < distTarget = น้ำมาก (สูง), dist > distTarget = น้ำน้อย
  unsigned int dColor;
  String dStatus;
  if (dist < 0) {
    dColor  = COLOR_RED; dStatus = "ERR";
    tft.drawText(45, 64, "--- cm", dColor);
  } else {
    float dDiff = dist - distTgt;
    if (abs(dDiff) <= distTgt * 0.05) {
      dColor = COLOR_GREEN; dStatus = "OK";
    } else if (dDiff < 0) {
      dColor = COLOR_CYAN;  dStatus = "HI"; // น้ำสูงเกิน (ระยะน้อยกว่า target)
    } else {
      dColor = COLOR_RED;   dStatus = "LO"; // น้ำน้อยเกิน (ระยะมากกว่า target)
    }
    tft.drawText(45,  64, String(dist, 1) + " cm", dColor);
    tft.drawText(150, 64, dStatus,                 dColor);
  }
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

  sensors.begin();           // DS18B20

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

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
  tft.drawText(2,  2,  "mV3:", COLOR_WHITE);
  tft.drawText(2,  16, "T:",   COLOR_WHITE);
  tft.drawText(2,  32, "EC:",  COLOR_WHITE);
  tft.drawText(2,  48, "TGT:", COLOR_WHITE);
  tft.drawText(2,  64, "Dst:", COLOR_WHITE);

  // กรอบกราฟ
  tft.drawLine(startX, topY,    startX, bottomY, COLOR_WHITE);
  tft.drawLine(startX, bottomY, endX,   bottomY, COLOR_WHITE);
  tft.drawText(2,   topY - 4,   "8000", COLOR_YELLOW);
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
    int   mV3  = (int)((ecTotal / NUM_MA) * 3300.0f / 4095.0f);
    float eq3  = calcEq3(mV3, coef);
    float dist = readDistance();

    // TFT
    drawScreen(mV3, eq3, currentTemp, ecTarget, dist, distTarget);

    // กราฟ
    int yEC = map(constrain((int)eq3, 500, 8000), 500, 8000, bottomY, topY);
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
    Serial.print(mV3);            Serial.print(",");
    Serial.print(eq3, 2);         Serial.print(",");
    Serial.print(ecTarget, 1);    Serial.print(",");
    Serial.print(dist, 1);        Serial.print(",");
    Serial.println(distTarget, 1);

    // ส่งข้อมูล
    sendToSheet(currentTemp, mV3, eq3, ecTarget, dist, distTarget);
    sendToNetpie(currentTemp, mV3, eq3, ecTarget, dist, distTarget);
  }
}
