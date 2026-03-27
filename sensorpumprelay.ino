#include <SPI.h>
#include <TFT_22_ILI9225.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ═══════════════════════════════════════════════════════════
//  การตั้งค่าเครือข่าย WiFi
// ═══════════════════════════════════════════════════════════
const char* ssid     = "...";
const char* password = "...";

// ═══════════════════════════════════════════════════════════
//  Google Apps Script URL สำหรับบันทึกข้อมูลลง Google Sheet
// ═══════════════════════════════════════════════════════════
const char* scriptURL = "...";

// ═══════════════════════════════════════════════════════════
//  การตั้งค่า NETPIE MQTT
// ═══════════════════════════════════════════════════════════
const char* mqtt_server           = "broker.netpie.io";
const int   mqtt_port             = 1883;
const char* client_id             = "...";
const char* token                 = "...";
const char* secret                = "...";
const char* TOPIC_CONFIG          = "@msg/ecTarget";
const char* TOPIC_SHADOW          = "@shadow/data/update";
const char* TOPIC_SHADOW_GET      = "@shadow/data/get";
const char* TOPIC_SHADOW_GET_RESP = "@shadow/data/get/response";

// ═══════════════════════════════════════════════════════════
//  จอ TFT ILI9225 (SPI)
// ═══════════════════════════════════════════════════════════
#define TFT_RST 4
#define TFT_RS  2
#define TFT_CS  5
#define TFT_SDI 23
#define TFT_CLK 18
#define TFT_LED 0
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED);

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์อุณหภูมิ DS18B20
// ═══════════════════════════════════════════════════════════
#define ONE_WIRE_BUS 15
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float         currentTemp = 25.0;
unsigned long tempTime    = 0;

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์ EC (Analog → GPIO34) — Moving Average 25 ค่า
// ═══════════════════════════════════════════════════════════
#define EC_PIN 34
const byte    NUM_MA = 25;
unsigned int  ecReadings[NUM_MA];
byte          ecIdx        = 0;
unsigned long ecTotal      = 0;
unsigned long ecSampleTime = 0;

float currentEC = 0.0;
int   currentMV = 0;

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์ระยะ Ultrasonic HC-SR04
// ═══════════════════════════════════════════════════════════
#define TRIG_PIN 13
#define ECHO_PIN 12

// ═══════════════════════════════════════════════════════════
//  Relay ควบคุมปั๊ม (Active LOW)
// ═══════════════════════════════════════════════════════════
#define RELAY_S1  14
#define RELAY_S2  25
#define RELAY_S3  26

// ═══════════════════════════════════════════════════════════
//  ตัวแปรควบคุมปั๊มสารละลาย
// ═══════════════════════════════════════════════════════════
const unsigned long PUMP_STOP_DELAY = 10000;
unsigned long pumpStopTime  = 0;
bool          pumpRunning   = false;
unsigned long pumpStartTime = 0;
unsigned long currentOnTime = 0;
unsigned long pumpCheckTime = 0;

int currentPWM_A = 0;
int currentPWM_B = 0;

// ═══════════════════════════════════════════════════════════
//  ค่าเป้าหมาย
// ═══════════════════════════════════════════════════════════
float ecTarget   = 1800.0;
float distTarget = 15.0;

// ═══════════════════════════════════════════════════════════
//  สถานะระบบ
// ═══════════════════════════════════════════════════════════
bool          systemReady = false;
unsigned long bootTime    = 0;

// ═══════════════════════════════════════════════════════════
//  กราฟ EC บนจอ TFT
// ═══════════════════════════════════════════════════════════
int startX = 25, endX = 220;
int topY   = 90, bottomY = 165;
int graphX = startX;
int lastYec = 165;

// ═══════════════════════════════════════════════════════════
//  Timer แสดงผล / ส่งข้อมูล ทุก 2500ms
// ═══════════════════════════════════════════════════════════
unsigned long printTime = 0;
const unsigned int printInterval = 2500;

// MQTT Client
WiFiClient   espClient;
PubSubClient mqttClient(espClient);

bool configUpdated = false;

// ─────────────────────────────────────────────────────────
//  cooldown สำหรับ WiFi reconnect และ MQTT reconnect
// ─────────────────────────────────────────────────────────
unsigned long lastMqttRetry = 0;
unsigned long lastWifiRetry = 0;


// ═══════════════════════════════════════════════════════════
//  บันทึกค่า ecTarget / distTarget ลง NETPIE Shadow
// ═══════════════════════════════════════════════════════════
void saveTargetsToShadow() {
  if (!mqttClient.connected()) return;
  String payload = "{\"data\":{"
    "\"ecTarget\":"   + String(ecTarget,   1) + ","
    "\"distTarget\":" + String(distTarget, 1) +
    "}}";
  mqttClient.publish(TOPIC_SHADOW, payload.c_str());
}


// ═══════════════════════════════════════════════════════════
//  MQTT Callback — รับข้อความจาก NETPIE
// ═══════════════════════════════════════════════════════════
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("TOPIC: " + String(topic));
  Serial.println("RAW MSG: " + msg);

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, msg)) return;

  if (String(topic) == TOPIC_CONFIG) {
    JsonVariant ecVal;
    if (doc.containsKey("data")) {
      ecVal = doc["data"]["ecTarget"];
    } else {
      ecVal = doc["ecTarget"];
    }

    JsonVariant distVal;
    if (doc.containsKey("data")) {
      distVal = doc["data"]["distTarget"];
    } else {
      distVal = doc["distTarget"];
    }

    if (!ecVal.isNull()) {
      ecTarget = ecVal.as<float>();
      configUpdated = true;
      Serial.println("ecTarget NOW: " + String(ecTarget));
    }
    if (!distVal.isNull()) {
      distTarget = distVal.as<float>();
      configUpdated = true;
    }
    saveTargetsToShadow();
  }

  if (String(topic) == TOPIC_SHADOW_GET_RESP) {
    if (configUpdated) {
      configUpdated = false;
      return;
    }
    JsonObject data = doc["data"];
    if (data.containsKey("ecTarget"))   ecTarget   = data["ecTarget"].as<float>();
    if (data.containsKey("distTarget")) distTarget = data["distTarget"].as<float>();
    Serial.println("Restored from Shadow — ecTarget: " + String(ecTarget));
  }
}


// ═══════════════════════════════════════════════════════════
//  reconnectMQTT — มี cooldown 5 วินาที
//  ป้องกัน retry ถี่เกินจน broker แบน/ตัด connection
// ═══════════════════════════════════════════════════════════
void reconnectMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  if (millis() - lastMqttRetry < 5000) return;
  lastMqttRetry = millis();

  Serial.print("MQTT connecting...");
  if (mqttClient.connect(client_id, token, secret)) {
    Serial.println("OK");
    mqttClient.subscribe(TOPIC_CONFIG);
    mqttClient.subscribe(TOPIC_SHADOW_GET_RESP);
    mqttClient.publish(TOPIC_SHADOW_GET, "");
  } else {
    Serial.println("fail rc=" + String(mqttClient.state()));
  }
}


// ═══════════════════════════════════════════════════════════
//  FreeRTOS Task: ส่งข้อมูลไป Google Sheet (Core 0)
//  มี timeout 5 วินาที ป้องกัน HTTP ค้างกวน WiFi stack
// ═══════════════════════════════════════════════════════════
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
        http.setTimeout(5000);  // timeout ป้องกัน HTTP ค้างนาน
        http.begin(url);
        http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
        http.GET();
        http.end();
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// ═══════════════════════════════════════════════════════════
//  Moving Average — เรียกทุก 100ms
// ═══════════════════════════════════════════════════════════
void updateMA() {
  ecTotal -= ecReadings[ecIdx];
  ecReadings[ecIdx] = analogRead(EC_PIN);
  ecTotal += ecReadings[ecIdx];
  ecIdx = (ecIdx + 1) % NUM_MA;
}


// ═══════════════════════════════════════════════════════════
//  แปลงแรงดัน mV → ค่า EC (µS/cm)
// ═══════════════════════════════════════════════════════════
float calcEC(int MV, float coef) {
  float x = (float)MV;
  float raw = ( 4.16e-05f * x * x * x)
            + (-3.60e-02f * x * x)
            + ( 11.69f    * x)
            +   456.31f;
  return raw / coef;
}


// ═══════════════════════════════════════════════════════════
//  ส่งข้อมูลไป Google Sheet ผ่าน HTTP GET
// ═══════════════════════════════════════════════════════════
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


// ═══════════════════════════════════════════════════════════
//  ส่งข้อมูลขึ้น NETPIE Shadow และ Feed
//  ไม่มี delay() ระหว่าง publish — buffer 1024 รองรับได้สบาย
//  ทุก publish ใช้ค่าจาก parameter ที่ snapshot มาแล้ว
//  ทำให้ค่าที่ส่งตรงกับจอ TFT 100%
// ═══════════════════════════════════════════════════════════
void sendToNetpie(float temp, int MV, float EC,
                  float ecTgt, float dist, float distTgt) {
  if (!mqttClient.connected()) return;

  unsigned long remainOn = 0;
  if (pumpRunning) {
    unsigned long elapsed = millis() - pumpStartTime;
    remainOn = (elapsed < currentOnTime) ? (currentOnTime - elapsed) / 1000 : 0;
  }

  String payload = "{\"data\":{"
    "\"ec\":"        + String(EC,   2) + ","
    "\"mv\":"        + String(MV)      + ","
    "\"temp\":"      + String(temp, 1) + ","
    "\"dist\":"      + String(dist, 1) + ","
    "\"pumpA\":"     + String(currentPWM_A) + ","
    "\"pumpB\":"     + String(currentPWM_B) + ","
    "\"onTime\":"    + String(currentOnTime / 1000) + ","
    "\"remainOn\":"  + String(remainOn) +
    "}}";

  mqttClient.publish(TOPIC_SHADOW, payload.c_str());

  // ส่ง feed ทุกตัวติดกันเลย ไม่มี delay()
  mqttClient.publish("@feed/ec",       String(EC,   2).c_str());
  mqttClient.publish("@feed/temp",     String(temp, 1).c_str());
  mqttClient.publish("@feed/dist",     String(dist, 1).c_str());
  mqttClient.publish("@feed/pumpA",    String(currentPWM_A).c_str());
  mqttClient.publish("@feed/pumpB",    String(currentPWM_B).c_str());
  mqttClient.publish("@feed/onTime",   String(currentOnTime / 1000).c_str());
  mqttClient.publish("@feed/remainOn", String(remainOn).c_str());

  Serial.println(payload);
}


// ═══════════════════════════════════════════════════════════
//  แสดงผลบนจอ TFT
// ═══════════════════════════════════════════════════════════
void drawScreen(int MV, float EC, float temp,
                float ecTgt, float dist, float distTgt) {
  tft.fillRectangle(45, 0, 220, 84, COLOR_BLACK);

  tft.drawText(45, 2,  String(MV) + " mV",    COLOR_YELLOW);
  tft.drawText(45, 16, String(temp, 1) + " C", COLOR_ORANGE);

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
  tft.drawText(45,  48, ">" + String(ecTgt, 0), COLOR_WHITE);

  unsigned int dColor;
  String dStatus;
  if (dist < 0) {
    dColor = COLOR_RED;
    tft.drawText(45, 64, "--- cm", dColor);
  } else {
    float dDiff = dist - distTgt;
    if (abs(dDiff) <= distTgt * 0.05) {
      dColor = COLOR_GREEN; dStatus = "OK";
    } else if (dDiff < 0) {
      dColor = COLOR_CYAN;  dStatus = "HI";
    } else {
      dColor = COLOR_RED;   dStatus = "LO";
    }
    tft.drawText(45,  64, String(dist, 1) + " cm", dColor);
    tft.drawText(150, 64, dStatus,                  dColor);
  }

  tft.fillRectangle(45, 75, 220, 84, COLOR_BLACK);
  tft.drawText(2, 75, "PMP:", COLOR_WHITE);

  if (!systemReady) {
    tft.drawText(45, 75, "WARMUP", COLOR_YELLOW);
  } else if (EC >= ecTgt) {
    tft.drawText(45, 75, "OFF", COLOR_RED);
  } else if (pumpRunning) {
    tft.drawText(45, 75, "ON " + String(currentOnTime / 1000) + "s", COLOR_GREEN);
  } else if (pumpStopTime > 0) {
    unsigned long remain = PUMP_STOP_DELAY - (millis() - pumpStopTime);
    tft.drawText(45, 75, "WAIT " + String(remain / 1000) + "s", COLOR_YELLOW);
  }
}


// ═══════════════════════════════════════════════════════════
//  ควบคุม Relay ปั๊ม A และ B พร้อมกัน (Active LOW)
// ═══════════════════════════════════════════════════════════
void setPeriPumps(bool onA, bool onB) {
  currentPWM_A = onA ? 1 : 0;
  currentPWM_B = onB ? 1 : 0;
  digitalWrite(RELAY_S2, onA ? LOW : HIGH);
  digitalWrite(RELAY_S3, onB ? LOW : HIGH);
}


// ═══════════════════════════════════════════════════════════
//  Logic ควบคุมปั๊มสารละลาย (Proportional Dosing)
// ═══════════════════════════════════════════════════════════
void updatePeriPumps(float EC, float ecTgt) {
  if (!systemReady) {
    setPeriPumps(false, false);
    return;
  }

  unsigned long now = millis();
  float err = (ecTgt * 0.95f) - EC;

  if (err <= 0) {
    pumpRunning = false;
    if (pumpStopTime == 0) pumpStopTime = now;
    setPeriPumps(false, false);
    return;
  }

  if (pumpStopTime > 0 && now - pumpStopTime < PUMP_STOP_DELAY) {
    setPeriPumps(false, false);
    return;
  }

  if (!pumpRunning) {
    pumpStopTime  = 0;
    float ratio = constrain(err / (ecTgt * 0.95f), 0.0f, 1.0f);
    unsigned long calcTime = (unsigned long)(ratio * 10000);

    if (calcTime < 500) {
      pumpRunning = false;
      if (pumpStopTime == 0) pumpStopTime = now;
      setPeriPumps(false, false);
      return;
    }

    currentOnTime = calcTime;
    pumpStartTime = now;
    pumpRunning   = true;
    Serial.printf("Pump ON err:%.1f onTime:%lums\n", err, currentOnTime);
  }

  unsigned long elapsed = now - pumpStartTime;
  if (elapsed < currentOnTime) {
    setPeriPumps(true, true);
  } else {
    pumpRunning  = false;
    pumpStopTime = now;
    setPeriPumps(false, false);
    Serial.println("Pump OFF — waiting next cycle");
  }
}


// ═══════════════════════════════════════════════════════════
//  วัดระยะด้วย Ultrasonic HC-SR04
// ═══════════════════════════════════════════════════════════
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


// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  delay(500);
  analogReadResolution(12);

  sensors.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(RELAY_S1, OUTPUT); digitalWrite(RELAY_S1, HIGH);
  pinMode(RELAY_S2, OUTPUT); digitalWrite(RELAY_S2, HIGH);
  pinMode(RELAY_S3, OUTPUT); digitalWrite(RELAY_S3, HIGH);
  digitalWrite(RELAY_S1, LOW);

  tft.begin();
  tft.setOrientation(1);
  tft.clear();
  tft.setFont(Terminal6x8);
  tft.drawText(2, 2, "Booting...", COLOR_WHITE);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  delay(1000);
  Serial.println(" OK: " + WiFi.localIP().toString());

  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(10);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  reconnectMQTT();

  urlMutex = xSemaphoreCreateMutex();
  xTaskCreate(sendTask, "sendTask", 8192, NULL, 1, NULL);

  tft.drawText(2, 16, "Warming up...", COLOR_YELLOW);
  Serial.println("Warming up EC sensor...");
  for (int i = 0; i < NUM_MA; i++) {
    updateMA();
    delay(100);
  }
  systemReady = true;
  bootTime    = millis();
  Serial.println("System Ready");

  tft.clear();
  tft.drawText(2,  2,  "MV:",  COLOR_WHITE);
  tft.drawText(2,  16, "T:",   COLOR_WHITE);
  tft.drawText(2,  32, "EC:",  COLOR_WHITE);
  tft.drawText(2,  48, "TGT:", COLOR_WHITE);
  tft.drawText(2,  64, "Dst:", COLOR_WHITE);
  tft.drawText(2,  75, "PMP:", COLOR_WHITE);

  tft.drawLine(startX, topY,    startX, bottomY, COLOR_WHITE);
  tft.drawLine(startX, bottomY, endX,   bottomY, COLOR_WHITE);
  tft.drawText(2,   topY - 4,    "8000", COLOR_YELLOW);
  tft.drawText(2,   bottomY - 4, " 500", COLOR_YELLOW);
  tft.drawText(180, bottomY + 4, "200s", COLOR_YELLOW);

  memset(ecReadings, 0, sizeof(ecReadings));
}


// ═══════════════════════════════════════════════════════════
//  LOOP หลัก — ไม่มี delay() เพื่อให้ปั๊มแม่นยำ
// ═══════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── WiFi reconnect — ตรวจสอบและเชื่อมต่อใหม่ถ้าหลุด ──
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiRetry > 10000) {
      lastWifiRetry = now;
      Serial.println("WiFi lost — reconnecting...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
    return;  // รอ WiFi ก่อน ยังไม่ทำงานอื่น
  }

  // ── ดูแล MQTT connection ──
  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  // ── อ่านอุณหภูมิทุก 5 วินาที ──
  if (now - tempTime >= 5000) {
    tempTime = now;
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) currentTemp = t;
  }

  // ── เก็บค่า ADC เข้า Moving Average buffer ทุก 100ms ──
  if (now - ecSampleTime >= 100) {
    ecSampleTime = now;
    updateMA();
  }

  // ── คำนวณ EC และควบคุมปั๊มทุก 100ms ──
  if (now - pumpCheckTime >= 100) {
    pumpCheckTime = now;
    float coefP = 1.0f + 0.0185f * (currentTemp - 25.0f);
    currentMV   = (int)((float)ecTotal / NUM_MA * 3300.0f / 4095.0f);
    currentEC   = calcEC(currentMV, coefP);
    updatePeriPumps(currentEC, ecTarget);
  }

  // ── แสดงผล / ส่งข้อมูล ทุก 2.5 วินาที ──
  if (now - printTime >= printInterval) {
    printTime = now;

    // snapshot ค่าทุกตัว ณ จุดนี้จุดเดียว
    // ทำให้ TFT, NETPIE, Google Sheet และ Serial ได้ค่าเดียวกัน 100%
    float snapEC   = currentEC;
    int   snapMV   = currentMV;
    float snapTemp = currentTemp;
    float snapDist = readDistance();

    // จอ TFT
    drawScreen(snapMV, snapEC, snapTemp, ecTarget, snapDist, distTarget);

    // กราฟ EC บนจอ
    int yEC = map(constrain((int)snapEC, 500, 8000), 500, 8000, bottomY, topY);
    if (graphX == startX) lastYec = yEC;
    tft.drawLine(graphX - 1, lastYec, graphX, yEC, COLOR_RED);
    lastYec = yEC;
    graphX++;
    if (graphX > endX) {
      tft.fillRectangle(startX + 1, topY, endX, bottomY - 1, COLOR_BLACK);
      graphX = startX;
    }

    // Serial Monitor
    Serial.print(snapTemp, 1); Serial.print(",");
    Serial.print(snapMV);      Serial.print(",");
    Serial.print(snapEC, 2);   Serial.print(",");
    Serial.print(ecTarget, 1); Serial.print(",");
    Serial.print(snapDist, 1); Serial.print(",");
    Serial.println(distTarget, 1);

    // ส่งขึ้น Cloud — ใช้ snap ทุกตัว ค่าตรงกับ TFT แน่นอน
    sendToSheet(snapTemp, snapMV, snapEC, ecTarget, snapDist, distTarget);
    sendToNetpie(snapTemp, snapMV, snapEC, ecTarget, snapDist, distTarget);
  }
}
