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
const char* ssid     = "EETAR3";
const char* password = "EETARNET";

// ═══════════════════════════════════════════════════════════
//  Google Apps Script URL สำหรับบันทึกข้อมูลลง Google Sheet
// ═══════════════════════════════════════════════════════════
const char* scriptURL = "https://script.google.com/macros/s/AKfycbxmDjtKTZ4p8QLgQQSNAfK-Xn3cYzkoTi3A3WlXApx3sIoa1yUXC2WkkJDCkGojDDjI/exec";

// ═══════════════════════════════════════════════════════════
// เปลี่ยนจาก MINUTES เป็น SECONDS
#define SET_TIME_SECONDS 8  // <--- ตั้งเป็นวินาทีได้เลย เช่น 30, 60, 120

// ═══════════════════════════════════════════════════════════
//  ตัวแปรสำหรับ Buffer ข้อมูลดิบ EC ส่ง Google Sheet
// ═══════════════════════════════════════════════════════════
const int MAX_RAW_BUFFER = 100;
float rawEcBuffer[MAX_RAW_BUFFER];
int   rawEcIndex = 0;

// ═══════════════════════════════════════════════════════════
//  การตั้งค่า NETPIE MQTT
// ═══════════════════════════════════════════════════════════
const char* mqtt_server           = "broker.netpie.io";
const int   mqtt_port             = 1883;
const char* client_id             = "7e7249ed-78ff-46c7-a78b-febf5ba8ebb3";
const char* token                 = "6cdv5bKihQjybaqzw4XqDQUKNxtLHbke";
const char* secret                = "L1K5bbVZZUpeyAncJKE7XPWr9hqdTH9Q";
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
//  การตั้งค่า SD Card (SPI ร่วมกับจอ TFT)
// ═══════════════════════════════════════════════════════════
// #include <SD.h>
// #define SD_CS_PIN 27
// const char* logFileName = "/datalog.csv"; // ชื่อไฟล์ใน SD Card

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์อุณหภูมิ DS18B20
// ═══════════════════════════════════════════════════════════
#define ONE_WIRE_BUS 15
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float         currentTemp = 25.0;
unsigned long tempTime    = 0;

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์ EC (Analog → GPIO34) — Moving Average 5 นาที
// ═══════════════════════════════════════════════════════════
#define EC_PIN 34
// 1 วินาที = 10 แซมเปิล (เพราะวัดทุก 100ms)
const int NUM_MA = SET_TIME_SECONDS * 10;
unsigned int  ecReadings[NUM_MA];
int          ecIdx        = 0;
unsigned long ecTotal      = 0;
unsigned long ecSampleTime = 0;

// ═══════════════════════════════════════════════════════════
//  Timer แยกสำหรับ จอ TFT และ NETPIE
// ═══════════════════════════════════════════════════════════
unsigned long tftTime = 0;
const unsigned int tftInterval = 5000; // จอทุก 5 วิ

unsigned long netpieTime = 0;
const unsigned int netpieInterval = 1000; // NETPIE ทุก 1 วิ

float instantEC = 0.0; // ค่า EC ล่าสุด (ไม่ผ่าน MA)
int   instantMV = 0;   // ค่า MV ล่าสุด
float currentEC = 0.0; // ค่า EC เฉลี่ย (ผ่าน MA)
int   currentMV = 0;   // ค่า MV เฉลี่ย (ผ่าน MA)

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์ระยะ Ultrasonic HC-SR04
// ═══════════════════════════════════════════════════════════
#define TRIG_PIN 13
#define ECHO_PIN 12

// ═══════════════════════════════════════════════════════════
//  Relay ควบคุมปั๊ม (เปลี่ยนเป็น Active HIGH)
// ═══════════════════════════════════════════════════════════
#define RELAY_S1  14
#define RELAY_S2  25
#define RELAY_S3  26

// ═══════════════════════════════════════════════════════════
//  ตัวแปรควบคุมปั๊มสารละลาย
// ═══════════════════════════════════════════════════════════
// 1 วินาที = 1,000 มิลลิวินาที
const unsigned long PUMP_STOP_DELAY = (unsigned long)SET_TIME_SECONDS * 1000;
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
float ecTarget   = 1500.0;
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
//  บันทึกค่าเป้าหมายลง NETPIE Shadow
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
//  MQTT Callback
// ═══════════════════════════════════════════════════════════
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  JsonDocument doc; // ArduinoJson 7 แนะนำให้ใช้ JsonDocument แทน StaticJsonDocument
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  if (String(topic) == TOPIC_CONFIG) {
    JsonVariant ecVal;
    JsonVariant distVal;

    // เปลี่ยนจาก ? : มาใช้ if-else เพื่อแก้ปัญหา Type Mismatch ใน ArduinoJson 7
    if (doc.containsKey("data")) {
      ecVal = doc["data"]["ecTarget"];
      distVal = doc["data"]["distTarget"];
    } else {
      ecVal = doc["ecTarget"];
      distVal = doc["distTarget"];
    }

    if (!ecVal.isNull()) {
      ecTarget = ecVal.as<float>();
      configUpdated = true;
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
  }
}

// ═══════════════════════════════════════════════════════════
//  reconnectMQTT
// ═══════════════════════════════════════════════════════════
void reconnectMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - lastMqttRetry < 5000) return;
  lastMqttRetry = millis();

  if (mqttClient.connect(client_id, token, secret)) {
    mqttClient.subscribe(TOPIC_CONFIG);
    mqttClient.subscribe(TOPIC_SHADOW_GET_RESP);
    mqttClient.publish(TOPIC_SHADOW_GET, "");
  }
}

// ═══════════════════════════════════════════════════════════
//  FreeRTOS Task: ส่งข้อมูลไป Google Sheet
// ═══════════════════════════════════════════════════════════
String pendingURL  = "";
bool   sendPending = false;
SemaphoreHandle_t urlMutex;

#include <WiFiClientSecure.h> 

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
        WiFiClientSecure client;
        client.setInsecure(); // สำคัญมาก 1: ข้ามการตรวจใบรับรองความปลอดภัย เพื่อให้ ESP32 คุยกับ Google ได้

        HTTPClient http;
        http.setTimeout(5000); // สำคัญมาก 2: เพิ่มเป็น 15 วินาที เพราะ Google ค่อนข้างใช้เวลาในการเขียน 100 บรรทัด
        
        // สำคัญมาก 3: ส่ง client (WiFiClientSecure) เข้าไปใน http.begin ด้วย
        if (http.begin(client, url)) { 
          http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
          
          Serial.println("[Sheet] Sending data to Google Sheets...");
          int httpCode = http.GET();
          
          if (httpCode > 0) {
            Serial.printf("[Sheet] Success! Response Code: %d\n", httpCode);
          } else {
            Serial.printf("[Sheet] Error: %s\n", http.errorToString(httpCode).c_str());
          }
          http.end();
        } else {
          Serial.println("[Sheet] Unable to connect to Google (begin failed)");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ═══════════════════════════════════════════════════════════
//  Moving Average
// ═══════════════════════════════════════════════════════════
void updateMA() {
  ecTotal -= ecReadings[ecIdx];
  ecReadings[ecIdx] = analogRead(EC_PIN);
  ecTotal += ecReadings[ecIdx];
  ecIdx = (ecIdx + 1) % NUM_MA;
}

// ═══════════════════════════════════════════════════════════
//  แปลงแรงดัน mV → ค่า EC
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
//  ส่งข้อมูลไป Google Sheet
// ═══════════════════════════════════════════════════════════
void sendToSheet(float temp, int MV, float EC,
                 float ecTgt, float dist, float distTgt, String rawData) {
  if (WiFi.status() != WL_CONNECTED) return;
  
  String url = String(scriptURL)
    + "?temp="       + String(temp,   1)
    + "&mv="         + String(MV)
    + "&ec="         + String(EC,     2)
    + "&ecTarget="   + String(ecTgt,  1)
    + "&dist="       + String(dist,   1)
    + "&distTarget=" + String(distTgt,1)
    + "&rawEc="      + rawData; 

  if (xSemaphoreTake(urlMutex, portMAX_DELAY)) {
    pendingURL  = url;
    sendPending = true;
    xSemaphoreGive(urlMutex);
  }
}

// ═══════════════════════════════════════════════════════════
//  ส่งข้อมูลขึ้น NETPIE
// ═══════════════════════════════════════════════════════════
void sendToNetpie(float temp, int MV, float EC, float instEC, 
                  float ecTgt, float dist, float distTgt, String rawData = "") {
  if (!mqttClient.connected()) return;

  unsigned long now = millis();
  unsigned long remainOn = 0;
  if (pumpRunning) {
    unsigned long elapsed = now - pumpStartTime;
    remainOn = (elapsed < currentOnTime) ? (currentOnTime - elapsed) / 1000 : 0;
  }

  // ── คำนวณเวลารอ (remainWait) หน่วยเป็นวินาที ──
  unsigned long remainWait = 0;
  if (!systemReady) {
    // ช่วง Warmup
    long w = 60 - ((now - bootTime) / 1000);
    remainWait = (w > 0) ? (unsigned long)w : 0;
  } else if (pumpStopTime > 0 && (now - pumpStopTime < PUMP_STOP_DELAY)) {
    // ช่วงรอรอบปั๊มถัดไป
    remainWait = (PUMP_STOP_DELAY - (now - pumpStopTime)) / 1000;
  }

  String payload = "{\"data\":{"
    "\"ec\":"        + String(EC,   2) + ","
    "\"instantEC\":" + String(instEC, 2) + ","
    "\"mv\":"        + String(MV)      + ","
    "\"temp\":"      + String(temp, 1) + ","
    "\"dist\":"      + String(dist, 1) + ","
    "\"pumpA\":"     + String(currentPWM_A) + ","
    "\"pumpB\":"     + String(currentPWM_B) + ","
    "\"onTime\":"    + String(currentOnTime / 1000) + ","
    "\"remainOn\":"  + String(remainOn) + ","
    "\"remainWait\":"+ String(remainWait); // <--- ส่งเข้า Shadow

  if (rawData != "") {
    payload += ",\"rawEc\":\"" + rawData + "\"";
  }
  payload += "}}";

  mqttClient.publish(TOPIC_SHADOW, payload.c_str());

  mqttClient.publish("@feed/ec",        String(EC,   2).c_str());
  mqttClient.publish("@feed/instantEC", String(instEC, 2).c_str());
  mqttClient.publish("@feed/temp",      String(temp, 1).c_str());
  mqttClient.publish("@feed/dist",      String(dist, 1).c_str());
  mqttClient.publish("@feed/pumpA",     String(currentPWM_A).c_str());
  mqttClient.publish("@feed/pumpB",     String(currentPWM_B).c_str());
  mqttClient.publish("@feed/onTime",    String(currentOnTime / 1000).c_str());
  mqttClient.publish("@feed/remainOn",  String(remainOn).c_str());
  
  // <--- ส่งเข้า Feed แยก
  mqttClient.publish("@feed/remainWait",String(remainWait).c_str()); 

  if (rawData != "") {
    mqttClient.publish("@feed/rawEc", rawData.c_str());
  }
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
  tft.drawText(45,  48, String(ecTgt, 0), COLOR_WHITE);

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
    // ระหว่าง 1 นาทีแรก จะแสดงคำว่า WARMUP
    long warmupRemain = 60 - ((millis() - bootTime) / 1000);
    tft.drawText(45, 75, "WARMUP " + String(warmupRemain) + "s", COLOR_YELLOW);
  } else if (EC >= ecTgt) {
    tft.drawText(45, 75, "OFF", COLOR_RED);
  } else if (pumpRunning) {
    tft.drawText(45, 75, "ON " + String(currentOnTime / 1000) + "s", COLOR_GREEN);
  } else if (pumpStopTime > 0) {
    unsigned long remain = PUMP_STOP_DELAY - (millis() - pumpStopTime);
    tft.drawText(45, 75, "WAIT " + String(remain / 1000) + "s", COLOR_YELLOW);
  }
  tft.fillRectangle(0, topY - 4, startX - 1, topY + 6, COLOR_BLACK);
  tft.drawText(2, topY - 4, String((int)(ecTgt + 100)), COLOR_YELLOW);
}

// ═══════════════════════════════════════════════════════════
//  ควบคุม Relay (เปลี่ยนเป็น Active HIGH)
// ═══════════════════════════════════════════════════════════
void setPeriPumps(bool onA, bool onB) {
  currentPWM_A = onA ? 1 : 0;
  currentPWM_B = onB ? 1 : 0;
  
  // เปลี่ยนตรรกะเป็น: on = HIGH, off = LOW
  digitalWrite(RELAY_S2, onA ? HIGH : LOW);
  digitalWrite(RELAY_S3, onB ? HIGH : LOW);
}

// ═══════════════════════════════════════════════════════════
//  Logic ควบคุมปั๊มสารละลาย
// ═══════════════════════════════════════════════════════════
void updatePeriPumps(float EC, float ecTgt) {
  // หากระบบยัง Warmup ไม่ครบ 1 นาที จะล็อกไม่ให้ปั๊มทำงาน
  if (!systemReady) {
    setPeriPumps(false, false);
    return;
  }

  unsigned long now = millis();
  float err = ecTgt - EC;

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
    float ratio = constrain(err / ecTgt, 0.0f, 1.0f);
    unsigned long calcTime = (unsigned long)(ratio * 20000);

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

  // เปลี่ยน Relay เป็น Active HIGH (กำหนดค่าเริ่มต้นเป็น LOW เพื่อให้ปิด)
  pinMode(RELAY_S1, OUTPUT); digitalWrite(RELAY_S1, LOW);
  pinMode(RELAY_S2, OUTPUT); digitalWrite(RELAY_S2, LOW);
  pinMode(RELAY_S3, OUTPUT); digitalWrite(RELAY_S3, LOW);

// ── เริ่มต้นการทำงาน SD Card ──
  // Serial.print("Initializing SD card...");
  // if (!SD.begin(SD_CS_PIN)) {
  //   Serial.println(" failed!");
  // } else {
  //   Serial.println(" OK!");
  //   // สร้างไฟล์และเขียนหัวตาราง (Header) หากยังไม่มีไฟล์นี้
  //   File dataFile = SD.open(logFileName, FILE_WRITE);
  //   if (dataFile) {
  //     if (dataFile.size() == 0) { // ถ้าเป็นไฟล์ใหม่ที่เพิ่งสร้าง
  //       dataFile.println("Time(ms),Temp(C),EC(uS/cm),Distance(cm)");
  //     }
  //     dataFile.close();
  //   }
  // }

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
  Serial.println(" OK");

  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(10);
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  reconnectMQTT();

  urlMutex = xSemaphoreCreateMutex();
  xTaskCreate(sendTask, "sendTask", 8192, NULL, 1, NULL);

  // ── อ่านค่าแรกมาถมให้เต็ม Array ทันที เพื่อให้มีค่า MA ตั้งแต่เปิดเครื่อง ──
  unsigned int firstRead = analogRead(EC_PIN);
  ecTotal = 0;
  for (int i = 0; i < NUM_MA; i++) {
    ecReadings[i] = firstRead;
    ecTotal += firstRead;
  }

  // ── บันทึกเวลาเริ่มเครื่อง (ยังไม่ให้ปั๊มพร้อมทำงาน) ──
  systemReady = false; 
  bootTime    = millis();

  tft.clear();
  tft.drawText(2,  2,  "MV:",  COLOR_WHITE);
  tft.drawText(2,  16, "Temp:",   COLOR_WHITE);
  tft.drawText(2,  32, "EC:",  COLOR_WHITE);
  tft.drawText(2,  48, "TGT:", COLOR_WHITE);
  tft.drawText(2,  64, "Dst:", COLOR_WHITE);
  tft.drawText(2,  75, "PMP:", COLOR_WHITE);

  tft.drawLine(startX, topY,    startX, bottomY, COLOR_WHITE);
  tft.drawLine(startX, bottomY, endX,   bottomY, COLOR_WHITE);

  tft.drawText(2,   bottomY - 4, "0", COLOR_YELLOW);
  tft.drawText(180, bottomY + 4, String(SET_TIME_SECONDS) + " sec", COLOR_YELLOW);
}

// ═══════════════════════════════════════════════════════════
//  LOOP หลัก
// ═══════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── ตรวจสอบ Warmup ครบ 1 นาที (60,000 ms) ถึงจะให้ปั๊มทำงาน ──
  if (!systemReady && (now - bootTime >= 60000)) {
    systemReady = true;
    Serial.println("Warm up 1 min finished. Pump is ready.");
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiRetry > 10000) {
      lastWifiRetry = now;
      WiFi.disconnect();
      WiFi.begin(ssid, password);
    }
    return;
  }

  if (!mqttClient.connected()) reconnectMQTT();
  mqttClient.loop();

  if (now - tempTime >= 5000) {
    tempTime = now;
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) currentTemp = t;
  }

  if (now - ecSampleTime >= 100) {
    ecSampleTime = now;
    updateMA();
  }

  if (now - pumpCheckTime >= 100) {
    pumpCheckTime = now;
    float coefP = 1.0f + 0.0185f * (currentTemp - 25.0f);
    
    currentMV   = (int)((float)ecTotal / NUM_MA * 3300.0f / 4095.0f);
    currentEC   = calcEC(currentMV, coefP);

    int lastIdx = (ecIdx == 0) ? (NUM_MA - 1) : (ecIdx - 1);
    instantMV   = (int)((float)ecReadings[lastIdx] * 3300.0f / 4095.0f);
    instantEC   = calcEC(instantMV, coefP);

    updatePeriPumps(currentEC, ecTarget);

    rawEcBuffer[rawEcIndex] = instantEC;
    rawEcIndex++;

    // ── บันทึกลง SD Card ทุก 100ms ──
    // float logDist = readDistance(); // อ่านค่าระยะห่าง
    
    // File dataFile = SD.open(logFileName, FILE_APPEND);
    // if (dataFile) {
    //   // เขียนข้อมูลคั่นด้วยลูกน้ำ (CSV Format)
    //   dataFile.print(now);
    //   dataFile.print(",");
    //   dataFile.print(currentTemp, 1);
    //   dataFile.print(",");
    //   dataFile.print(instantEC, 2); // ใช้ instantEC เพื่อเก็บค่าที่เปลี่ยนแปลงรวดเร็ว (ไม่ผ่าน MA)
    //   dataFile.print(",");
    //   dataFile.println(logDist, 1);
    //   dataFile.close();
    // } else {
    //   Serial.println("SD Write Error!");
    // }
    //

    if (rawEcIndex >= MAX_RAW_BUFFER) {

      String rawStr = "";
      rawStr.reserve(600);
      for (int i = 0; i < MAX_RAW_BUFFER; i++) {
        rawStr += String(rawEcBuffer[i], 1);
        if (i < MAX_RAW_BUFFER - 1) rawStr += ",";
      }
      
      float snapDist = readDistance();
      
      sendToSheet(currentTemp, currentMV, currentEC, ecTarget, snapDist, distTarget, rawStr);
      sendToNetpie(currentTemp, currentMV, currentEC, instantEC, ecTarget, snapDist, distTarget, rawStr);
      
      rawEcIndex = 0;
    }
  }

  if (now - netpieTime >= netpieInterval) {
    netpieTime = now;
    float snapDist = readDistance();
    sendToNetpie(currentTemp, instantMV, currentEC, instantEC, ecTarget, snapDist, distTarget, "");
  }

  if (now - tftTime >= tftInterval) {
    tftTime = now;
    float snapDist = readDistance();

    drawScreen(currentMV, currentEC, currentTemp, ecTarget, snapDist, distTarget);

    int maxGraph = (int)(ecTarget + 100);
    int yEC = map(constrain((int)currentEC, 500, maxGraph), 500, maxGraph, bottomY, topY);
    if (graphX == startX) lastYec = yEC;
    tft.drawLine(graphX - 1, lastYec, graphX, yEC, COLOR_RED);
    lastYec = yEC;
    graphX++;
    if (graphX > endX) {
      tft.fillRectangle(startX + 1, topY, endX, bottomY - 1, COLOR_BLACK);
      graphX = startX;
    }
  }
}
