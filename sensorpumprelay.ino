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
//  TOPIC_CONFIG          → รับค่า ecTarget / distTarget จาก Dashboard
//  TOPIC_SHADOW          → ส่งข้อมูล sensor และสถานะปั๊มขึ้น Cloud
//  TOPIC_SHADOW_GET      → ขอดึงค่าที่บันทึกไว้ล่าสุดตอน boot
//  TOPIC_SHADOW_GET_RESP → รับค่าที่ดึงกลับมา (restore หลัง reboot)
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
//  TFT_LED = 0 → ไม่ได้ควบคุมแบ็คไลท์ผ่านโค้ด (ต่อ +3.3V ตรง)
// ═══════════════════════════════════════════════════════════
#define TFT_RST 4
#define TFT_RS  2
#define TFT_CS  5
#define TFT_SDI 23
#define TFT_CLK 18
#define TFT_LED 0
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED);

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์อุณหภูมิ DS18B20 (OneWire)
//  อัปเดตทุก 5 วินาที เพื่อลด blocking time
// ═══════════════════════════════════════════════════════════
#define ONE_WIRE_BUS 15
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float         currentTemp = 25.0;   // ค่าเริ่มต้น 25°C ก่อนอ่านครั้งแรก
unsigned long tempTime    = 0;

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์ EC (Analog → GPIO34)
//  ใช้ Moving Average 25 ค่า เก็บทุก 100ms = window 2.5 วินาที
//  เพื่อลด noise จาก ADC และสัญญาณรบกวนในสารละลาย
// ═══════════════════════════════════════════════════════════
#define EC_PIN 34
const byte    NUM_MA = 25; //  Moving Average
unsigned int  ecReadings[NUM_MA];   // buffer เก็บค่า ADC ดิบ
byte          ecIdx        = 0;     // ตำแหน่งปัจจุบันใน buffer
unsigned long ecTotal      = 0;     // ผลรวมสะสม (ใช้คำนวณค่าเฉลี่ย)
unsigned long ecSampleTime = 0;

// ค่า EC และ MV ที่คำนวณแล้ว ใช้ร่วมกันระหว่าง pump loop และ print loop
float currentEC = 0.0;
int   currentMV = 0;

// ═══════════════════════════════════════════════════════════
//  เซ็นเซอร์ระยะ Ultrasonic HC-SR04
//  วัดระดับน้ำในถัง (ค่าติดลบ = อ่านไม่ได้ / เกินระยะ)
// ═══════════════════════════════════════════════════════════
#define TRIG_PIN 13
#define ECHO_PIN 12

// ═══════════════════════════════════════════════════════════
//  Relay ควบคุมปั๊ม (Active LOW)
//  RELAY_S1 = ปั๊มหลักหมุนเวียนน้ำ (เปิดตลอดเวลา)
//  RELAY_S2 = ปั๊มสารละลาย A (ปุ๋ยพาร์ท A)
//  RELAY_S3 = ปั๊มสารละลาย B (ปุ๋ยพาร์ท B)
// ═══════════════════════════════════════════════════════════
#define RELAY_S1  14
#define RELAY_S2  25
#define RELAY_S3  26

// ═══════════════════════════════════════════════════════════
//  ตัวแปรควบคุมปั๊มสารละลาย (Proportional Dosing)
//  Logic: เปิดปั๊ม N วินาที (สัดส่วนตาม error) → หยุด 10 วิ → วัดใหม่
//  PUMP_STOP_DELAY = เวลารอหลังปั๊มปิด ให้สารละลายผสมก่อนวัดซ้ำ
// ═══════════════════════════════════════════════════════════
const unsigned long PUMP_STOP_DELAY = 10000;  // ms
unsigned long pumpStopTime  = 0;
bool          pumpRunning   = false;
unsigned long pumpStartTime = 0;
unsigned long currentOnTime = 0;   // เวลาที่ปั๊มจะเปิดรอบนี้ (ms)
unsigned long pumpCheckTime = 0;

// สถานะปั๊มที่ส่งไป NETPIE (0 = ปิด, 1 = เปิด)
int currentPWM_A = 0;
int currentPWM_B = 0;

// ═══════════════════════════════════════════════════════════
//  ค่าเป้าหมาย (สามารถปรับได้จาก NETPIE Dashboard)
//  ecTarget   = ค่า EC เป้าหมาย (µS/cm)
//  distTarget = ระยะวัดระดับน้ำเป้าหมาย (cm)
// ═══════════════════════════════════════════════════════════
float ecTarget   = 1800.0;
float distTarget = 15.0;

// ═══════════════════════════════════════════════════════════
//  สถานะระบบ
//  systemReady = false ระหว่าง warmup → ปั๊มจะไม่ทำงาน
// ═══════════════════════════════════════════════════════════
bool          systemReady = false;
unsigned long bootTime    = 0;

// ═══════════════════════════════════════════════════════════
//  กราฟ EC บนจอ TFT
//  แกน X = เวลา (startX ถึง endX), แกน Y = EC (500-8000)
//  เขียนทับจากซ้ายไปขวา วนซ้ำเมื่อถึงขอบขวา
// ═══════════════════════════════════════════════════════════
int startX = 25, endX = 220;
int topY   = 90, bottomY = 165;
int graphX = startX;
int lastYec = 165;

// ═══════════════════════════════════════════════════════════
//  ความถี่การแสดงผล / ส่งข้อมูล = 2500ms (2.5 วินาที)
//  ตรงกับ window ของ Moving Average พอดี
// ═══════════════════════════════════════════════════════════
unsigned long printTime = 0;
const unsigned int printInterval = 2500;

// MQTT Client
WiFiClient   espClient;
PubSubClient mqttClient(espClient);

// Flag ป้องกัน shadow override ค่าที่เพิ่งรับจาก TOPIC_CONFIG
bool configUpdated = false;


// ═══════════════════════════════════════════════════════════
//  บันทึกค่า ecTarget / distTarget ลง NETPIE Shadow
//  เรียกทุกครั้งที่รับค่าใหม่จาก Dashboard เพื่อให้ restore ได้หลัง reboot
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
//  1) TOPIC_CONFIG       → อัปเดต ecTarget / distTarget
//  2) TOPIC_SHADOW_GET_RESP → restore ค่าหลัง reboot
// ═══════════════════════════════════════════════════════════
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("TOPIC: " + String(topic));
  Serial.println("RAW MSG: " + msg);

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, msg)) return;  // ถ้า parse ไม่ได้ให้ข้ามไป

  // ── รับค่า Target ใหม่จาก Dashboard ──
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

    // บันทึกลง Shadow ทันที เพื่อให้ restore ได้หลัง reboot
    saveTargetsToShadow();
  }

  // ── Restore ค่า Target หลัง reboot (จาก Shadow) ──
  if (String(topic) == TOPIC_SHADOW_GET_RESP) {
    // ถ้าเพิ่งรับค่าใหม่จาก CONFIG ให้ข้ามการ restore (ป้องกันค่าเก่าทับ)
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
//  เชื่อมต่อ MQTT (เรียกซ้ำได้เมื่อหลุด)
//  หลังเชื่อมสำเร็จ → subscribe topics และดึงค่าจาก Shadow ทันที
// ═══════════════════════════════════════════════════════════
void reconnectMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.print("MQTT connecting...");
  if (mqttClient.connect(client_id, token, secret)) {
    Serial.println("OK");
    mqttClient.subscribe(TOPIC_CONFIG);
    mqttClient.subscribe(TOPIC_SHADOW_GET_RESP);
    mqttClient.publish(TOPIC_SHADOW_GET, "");  // ขอดึงค่าจาก Shadow
  } else {
    Serial.println("fail rc=" + String(mqttClient.state()));
  }
}


// ═══════════════════════════════════════════════════════════
//  FreeRTOS Task: ส่งข้อมูลไป Google Sheet (Core 0)
//  ทำงานแยก task เพื่อไม่ให้ HTTP blocking รบกวน loop หลัก
//  ใช้ Mutex ป้องกัน race condition กับ loop ที่เขียน pendingURL
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
//  เพิ่มค่า ADC ใหม่เข้า buffer และหักค่าเก่าออกจาก total
//  (Sliding Window ไม่ต้องวนลูปคำนวณใหม่ทั้งหมด)
// ═══════════════════════════════════════════════════════════
void updateMA() {
  ecTotal -= ecReadings[ecIdx];           // หักค่าเก่าออก
  ecReadings[ecIdx] = analogRead(EC_PIN); // อ่านค่าใหม่
  ecTotal += ecReadings[ecIdx];           // บวกค่าใหม่เข้า
  ecIdx = (ecIdx + 1) % NUM_MA;          // เลื่อน index วนรอบ
}


// ═══════════════════════════════════════════════════════════
//  แปลงแรงดัน mV → ค่า EC (µS/cm)
//  ใช้สมการ Polynomial ที่ได้จากการ Calibrate sensor
//  coef = ตัวคูณชดเชยอุณหภูมิ (1 + 0.0185 × (T - 25))
// ═══════════════════════════════════════════════════════════
float calcEC(int MV, float coef) {
  float x = (float)MV;
  float raw = ( 4.16e-05f * x * x * x)
            + (-3.60e-02f * x * x)
            + ( 11.69f    * x)
            +   456.31f;
  return raw / coef;  // หาร coef เพื่อชดเชยอุณหภูมิ
}


// ═══════════════════════════════════════════════════════════
//  ส่งข้อมูลไป Google Sheet ผ่าน HTTP GET
//  ส่งเป็น URL parameter → Apps Script จะเขียนลง Spreadsheet
//  การส่งจริงเกิดใน sendTask() (non-blocking)
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
//  Shadow → เก็บค่าล่าสุด (ดูย้อนหลังได้)
//  Feed   → ใช้วาดกราฟใน Dashboard แบบ real-time
//  remainOn → เวลาที่ปั๊มจะเปิดอีกกี่วินาที (countdown)
// ═══════════════════════════════════════════════════════════
void sendToNetpie(float temp, int MV, float EC,
                  float ecTgt, float dist, float distTgt) {
  if (!mqttClient.connected()) return;

  // คำนวณเวลาที่เหลือก่อนปั๊มปิด
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
    "\"pumpA\":"     + String(currentPWM_A) + ","   // 1=เปิด, 0=ปิด
    "\"pumpB\":"     + String(currentPWM_B) + ","   // 1=เปิด, 0=ปิด
    "\"onTime\":"    + String(currentOnTime / 1000) + ","  // วินาทีที่เปิดรอบนี้
    "\"remainOn\":"  + String(remainOn) +                  // วินาทีที่เหลือ
    "}}";

  mqttClient.publish(TOPIC_SHADOW, payload.c_str());
  
  // *** นี่คือส่วนของ NETPIE Feed ที่ผมเผลอตัดไปในรอบก่อน นำกลับมาให้ครบแล้วครับ ***
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
//  อัปเดตเฉพาะส่วนขวา (x=45 ขึ้นไป) เพื่อไม่ลบ label ซ้าย
//  สีบ่งบอกสถานะ: เขียว=OK, แดง=HI/LO/ผิดพลาด, ฟ้า=LO
// ═══════════════════════════════════════════════════════════
void drawScreen(int MV, float EC, float temp,
                float ecTgt, float dist, float distTgt) {
  tft.fillRectangle(45, 0, 220, 84, COLOR_BLACK);  // ล้างพื้นที่ค่าเดิม

  // ── แถวที่ 1: แรงดัน mV ──
  tft.drawText(45, 2,  String(MV) + " mV",    COLOR_YELLOW);
  // ── แถวที่ 2: อุณหภูมิ ──
  tft.drawText(45, 16, String(temp, 1) + " C", COLOR_ORANGE);

  // ── แถวที่ 3-4: EC และสถานะ ──
  float ecDiff = EC - ecTgt;
  unsigned int ecColor;
  String ecStatus;
  if (abs(ecDiff) <= ecTgt * 0.05) {
    ecColor = COLOR_GREEN; ecStatus = "OK";       // อยู่ใน ±5%
  } else if (ecDiff > 0) {
    ecColor = COLOR_RED;   ecStatus = "HI";       // EC สูงเกิน
  } else {
    ecColor = COLOR_CYAN;  ecStatus = "LO";       // EC ต่ำเกิน
  }
  tft.drawText(45,  32, String(EC, 1), ecColor);
  tft.drawText(150, 32, ecStatus,      ecColor);
  tft.drawText(45,  48, ">" + String(ecTgt, 0), COLOR_WHITE);  // แสดง target

  // ── แถวที่ 5: ระยะน้ำ ──
  unsigned int dColor;
  String dStatus;
  if (dist < 0) {
    // อ่านค่าไม่ได้ (sensor ไม่เจอผิวน้ำ)
    dColor = COLOR_RED;
    tft.drawText(45, 64, "--- cm", dColor);
  } else {
    float dDiff = dist - distTgt;
    if (abs(dDiff) <= distTgt * 0.05) {
      dColor = COLOR_GREEN; dStatus = "OK";       // ระดับน้ำปกติ
    } else if (dDiff < 0) {
      dColor = COLOR_CYAN;  dStatus = "HI";       // น้ำเยอะ (ระยะสั้น = น้ำสูง)
    } else {
      dColor = COLOR_RED;   dStatus = "LO";       // น้ำน้อย
    }
    tft.drawText(45,  64, String(dist, 1) + " cm", dColor);
    tft.drawText(150, 64, dStatus,                  dColor);
  }

  // ── แถวที่ 6: สถานะปั๊ม ──
  tft.fillRectangle(45, 75, 220, 84, COLOR_BLACK);
  tft.drawText(2, 75, "PMP:", COLOR_WHITE);

  if (!systemReady) {
    tft.drawText(45, 75, "WARMUP", COLOR_YELLOW);           // กำลัง warmup
  } else if (EC >= ecTgt) {
    tft.drawText(45, 75, "OFF", COLOR_RED);                 // EC ถึง target แล้ว
  } else if (pumpRunning) {
    tft.drawText(45, 75, "ON " + String(currentOnTime / 1000) + "s", COLOR_GREEN);  // กำลังเปิด
  } else if (pumpStopTime > 0) {
    unsigned long remain = PUMP_STOP_DELAY - (millis() - pumpStopTime);
    tft.drawText(45, 75, "WAIT " + String(remain / 1000) + "s", COLOR_YELLOW);      // รอผสม
  }
}


// ═══════════════════════════════════════════════════════════
//  ควบคุม Relay ปั๊ม A และ B พร้อมกัน
//  Active LOW: LOW = เปิดปั๊ม, HIGH = ปิดปั๊ม
// ═══════════════════════════════════════════════════════════
void setPeriPumps(bool onA, bool onB) {
  currentPWM_A = onA ? 1 : 0;
  currentPWM_B = onB ? 1 : 0;
  digitalWrite(RELAY_S2, onA ? LOW : HIGH);
  digitalWrite(RELAY_S3, onB ? LOW : HIGH);
}


// ═══════════════════════════════════════════════════════════
//  Logic ควบคุมปั๊มสารละลาย (Proportional Dosing)
//
//  ขั้นตอน:
//  1) ถ้า EC >= 95% ของ target → ปิดปั๊ม, เริ่มจับเวลารอ
//  2) ถ้ายังอยู่ในช่วงรอ (PUMP_STOP_DELAY) → ปิดปั๊มต่อ
//  3) ถ้าพร้อมเปิดรอบใหม่ → คำนวณเวลาเปิดตามสัดส่วน error
//  4) ถ้าครบเวลาที่กำหนด → ปิดปั๊ม, เริ่มนับรอรอบถัดไป
// ═══════════════════════════════════════════════════════════
void updatePeriPumps(float EC, float ecTgt) {
  if (!systemReady) {
    setPeriPumps(false, false);
    return;
  }

  unsigned long now = millis();
  float err = (ecTgt * 0.95f) - EC;  // หยุดเมื่อถึง 95% เพื่อป้องกัน overshoot

  // EC สูงพอแล้ว → ปิดปั๊ม
  if (err <= 0) {
    pumpRunning = false;
    if (pumpStopTime == 0) pumpStopTime = now;
    setPeriPumps(false, false);
    return;
  }

  // อยู่ในช่วงรอให้สารละลายผสม → ยังไม่เปิดปั๊ม
  if (pumpStopTime > 0 && now - pumpStopTime < PUMP_STOP_DELAY) {
    setPeriPumps(false, false);
    return;
  }

  // เริ่มรอบใหม่ → คำนวณเวลาเปิดปั๊มตามสัดส่วน error (max 10 วินาที)
  if (!pumpRunning) {
    pumpStopTime  = 0;
    float ratio = constrain(err / (ecTgt * 0.95f), 0.0f, 1.0f);
    unsigned long calcTime = (unsigned long)(ratio * 10000); // คำนวณเวลาออกมาก่อน


    // ถ้าคำนวณแล้วเวลาเปิดปั๊มน้อยกว่า 500 ms ให้ถือว่าถึงเป้าแล้ว ไม่ต้องทำอะไร
    if (calcTime < 500) {  
      pumpRunning = false;
      if (pumpStopTime == 0) pumpStopTime = now;
      setPeriPumps(false, false);
      return; 
    }
    // -------------------------------------------------------------------------

    currentOnTime = calcTime; // ถ้าเกิน 500ms ค่อยให้ปั๊มทำงาน
    pumpStartTime = now;
    pumpRunning   = true;
    Serial.printf("Pump ON err:%.1f onTime:%lums\n", err, currentOnTime);
  }

  // ควบคุมการเปิด/ปิดตามเวลาที่คำนวณ
  unsigned long elapsed = now - pumpStartTime;
  if (elapsed < currentOnTime) {
    setPeriPumps(true, true);   // ยังไม่ครบเวลา → เปิดปั๊ม
  } else {
    pumpRunning  = false;
    pumpStopTime = now;
    setPeriPumps(false, false); // ครบเวลา → ปิดปั๊ม รอรอบถัดไป
    Serial.println("Pump OFF — waiting next cycle");
  }
}


// ═══════════════════════════════════════════════════════════
//  วัดระยะด้วย Ultrasonic HC-SR04
//  timeout 30ms (≈ 5 เมตร) ถ้าไม่ได้รับสัญญาณ return -1
// ═══════════════════════════════════════════════════════════
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1.0;                    // อ่านไม่ได้
  return (float)dur / 29.0 / 2.0;              // แปลง µs → cm
}


// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);
  delay(500);
  analogReadResolution(12);  // ESP32 ADC 12-bit (0-4095)

  sensors.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ตั้งค่า Relay เป็น HIGH (ปิด) ก่อน แล้วค่อยเปิด S1 (ปั๊มหลัก)
  pinMode(RELAY_S1, OUTPUT); digitalWrite(RELAY_S1, HIGH);
  pinMode(RELAY_S2, OUTPUT); digitalWrite(RELAY_S2, HIGH);
  pinMode(RELAY_S3, OUTPUT); digitalWrite(RELAY_S3, HIGH);
  digitalWrite(RELAY_S1, LOW);  // เปิดปั๊มหลักหมุนเวียนน้ำทันที

  tft.begin();
  tft.setOrientation(1);
  tft.clear();
  tft.setFont(Terminal6x8);
  tft.drawText(2, 2, "Booting...", COLOR_WHITE);

  // เชื่อมต่อ WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  delay(1000);
  Serial.println(" OK: " + WiFi.localIP().toString());

  // เชื่อมต่อ MQTT และดึงค่า Shadow
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  reconnectMQTT();

  // สร้าง Mutex และ Task สำหรับส่ง HTTP (Core 0)
  urlMutex = xSemaphoreCreateMutex();
  xTaskCreate(sendTask, "sendTask", 8192, NULL, 1, NULL);

  // Warmup EC Sensor — เติม buffer ให้ครบ 25 ค่าก่อนเริ่มใช้งาน
  tft.drawText(2, 16, "Warming up...", COLOR_YELLOW);
  Serial.println("Warming up EC sensor...");
  for (int i = 0; i < NUM_MA; i++) {
    updateMA();
    delay(100);
  }
  systemReady = true;
  bootTime    = millis();
  Serial.println("System Ready");

  // วาด Label หน้าจอ TFT (วาดครั้งเดียว ไม่ต้องวาดซ้ำใน loop)
  tft.clear();
  tft.drawText(2,  2,  "MV:",  COLOR_WHITE);
  tft.drawText(2,  16, "T:",   COLOR_WHITE);
  tft.drawText(2,  32, "EC:",  COLOR_WHITE);
  tft.drawText(2,  48, "TGT:", COLOR_WHITE);
  tft.drawText(2,  64, "Dst:", COLOR_WHITE);
  tft.drawText(2,  75, "PMP:", COLOR_WHITE);


  // วาดกรอบกราฟ EC พร้อม label แกน
  tft.drawLine(startX, topY,    startX, bottomY, COLOR_WHITE);  // แกน Y
  tft.drawLine(startX, bottomY, endX,   bottomY, COLOR_WHITE);  // แกน X
  tft.drawText(2,   topY - 4,    "8000", COLOR_YELLOW);         // ค่าสูงสุด
  tft.drawText(2,   bottomY - 4, " 500", COLOR_YELLOW);         // ค่าต่ำสุด
  tft.drawText(180, bottomY + 4, "200s", COLOR_YELLOW);         // ช่วงเวลา

  memset(ecReadings, 0, sizeof(ecReadings));  // reset buffer MA
}


// ═══════════════════════════════════════════════════════════
//  LOOP หลัก
//  ทุก cycle จะตรวจสอบ 4 timer แยกกัน (non-blocking)
//  ไม่มี delay() ใดๆ เพื่อให้ปั๊มทำงานแม่นยำ
// ═══════════════════════════════════════════════════════════
void loop() {
  // ── ดูแล MQTT connection ──
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
  }

  unsigned long now = millis();

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
    float coefP = 1.0f + 0.0185f * (currentTemp - 25.0f);  // ชดเชยอุณหภูมิ
    currentMV   = (int)((float)ecTotal / NUM_MA * 3300.0f / 4095.0f);
    currentEC   = calcEC(currentMV, coefP);
    updatePeriPumps(currentEC, ecTarget);
  }

  // ── แสดงผล / ส่งข้อมูล ทุก 2.5 วินาที ──
  if (now - printTime >= printInterval) {
    printTime = now;

    float dist = readDistance();

    // วาดหน้าจอและกราฟ EC
    drawScreen(currentMV, currentEC, currentTemp, ecTarget, dist, distTarget);
    int yEC = map(constrain((int)currentEC, 500, 8000), 500, 8000, bottomY, topY);
    if (graphX == startX) lastYec = yEC;
    tft.drawLine(graphX - 1, lastYec, graphX, yEC, COLOR_RED);
    lastYec = yEC;
    graphX++;
    if (graphX > endX) {
      // กราฟเต็ม → ล้างและเริ่มใหม่จากซ้าย
      tft.fillRectangle(startX + 1, topY, endX, bottomY - 1, COLOR_BLACK);
      graphX = startX;
    }

    // พิมพ์ข้อมูลลง Serial Monitor
    Serial.print(currentTemp, 1); Serial.print(",");
    Serial.print(currentMV);      Serial.print(",");
    Serial.print(currentEC, 2);   Serial.print(",");
    Serial.print(ecTarget, 1);    Serial.print(",");
    Serial.print(dist, 1);        Serial.print(",");
    Serial.println(distTarget, 1);

    // ส่งข้อมูลขึ้น Cloud
    sendToSheet(currentTemp, currentMV, currentEC, ecTarget, dist, distTarget);
    sendToNetpie(currentTemp, currentMV, currentEC, ecTarget, dist, distTarget);
  }
}
