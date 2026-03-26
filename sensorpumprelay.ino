#include <SPI.h>
#include <TFT_22_ILI9225.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid     = "...";
const char* password = "...";

const char* scriptURL = "...";

const char* mqtt_server           = "broker.netpie.io";
const int   mqtt_port             = 1883;
const char* client_id             = "...";
const char* token                 = "...";
const char* secret                = "...";
const char* TOPIC_CONFIG          = "@msg/ecTarget";
const char* TOPIC_SHADOW          = "@shadow/data/update";
const char* TOPIC_SHADOW_GET      = "@shadow/data/get";
const char* TOPIC_SHADOW_GET_RESP = "@shadow/data/get/response";

#define TFT_RST 4
#define TFT_RS  2
#define TFT_CS  5
#define TFT_SDI 23
#define TFT_CLK 18
#define TFT_LED 0
TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED);

#define ONE_WIRE_BUS 15
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float currentTemp  = 25.0;
unsigned long tempTime = 0;

#define EC_PIN 34
const byte    NUM_MA = 25;
unsigned int  ecReadings[NUM_MA];
byte          ecIdx        = 0;
unsigned long ecTotal      = 0;
unsigned long ecSampleTime = 0;

// ── Global EC/MV ที่ pump loop คำนวณ → ใช้ร่วมกับ print/send loop ──
float currentEC = 0.0;
int   currentMV = 0;

#define TRIG_PIN 13
#define ECHO_PIN 12

#define RELAY_S1  14
#define RELAY_S2  25
#define RELAY_S3  26

const unsigned long PUMP_STOP_DELAY = 15000;
unsigned long pumpStopTime  = 0;
bool          pumpRunning   = false;
unsigned long pumpStartTime = 0;
unsigned long currentOnTime = 0;
unsigned long pumpCheckTime = 0;

int currentPWM_A = 0;
int currentPWM_B = 0;

float ecTarget   = 1800.0;
float distTarget = 15.0;

bool          systemReady = false;
unsigned long bootTime    = 0;

int startX = 25, endX = 220;
int topY   = 90, bottomY = 165;
int graphX = startX;
int lastYec = 165;

unsigned long printTime = 0;
const unsigned int printInterval = 2500;

WiFiClient   espClient;
PubSubClient mqttClient(espClient);

bool configUpdated = false;

// ─────────────────────────────────────────────
void saveTargetsToShadow() {
  if (!mqttClient.connected()) return;
  String payload = "{\"data\":{"
    "\"ecTarget\":"   + String(ecTarget,   1) + ","
    "\"distTarget\":" + String(distTarget, 1) +
    "}}";
  mqttClient.publish(TOPIC_SHADOW, payload.c_str());
}

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

void reconnectMQTT() {
  if (mqttClient.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;

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

// ─────────────────────────────────────────────
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

// ─────────────────────────────────────────────
void updateMA() {
  ecTotal -= ecReadings[ecIdx];
  ecReadings[ecIdx] = analogRead(EC_PIN);
  ecTotal += ecReadings[ecIdx];
  ecIdx = (ecIdx + 1) % NUM_MA;
}

float calcEC(int MV, float coef) {
  float x = (float)MV;
  float raw = ( 4.16e-05f * x * x * x)
            + (-3.60e-02f * x * x)
            + ( 11.69f    * x)
            +   456.31f;
  return raw / coef;
}

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
  mqttClient.publish("@feed/ec",       String(EC,   2).c_str());
  mqttClient.publish("@feed/temp",     String(temp, 1).c_str());
  mqttClient.publish("@feed/dist",     String(dist, 1).c_str());
  mqttClient.publish("@feed/pumpA",    String(currentPWM_A).c_str());
  mqttClient.publish("@feed/pumpB",    String(currentPWM_B).c_str());
  mqttClient.publish("@feed/onTime",   String(currentOnTime / 1000).c_str());
  mqttClient.publish("@feed/remainOn", String(remainOn).c_str());

  Serial.println(payload);
}

// ─────────────────────────────────────────────
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

void setPeriPumps(bool onA, bool onB) {
  currentPWM_A = onA ? 1 : 0;
  currentPWM_B = onB ? 1 : 0;
  digitalWrite(RELAY_S2, onA ? LOW : HIGH);
  digitalWrite(RELAY_S3, onB ? LOW : HIGH);
}

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
      unsigned long calcTime = (unsigned long)(ratio * 10000); // คำนวณเวลาออกมาก่อน

      // --- ไอเดียของคุณอยู่ตรงนี้ครับ ---
      // ถ้าคำนวณแล้วเวลาเปิดปั๊มน้อยกว่า 1000 ms (1 วินาที) ให้ถือว่าถึงเป้าแล้ว ไม่ต้องทำอะไร
      if (calcTime < 500) {  
        pumpRunning = false;
        if (pumpStopTime == 0) pumpStopTime = now;
        setPeriPumps(false, false);
        return; 
      }
      // ----------------------------

      // ถ้าผ่านเงื่อนไขมาได้ (มากกว่า 1 วินาที) ค่อยสั่งปั๊มทำงาน
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

// ─────────────────────────────────────────────
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

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
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

// ─────────────────────────────────────────────
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) reconnectMQTT();
    mqttClient.loop();
  }

  unsigned long now = millis();

  // อ่านอุณหภูมิทุก 5 วินาที
  if (now - tempTime >= 5000) {
    tempTime = now;
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C) currentTemp = t;
  }

  // อัปเดต moving average ทุก 100ms
  if (now - ecSampleTime >= 100) {
    ecSampleTime = now;
    updateMA();
  }

  // คำนวณ EC จริง → เก็บ global → ควบคุมปั๊ม (ทุก 100ms)
  if (now - pumpCheckTime >= 100) {
    pumpCheckTime = now;
    float coefP = 1.0f + 0.0185f * (currentTemp - 25.0f);
    currentMV   = (int)((ecTotal / NUM_MA) * 3300.0f / 4095.0f);  // ← global
    currentEC   = calcEC(currentMV, coefP);                        // ← global
    updatePeriPumps(currentEC, ecTarget);
  }

  // ส่งข้อมูล / วาดหน้าจอ (ทุก 2500ms) — ใช้ currentEC/currentMV โดยตรง
  if (now - printTime >= printInterval) {
    printTime = now;

    float dist = readDistance();

    drawScreen(currentMV, currentEC, currentTemp, ecTarget, dist, distTarget);

    int yEC = map(constrain((int)currentEC, 500, 8000), 500, 8000, bottomY, topY);
    if (graphX == startX) lastYec = yEC;
    tft.drawLine(graphX - 1, lastYec, graphX, yEC, COLOR_RED);
    lastYec = yEC;
    graphX++;
    if (graphX > endX) {
      tft.fillRectangle(startX + 1, topY, endX, bottomY - 1, COLOR_BLACK);
      graphX = startX;
    }

    Serial.print(currentTemp, 1); Serial.print(",");
    Serial.print(currentMV);      Serial.print(",");
    Serial.print(currentEC, 2);   Serial.print(",");
    Serial.print(ecTarget, 1);    Serial.print(",");
    Serial.print(dist, 1);        Serial.print(",");
    Serial.println(distTarget, 1);

    sendToSheet(currentTemp, currentMV, currentEC, ecTarget, dist, distTarget);
    sendToNetpie(currentTemp, currentMV, currentEC, ecTarget, dist, distTarget);
  }
}
