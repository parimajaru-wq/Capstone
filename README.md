# EC Water Monitor System
ระบบวัดค่า EC (Electrical Conductivity) และระยะระดับน้ำ ด้วย ESP32  
ส่งข้อมูลขึ้น NETPIE และ Google Sheet แบบ real-time

---
 
## Libraries ที่ต้องติดตั้ง
 
ติดตั้งผ่าน Arduino Library Manager:
 
| Library | by |
|---------|----|
| TFT_22_ILI9225 | Nkawu |
| PubSubClient | Nick O'Leary |
| ArduinoJson | Benoit Blanchon |
| OneWire | Paul Stoffregen |
| DallasTemperature | Miles Burton |
 
---
 
## การตั้งค่าก่อน Upload
 
เปิดไฟล์ `.ino` แล้วกรอกค่าในส่วนนี้:
 
```cpp
// ==========================================
// WiFi
// ==========================================
const char* ssid     = "ชื่อ WiFi";
const char* password = "รหัส WiFi";
 
// ==========================================
// Google Sheet
// ==========================================
const char* scriptURL = "https://script.google.com/macros/s/YOUR_SCRIPT_ID/exec";
 
// ==========================================
// NETPIE 2020
// ==========================================
const char* mqtt_server = "broker.netpie.io";  // ไม่ต้องแก้
const int   mqtt_port   = 1883;                 // ไม่ต้องแก้
const char* client_id   = "YOUR_CLIENT_ID";
const char* token       = "YOUR_TOKEN";
const char* secret      = "YOUR_SECRET";
```
 
### วิธีได้ค่า NETPIE
 
1. เข้า [netpie.io](https://netpie.io) → สร้าง Project → สร้าง Device
2. คัดลอก **Client ID / Token / Secret** มาใส่ในโค้ด
 
### วิธีได้ค่า scriptURL
 
1. เปิด Google Sheet → Extensions → Apps Script
2. วาง code จากไฟล์ `google_apps_script.js`
3. Deploy → New deployment → Execute as: Me → Anyone
4. คัดลอก URL มาใส่ในโค้ด
 
---
 
## CONFIG ที่ปรับได้ในโค้ด
 
```cpp
float ecTarget   = 2000.0;  // µS/cm ค่า EC เป้าหมาย (ปรับได้จาก NETPIE)
float distTarget = 15.0;    // cm ระยะ sensor→ผิวน้ำ เป้าหมาย (ปรับได้จาก NETPIE)
```
 
---
 
## NETPIE Dashboard
 
### ส่ง config กลับ ESP32
 
Publish ไปที่ topic `@msg/config`:
 
```json
{"ecTarget": 2000}
{"distTarget": 15}
```
 
### ค่าที่ ESP32 ส่งขึ้น Shadow
 
| Key | ความหมาย |
|-----|----------|
| `eq3` | ค่า EC (µS/cm) ชดเชยอุณหภูมิแล้ว |
| `mV3` | ค่า mV จาก ADC |
| `temp` | อุณหภูมิ (°C) จาก DS18B20 |
| `ecTarget` | EC เป้าหมายปัจจุบัน |
| `dist` | ระยะ sensor→ผิวน้ำ (cm) |
| `distTarget` | ระยะเป้าหมายปัจจุบัน |
 
---
 
## Google Apps Script
 
```javascript
const DEFAULT_SHEET = "Run1";
 
function doGet(e) {
  const ss        = SpreadsheetApp.getActiveSpreadsheet();
  const sheetName = e.parameter.sheet || DEFAULT_SHEET;
 
  let sheet = ss.getSheetByName(sheetName);
  if (!sheet) {
    sheet = ss.insertSheet(sheetName);
  }
 
  if (sheet.getLastRow() === 0) {
    sheet.appendRow([
      "Timestamp", "Temp(°C)",
      "mV3", "eq3", "ecTarget",
      "dist sensor2water(cm)", "distTarget(cm)"
    ]);
  }
 
  const p = e.parameter;
  const row = [
    new Date(),
    parseFloat(p.temp),
    parseInt(p.mv3),
    parseFloat(p.eq3),
    parseFloat(p.ecTarget),
    parseFloat(p.dist),
    parseFloat(p.distTarget)
  ];
 
  sheet.getRange(sheet.getLastRow() + 1, 1, 1, row.length).setValues([row]);
 
  return ContentService
    .createTextOutput("OK")
    .setMimeType(ContentService.MimeType.TEXT);
}
```
 
> เปลี่ยนแท็บจาก NETPIE ได้: `{"sheet": "Run2"}`
 
---
 
## TFT แสดงผล
 
```
mV3:  xxxx mV
T:    xx.x C
EC:   xxxx.x  OK / HI / LO
TGT:  >2000
Dst:  xx.x cm  OK / HI / LO
[กราฟ EC 500–8000 µS/cm]
```
 
**EC status:** ±5% = OK (เขียว) · สูงเกิน = HI (แดง) · ต่ำเกิน = LO (ฟ้า)  
**Dist status:** ±5% = OK (เขียว) · น้ำสูงเกิน = HI (ฟ้า) · น้ำน้อยเกิน = LO (แดง)
 
---
 
## Timing
 
| ช่วงเวลา | งาน |
|---------|-----|
| ทุก 100ms | updateMA() เก็บค่า ADC sliding window 10 ค่า |
| ทุก 1000ms | คำนวณ EC, อ่าน dist, วาด TFT, ส่ง Sheet + NETPIE |
| ทุก 5000ms | DS18B20 อ่านอุณหภูมิ |
| ตลอดเวลา | mqttClient.loop() รับ config จาก NETPIE |
| FreeRTOS task | sendTask() ยิง HTTP ไป Google Sheet แยก core |
 
---
 
## สูตรคำนวณ EC
 
```
coef = 1 + 0.0185 × (temp - 25)
mV3  = ADC_avg × 3300 / 4095
eq3  = (5.2254 × mV3 + 408.59) / coef
```
 
ชดเชยอุณหภูมิ reference ที่ 25°C ด้วย DS18B20
