#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// ===== Wi-Fi config =====
const char *WIFI_SSID = "520卧龙凤雏";
const char *WIFI_PASS = "gsuxnfeDf26472";
const char *AP_SSID = "ESP32S3-MOTOR";
const char *AP_PASS = "12345678";

// ===== Motor driver UART =====
static const int MOTOR_UART_RX = 18;  // ESP32-S3 RX <- Driver TX
static const int MOTOR_UART_TX = 17;  // ESP32-S3 TX -> Driver RX
static const uint32_t MOTOR_BAUD = 115200;

// ===== HY-SRF05 ultrasonic =====
// 正点原子成品板上通常可直接接排针 GPIO4/GPIO5（避免使用不可用管脚）
// 如果你的板子丝印不同，只改下面这两行即可。
static const int ULTRA_TRIG_PIN = 4;
static const int ULTRA_ECHO_PIN = 5;
static const unsigned long ULTRA_TIMEOUT_US = 30000;  // ~5m max

// ===== ATK-MD0130 (ST7789, 240x240) =====
// DNESP32S3 Arduino guide mapping:
// SPILCD: CS=IO21, DC=IO40, SDA(MOSI)=IO11, SCK=IO12
// XL9555: RST=IO1_2, PWR=IO1_3
static const int LCD_CS_PIN = 21;
static const int LCD_DC_PIN = 40;
static const int LCD_SCK_PIN = 12;
static const int LCD_MOSI_PIN = 11;

static const int EXIO_SDA_PIN = 41;
static const int EXIO_SCL_PIN = 42;
static const uint8_t EXIO_ADDR = 0x20;
static const uint8_t XL9555_OUTPUT_PORT0_REG = 0x02;
static const uint8_t XL9555_OUTPUT_PORT1_REG = 0x03;
static const uint8_t XL9555_CONFIG_PORT0_REG = 0x06;
static const uint8_t XL9555_CONFIG_PORT1_REG = 0x07;
static const uint16_t XL9555_LCD_RST = (1U << 10);  // IO1_2
static const uint16_t XL9555_LCD_PWR = (1U << 11);  // IO1_3

SPIClass lcdSpi(HSPI);
Adafruit_ST7789 lcd = Adafruit_ST7789(&lcdSpi, LCD_CS_PIN, LCD_DC_PIN, -1);

WebServer server(80);
unsigned long g_lastLcdUpdateMs = 0;
const unsigned long LCD_UPDATE_INTERVAL_MS = 300;
bool g_qmaReady = false;
float g_prevDistanceCm = 0.0f;
unsigned long g_prevDistanceMs = 0;
bool g_hasPrevDistance = false;
float g_speedCms = 0.0f;

// ===== QMA6100P (per DNESP32S3 Arduino guide chapter 25) =====
static const uint8_t IMU_DEV_ADDR = 0x12;  // 7-bit address
static const uint8_t QMA6100P_DEVICE_ID = 0x90;
static const uint8_t QMA6100P_CHIP_ID = 0x00;
static const uint8_t QMA6100P_XOUTL = 0x01;
static const uint8_t QMA6100P_REG_RESET = 0x36;
static const uint8_t QMA6100P_REG_RANGE = 0x0F;
static const uint8_t QMA6100P_REG_BW_ODR = 0x10;
static const uint8_t QMA6100P_REG_POWER_MANAGE = 0x11;
static const float M_G = 9.80665f;

void qma6100pWriteReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(IMU_DEV_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void qma6100pReadReg(uint8_t reg, uint8_t *buf, uint16_t num) {
  uint16_t i = 0;
  Wire.beginTransmission(IMU_DEV_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)IMU_DEV_ADDR, (int)num);
  while (Wire.available() && i < num) {
    buf[i++] = (uint8_t)Wire.read();
  }
}

bool qma6100pInit() {
  uint8_t id = 0;
  qma6100pReadReg(QMA6100P_CHIP_ID, &id, 1);
  if (id != QMA6100P_DEVICE_ID) {
    Serial.printf("QMA6100P ID mismatch: 0x%02X\r\n", id);
    return false;
  }

  qma6100pWriteReg(QMA6100P_REG_RESET, 0xB6);
  delay(5);
  qma6100pWriteReg(QMA6100P_REG_RESET, 0x00);
  delay(10);

  // Vendor recommended init sequence from tutorial.
  qma6100pWriteReg(0x11, 0x80);
  qma6100pWriteReg(0x11, 0x84);
  qma6100pWriteReg(0x4A, 0x20);
  qma6100pWriteReg(0x56, 0x01);
  qma6100pWriteReg(0x5F, 0x80);
  delay(2);
  qma6100pWriteReg(0x5F, 0x00);
  delay(10);

  // 8G range, 100Hz ODR, active mode.
  qma6100pWriteReg(QMA6100P_REG_RANGE, 0x04);
  qma6100pWriteReg(QMA6100P_REG_BW_ODR, 0x02);
  qma6100pWriteReg(QMA6100P_REG_POWER_MANAGE, 0x84);
  return true;
}

void qma6100pReadRawXyz(int16_t data[3]) {
  uint8_t b[6] = {0};
  qma6100pReadReg(QMA6100P_XOUTL, b, 6);
  int16_t rawX = (int16_t)(((uint16_t)b[1] << 8) | b[0]);
  int16_t rawY = (int16_t)(((uint16_t)b[3] << 8) | b[2]);
  int16_t rawZ = (int16_t)(((uint16_t)b[5] << 8) | b[4]);
  data[0] = rawX >> 2;
  data[1] = rawY >> 2;
  data[2] = rawZ >> 2;
}

void qma6100pReadAccXyz(float acc[3]) {
  int16_t raw[3];
  qma6100pReadRawXyz(raw);
  acc[0] = (float)(raw[0] * M_G) / 1024.0f;
  acc[1] = (float)(raw[1] * M_G) / 1024.0f;
  acc[2] = (float)(raw[2] * M_G) / 1024.0f;
}

void accGetAngle(float accIn[3], float angle[2]) {
  float n = sqrtf(accIn[0] * accIn[0] + accIn[1] * accIn[1] + accIn[2] * accIn[2]);
  if (n < 0.001f) {
    angle[0] = 0.0f;
    angle[1] = 0.0f;
    return;
  }

  float norm[3];
  norm[0] = accIn[0] / n;
  norm[1] = accIn[1] / n;
  norm[2] = accIn[2] / n;

  float pitch = -atan2f(accIn[0], accIn[2]);
  angle[0] = pitch * (180.0f / 3.14159265358979323846f);

  float nn = sqrtf(norm[0] * norm[0] + norm[1] * norm[1] + norm[2] * norm[2]);
  float roll = asinf(norm[1] / nn);
  angle[1] = roll * (180.0f / 3.14159265358979323846f);
}

void updateLcdDistance() {
  if (millis() - g_lastLcdUpdateMs < LCD_UPDATE_INTERVAL_MS) {
    return;
  }
  unsigned long nowMs = millis();
  g_lastLcdUpdateMs = nowMs;

  float cm = readDistanceCm();
  float imuAccelAbs = 0.0f;

  // Refresh only data area to reduce flicker.
  lcd.fillRect(10, 90, 220, 120, ST77XX_WHITE);
  lcd.setCursor(10, 95);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_BLUE);

  if (cm < 0) {
    lcd.print("len: -- cm");
    g_hasPrevDistance = false;
    g_speedCms = 0.0f;
  } else {
    lcd.print("len: ");
    lcd.print((int)(cm + 0.5f));
    lcd.print(" cm");

    if (g_hasPrevDistance && nowMs > g_prevDistanceMs) {
      float dt = (float)(nowMs - g_prevDistanceMs) / 1000.0f;
      float instSpeed = (cm - g_prevDistanceCm) / dt;  // cm/s
      g_speedCms = 0.7f * g_speedCms + 0.3f * instSpeed;
    }
    g_prevDistanceCm = cm;
    g_prevDistanceMs = nowMs;
    g_hasPrevDistance = true;
  }

  if (g_qmaReady) {
    float acc[3];
    float angle[2];
    qma6100pReadAccXyz(acc);
    accGetAngle(acc, angle);

    lcd.setCursor(10, 135);
    lcd.print("Pitch:");
    lcd.print(angle[0], 1);
    lcd.print("   ");

    lcd.setCursor(10, 165);
    lcd.print("Roll :");
    lcd.print(angle[1], 1);
    lcd.print("   ");

    float accNorm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    imuAccelAbs = fabsf(accNorm - M_G);
  } else {
    lcd.setCursor(10, 135);
    lcd.print("QMA6100P init fail");
  }

  lcd.setCursor(10, 195);
  if (g_speedCms > 1.0f) {
    lcd.setTextColor(ST77XX_GREEN);
  } else if (g_speedCms < -1.0f) {
    lcd.setTextColor(ST77XX_RED);
  } else {
    lcd.setTextColor(ST77XX_BLUE);
  }
  lcd.print("V:");
  lcd.print(g_speedCms, 1);
  lcd.print("cm/s ");

  lcd.setTextColor(ST77XX_BLUE);
  lcd.print("A:");
  if (g_qmaReady) {
    lcd.print(imuAccelAbs, 2);
  } else {
    lcd.print("--");
  }
  lcd.print("m/s2");
}

void xl9555WriteReg(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(EXIO_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t xl9555ReadReg(uint8_t reg) {
  Wire.beginTransmission(EXIO_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)EXIO_ADDR, 1);
  if (Wire.available() > 0) {
    return (uint8_t)Wire.read();
  }
  return 0xFF;
}

void xl9555IoConfig(uint16_t pinMask, bool outputMode) {
  const bool isPort1 = pinMask > 0xFF;
  const uint8_t bitMask = (uint8_t)(pinMask >> (isPort1 ? 8 : 0));
  const uint8_t reg = isPort1 ? XL9555_CONFIG_PORT1_REG : XL9555_CONFIG_PORT0_REG;
  uint8_t cfg = xl9555ReadReg(reg);
  if (outputMode) {
    cfg &= (uint8_t)(~bitMask);
  } else {
    cfg |= bitMask;
  }
  xl9555WriteReg(reg, cfg);
}

void xl9555PinSet(uint16_t pinMask, bool highLevel) {
  const bool isPort1 = pinMask > 0xFF;
  const uint8_t bitMask = (uint8_t)(pinMask >> (isPort1 ? 8 : 0));
  const uint8_t reg = isPort1 ? XL9555_OUTPUT_PORT1_REG : XL9555_OUTPUT_PORT0_REG;
  uint8_t out = xl9555ReadReg(reg);
  if (highLevel) {
    out |= bitMask;
  } else {
    out &= (uint8_t)(~bitMask);
  }
  xl9555WriteReg(reg, out);
}

bool initLcd() {
  pinMode(LCD_DC_PIN, OUTPUT);
  digitalWrite(LCD_DC_PIN, HIGH);

  Wire.begin(EXIO_SDA_PIN, EXIO_SCL_PIN, 400000);
  xl9555IoConfig(XL9555_LCD_RST, true);
  xl9555IoConfig(XL9555_LCD_PWR, true);
  xl9555PinSet(XL9555_LCD_PWR, true);
  xl9555PinSet(XL9555_LCD_RST, true);

  // ATK-MD0130 manual requires hardware reset:
  // RST low >=10us, then high and wait >=120ms.
  xl9555PinSet(XL9555_LCD_RST, false);
  delayMicroseconds(20);
  xl9555PinSet(XL9555_LCD_RST, true);
  delay(120);

  lcdSpi.begin(LCD_SCK_PIN, -1, LCD_MOSI_PIN, LCD_CS_PIN);
  lcd.init(240, 240);
  lcd.setRotation(0);
  lcd.fillScreen(ST77XX_BLACK);
  delay(200);
  lcd.fillScreen(ST77XX_RED);
  delay(200);
  lcd.fillScreen(ST77XX_GREEN);
  delay(200);
  lcd.fillScreen(ST77XX_BLUE);
  delay(200);
  lcd.fillScreen(ST77XX_WHITE);
  lcd.setTextWrap(false);
  lcd.setTextColor(ST77XX_RED);
  lcd.setTextSize(2);
  lcd.setCursor(10, 20);
  lcd.print("ESP32-S3 CAR");
  lcd.setCursor(10, 55);
  lcd.print("Distance + IMU");
  return true;
}

int clampValue(int value, int minV, int maxV) {
  if (value < minV) return minV;
  if (value > maxV) return maxV;
  return value;
}

void sendMotorCommand(const String &cmd) {
  Serial2.print(cmd);
  Serial.println("[TX] " + cmd);
}

String buildCommand(const String &type, int m1, int m2, int m3, int m4) {
  return "$" + type + ":" + String(m1) + "," + String(m2) + "," + String(m3) + "," + String(m4) + "#";
}

float readDistanceCm() {
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, ULTRA_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  return (duration * 0.0343f) / 2.0f;
}

void handleRoot() {
  const char html[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32S3 电机控制</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 16px; background: #f5f7fb; color: #1f2937; }
    .card { background: #fff; border-radius: 12px; padding: 16px; margin-bottom: 12px; box-shadow: 0 3px 10px rgba(0,0,0,.08); }
    h1 { font-size: 20px; margin: 0 0 12px; }
    h2 { font-size: 16px; margin: 0 0 10px; }
    .row { display: flex; gap: 10px; align-items: center; margin: 8px 0; }
    .row label { width: 40px; font-weight: bold; }
    .row input[type=range] { flex: 1; }
    .value { width: 56px; text-align: right; font-family: monospace; }
    .btns { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 10px; }
    button { border: 0; border-radius: 8px; padding: 10px 14px; font-size: 14px; cursor: pointer; }
    .primary { background: #2563eb; color: #fff; }
    .warn { background: #dc2626; color: #fff; }
    .muted { background: #e5e7eb; color: #111827; }
    .hint { font-size: 12px; color: #4b5563; margin-top: 8px; }
    .mode { display: flex; gap: 14px; margin-top: 6px; }
    .status { font-size: 12px; color: #065f46; margin-top: 8px; min-height: 18px; }
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32-S3 后驱小车控制</h1>
    <div class="mode">
      <label><input type="radio" name="mode" value="pwm" checked /> PWM模式</label>
      <label><input type="radio" name="mode" value="spd" /> 速度模式(编码器)</label>
    </div>
    <div class="hint">PWM范围: -3600~3600，速度范围: -1000~1000</div>
  </div>

  <div class="card">
    <h2>后驱电机 (M1 / M3)</h2>
    <div class="row"><label>M1</label><input id="m1" type="range" min="-3600" max="3600" value="0"><span class="value" id="v1">0</span></div>
    <div class="row"><label>M3</label><input id="m3" type="range" min="-3600" max="3600" value="0"><span class="value" id="v3">0</span></div>
    <div class="btns">
      <button class="primary" onclick="sendNow()">发送</button>
      <button class="warn" onclick="stopAll()">急停</button>
      <button class="muted" onclick="presetForward()">全前进</button>
      <button class="muted" onclick="presetBackward()">全后退</button>
      <button class="muted" onclick="getLen()">获取距离</button>
    </div>
    <div class="status" id="status"></div>
  </div>

  <script>
    const sliders = ["m1","m3"];
    const valueViews = ["v1","v3"];

    function modeValue() {
      return document.querySelector("input[name=mode]:checked").value;
    }

    function currentMax() {
      return modeValue() === "spd" ? 1000 : 3600;
    }

    function updateRangeForMode() {
      const max = currentMax();
      sliders.forEach(id => {
        const el = document.getElementById(id);
        el.min = -max;
        el.max = max;
        el.value = Math.max(-max, Math.min(max, parseInt(el.value, 10) || 0));
      });
      refreshValues();
    }

    function refreshValues() {
      sliders.forEach((id, i) => {
        valueViews[i] && (document.getElementById(valueViews[i]).textContent = document.getElementById(id).value);
      });
    }

    function setAll(v1, v3) {
      document.getElementById("m1").value = v1;
      document.getElementById("m3").value = v3;
      refreshValues();
    }

    function presetForward() { const s = modeValue()==="spd" ? 300 : 1200; setAll(s, s); sendNow(); }
    function presetBackward(){ const s = modeValue()==="spd" ? -300 : -1200; setAll(s, s); sendNow(); }

    function stopAll() {
      setAll(0,0);
      sendNow();
    }

    async function sendNow() {
      const payload = {
        mode: modeValue(),
        m1: parseInt(document.getElementById("m1").value, 10),
        m3: parseInt(document.getElementById("m3").value, 10)
      };

      const res = await fetch("/api/drive", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload)
      });
      const data = await res.json();
      document.getElementById("status").textContent = data.ok
        ? ("已发送: " + data.command)
        : ("发送失败: " + (data.error || "unknown"));
    }

    async function getLen() {
      const cmdRes = await fetch("/api/cmd?cmd=get_len");
      const txt = await cmdRes.text();
      document.getElementById("status").textContent = "超声波: " + txt;
    }

    sliders.forEach(id => document.getElementById(id).addEventListener("input", refreshValues));
    document.querySelectorAll("input[name=mode]").forEach(el => el.addEventListener("change", updateRangeForMode));
    updateRangeForMode();
  </script>
</body>
</html>
)HTML";

  server.send(200, "text/html; charset=utf-8", html);
}

void handleDrive() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing body\"}");
    return;
  }

  String body = server.arg("plain");
  body.replace(" ", "");
  body.replace("\n", "");
  body.replace("\r", "");

  auto getInt = [&](const String &key, int defaultValue) -> int {
    int keyPos = body.indexOf("\"" + key + "\":");
    if (keyPos < 0) return defaultValue;
    int colonPos = body.indexOf(':', keyPos);
    if (colonPos < 0) return defaultValue;
    int endPos = body.indexOf(',', colonPos);
    if (endPos < 0) endPos = body.indexOf('}', colonPos);
    if (endPos < 0) return defaultValue;
    String val = body.substring(colonPos + 1, endPos);
    return val.toInt();
  };

  bool isSpd = body.indexOf("\"mode\":\"spd\"") >= 0;
  int minV = isSpd ? -1000 : -3600;
  int maxV = isSpd ? 1000 : 3600;

  // Rear drive only: M1 and M3.
  int m1 = clampValue(getInt("m1", 0), minV, maxV);
  int m3 = clampValue(getInt("m3", 0), minV, maxV);
  int m2 = 0;
  int m4 = 0;

  String command = buildCommand(isSpd ? "spd" : "pwm", m1, m2, m3, m4);
  sendMotorCommand(command);

  String resp = "{\"ok\":true,\"command\":\"" + command + "\"}";
  server.send(200, "application/json", resp);
}

void handleCmd() {
  String cmd = server.arg("cmd");
  cmd.toLowerCase();

  if (cmd == "get_len" || cmd == "len" || cmd == "distance") {
    float cm = readDistanceCm();
    if (cm < 0) {
      server.send(200, "text/plain; charset=utf-8", "len:-1cm");
      return;
    }
    int cmInt = (int)(cm + 0.5f);
    server.send(200, "text/plain; charset=utf-8", "len:" + String(cmInt) + "cm");
    return;
  }

  if (cmd == "lcd_on") {
    xl9555PinSet(XL9555_LCD_PWR, true);
    server.send(200, "text/plain; charset=utf-8", "lcd:on");
    return;
  }

  if (cmd == "lcd_off") {
    xl9555PinSet(XL9555_LCD_PWR, false);
    server.send(200, "text/plain; charset=utf-8", "lcd:off");
    return;
  }

  if (cmd == "lcd_black" || cmd == "lcd_red" || cmd == "lcd_green" || cmd == "lcd_blue") {
    uint16_t color = ST77XX_BLACK;
    if (cmd == "lcd_red") color = ST77XX_RED;
    if (cmd == "lcd_green") color = ST77XX_GREEN;
    if (cmd == "lcd_blue") color = ST77XX_BLUE;
    lcd.fillScreen(color);
    server.send(200, "text/plain; charset=utf-8", "lcd:ok");
    return;
  }

  server.send(400, "text/plain; charset=utf-8", "unknown_cmd");
}

bool connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
  unsigned long startMs = millis();
  const unsigned long timeoutMs = 15000;

  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Wi-Fi connected. IP: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("\nWi-Fi connect failed.");
  return false;
}

void startAccessPointFallback() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  if (!ok) {
    Serial.println("AP start failed.");
    return;
  }

  Serial.print("AP started. SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP password: ");
  Serial.println(AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  Serial.printf("HY-SRF05 pins: TRIG=GPIO%d, ECHO=GPIO%d\n", ULTRA_TRIG_PIN, ULTRA_ECHO_PIN);

  bool lcdOk = initLcd();
  Serial.printf("LCD pins: CS=GPIO%d DC=GPIO%d SCK=GPIO%d MOSI=GPIO%d | XL9555 RST=IO1_2 PWR=IO1_3\n",
                LCD_CS_PIN, LCD_DC_PIN, LCD_SCK_PIN, LCD_MOSI_PIN);
  Serial.printf("LCD init: %s\n", lcdOk ? "OK" : "FAILED");
  g_qmaReady = qma6100pInit();
  Serial.printf("QMA6100P init: %s\n", g_qmaReady ? "OK" : "FAILED");

  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_UART_RX, MOTOR_UART_TX);

  bool wifiOk = connectWifi();
  if (!wifiOk) {
    startAccessPointFallback();
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/cmd", HTTP_GET, handleCmd);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.begin();

  Serial.println("Web server started.");
  if (WiFi.getMode() == WIFI_AP) {
    Serial.println("Open in phone browser: http://192.168.4.1/");
  } else {
    Serial.print("Open in phone browser: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
  }
}

void loop() {
  server.handleClient();
  updateLcdDistance();
}
