#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ArduinoOTA.h>

// ===== Wi-Fi config =====
const char *WIFI_SSID = "520卧龙凤雏";
const char *WIFI_PASS = "gsuxnfeDf26472";
const char *AP_SSID = "ESP32S3-MOTOR";
const char *AP_PASS = "12345678";

// ===== ArduinoOTA（与 IDE 网络上传密码一致）=====
const char *OTA_HOSTNAME = "xiaoche-ota";
const char *OTA_PASSWORD = "xiaocheota";

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
static const uint8_t XL9555_INPUT_PORT0_REG = 0x00;
static const uint8_t XL9555_INPUT_PORT1_REG = 0x01;
static const uint16_t XL9555_LCD_RST = (1U << 10);  // IO1_2
static const uint16_t XL9555_LCD_PWR = (1U << 11);  // IO1_3
static const uint16_t XL9555_KEY0 = (1U << 15);     // IO1_7

SPIClass lcdSpi(HSPI);
Adafruit_ST7789 lcd = Adafruit_ST7789(&lcdSpi, LCD_CS_PIN, LCD_DC_PIN, -1);

WebServer server(80);
unsigned long g_lastLcdUpdateMs = 0;
const unsigned long LCD_UPDATE_INTERVAL_MS = 300;
bool g_qmaReady = false;
float g_speedCms = 0.0f;
bool g_mspdOnline = false;  // true when recent $MTEP frames received (used for speed display/API)
String g_motorRxLine;
unsigned long g_lastMspdMs = 0;
const unsigned long MSPD_TIMEOUT_MS = 1200;

// ===== Speed from $MTEP (10 ms encoder delta; no need for driver $wdiameter / MSPD) =====
// Tune ENCODER_PULSES_PER_WHEEL_REV to match your motor ($mline, $mphase) or by measuring one wheel turn.
static const float MTEP_WINDOW_S = 0.01f;
static const float SPEED_WHEEL_DIAMETER_CM = 6.5f;
static const float ENCODER_PULSES_PER_WHEEL_REV = 520.0f;
float g_distanceCm = -1.0f;
unsigned long g_lastUltraSampleMs = 0;
unsigned long g_lastImuSampleMs = 0;
const unsigned long ULTRA_SAMPLE_INTERVAL_MS = 70;
const unsigned long IMU_SAMPLE_INTERVAL_MS = 20;
float g_imuPitchDeg = 0.0f;
float g_imuRollDeg = 0.0f;
float g_imuAccelAbs = 0.0f;
bool g_hasImuSample = false;

// ===== WiFi real-time debug (browser + TCP JSON stream) =====
static const uint16_t DEBUG_TCP_PORT = 8888;
static const unsigned long DEBUG_TCP_JSON_INTERVAL_MS = 100;
WiFiServer g_debugTcp(DEBUG_TCP_PORT);
WiFiClient g_debugTcpClient;
unsigned long g_lastDebugTcpPushMs = 0;
enum LcdPageMode {
  LCD_PAGE_SPEED = 0,
  LCD_PAGE_CONN = 1
};
LcdPageMode g_lcdPageMode = LCD_PAGE_SPEED;
bool g_key0RawPressed = false;
bool g_key0StablePressed = false;
unsigned long g_key0DebounceMs = 0;
const unsigned long KEY_DEBOUNCE_MS = 30;

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

bool readImuData(float &pitchDeg, float &rollDeg, float &accelAbs) {
  if (!g_qmaReady) return false;
  float acc[3];
  float angle[2];
  qma6100pReadAccXyz(acc);
  accGetAngle(acc, angle);

  float accNorm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  pitchDeg = angle[0];
  rollDeg = angle[1];
  accelAbs = fabsf(accNorm - M_G);
  return true;
}

String wifiStationOrApIp() {
  wifi_mode_t m = WiFi.getMode();
  if (m == WIFI_AP || m == WIFI_AP_STA) {
    return WiFi.softAPIP().toString();
  }
  return WiFi.localIP().toString();
}

String buildLiveJson() {
  String modeStr = "STA";
  wifi_mode_t wm = WiFi.getMode();
  if (wm == WIFI_AP) {
    modeStr = "AP";
  } else if (wm == WIFI_AP_STA) {
    modeStr = "AP_STA";
  }

  String j = "{";
  j += "\"uptime_ms\":" + String(millis()) + ",";
  j += "\"heap\":" + String(ESP.getFreeHeap()) + ",";
  j += "\"wifi_mode\":\"" + modeStr + "\",";
  j += "\"ip\":\"" + wifiStationOrApIp() + "\",";
  j += "\"spd_ok\":" + String(g_mspdOnline ? "true" : "false") + ",";
  j += "\"spd_cms\":" + String(g_speedCms, 2) + ",";
  j += "\"len_cm\":";
  if (g_distanceCm >= 0.0f) {
    j += String(g_distanceCm, 1);
  } else {
    j += "-1";
  }
  j += ",";
  j += "\"imu_ok\":" + String((g_qmaReady && g_hasImuSample) ? "true" : "false") + ",";
  j += "\"pitch\":" + String(g_imuPitchDeg, 2) + ",";
  j += "\"roll\":" + String(g_imuRollDeg, 2) + ",";
  j += "\"acc\":" + String(g_imuAccelAbs, 3);
  j += "}";
  return j;
}

void handleLiveJson() {
  server.send(200, "application/json; charset=utf-8", buildLiveJson());
}

void handleDebugPage() {
  const char html[] PROGMEM = R"HTML(
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>WiFi 实时调试</title>
  <style>
    body { font-family: system-ui, sans-serif; margin: 12px; background: #111827; color: #e5e7eb; }
    a { color: #93c5fd; }
    pre { white-space: pre-wrap; word-break: break-all; font-size: 12px; background: #1f2937; padding: 10px; border-radius: 8px; }
    table { border-collapse: collapse; width: 100%; font-size: 14px; margin-bottom: 12px; }
    td { padding: 6px 8px; border-bottom: 1px solid #374151; }
    td:first-child { color: #9ca3af; width: 42%; }
    .hint { font-size: 12px; color: #9ca3af; margin: 8px 0 12px; }
  </style>
</head>
<body>
  <h2 style="margin:0 0 8px;">WiFi 实时调试</h2>
  <div class="hint">
    <a href="/">返回控制页</a>
    · JSON <a href="/api/live">/api/live</a>
    · TCP 端口 <code>8888</code>（连接后约每 100ms 一行 JSON，输入 ping 回 pong）
  </div>
  <table id="tbl"></table>
  <pre id="raw"></pre>
  <script>
    async function tick() {
      try {
        const r = await fetch("/api/live");
        const j = await r.json();
        document.getElementById("raw").textContent = JSON.stringify(j, null, 2);
        const rows = [
          ["运行时间 (ms)", j.uptime_ms],
          ["剩余堆 heap", j.heap],
          ["WiFi 模式", j.wifi_mode],
          ["IP", j.ip],
          ["编码器速度在线", j.spd_ok],
          ["速度 (cm/s)", j.spd_cms],
          ["超声波 (cm)", j.len_cm],
          ["IMU 在线", j.imu_ok],
          ["Pitch (°)", j.pitch],
          ["Roll (°)", j.roll],
          ["加速度偏差 (m/s²)", j.acc]
        ];
        document.getElementById("tbl").innerHTML = rows
          .map(([k, v]) => "<tr><td>" + k + "</td><td>" + v + "</td></tr>")
          .join("");
      } catch (e) {
        document.getElementById("raw").textContent = "请求失败";
      }
    }
    tick();
    setInterval(tick, 150);
  </script>
</body>
</html>
)HTML";
  server.send(200, "text/html; charset=utf-8", html);
}

void pollDebugTcp() {
  WiFiClient incoming = g_debugTcp.available();
  if (incoming) {
    if (g_debugTcpClient && g_debugTcpClient.connected()) {
      g_debugTcpClient.stop();
    }
    g_debugTcpClient = incoming;
    g_debugTcpClient.setNoDelay(true);
    g_debugTcpClient.println(buildLiveJson());
    g_lastDebugTcpPushMs = millis();
  }

  if (!g_debugTcpClient || !g_debugTcpClient.connected()) {
    return;
  }

  while (g_debugTcpClient.available()) {
    String line = g_debugTcpClient.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) {
      continue;
    }
    if (line.equalsIgnoreCase("ping")) {
      g_debugTcpClient.println("pong");
    }
  }

  unsigned long nowMs = millis();
  if (nowMs - g_lastDebugTcpPushMs >= DEBUG_TCP_JSON_INTERVAL_MS) {
    g_lastDebugTcpPushMs = nowMs;
    g_debugTcpClient.println(buildLiveJson());
  }
}

void processMotorLine(const String &lineIn) {
  String line = lineIn;
  line.trim();
  if (!line.startsWith("$MTEP:")) return;

  int colon = line.indexOf(':');
  int end = line.lastIndexOf('#');
  if (colon < 0 || end <= colon) return;

  String payload = line.substring(colon + 1, end);
  int c1 = payload.indexOf(',');
  int c2 = payload.indexOf(',', c1 + 1);
  int c3 = payload.indexOf(',', c2 + 1);
  if (c1 < 0 || c2 < 0 || c3 < 0) return;

  long m1 = payload.substring(0, c1).toInt();
  long m2 = payload.substring(c1 + 1, c2).toInt();
  long m3 = payload.substring(c2 + 1, c3).toInt();
  long m4 = payload.substring(c3 + 1).toInt();
  (void)m2;
  (void)m4;

  // Rear-drive: average M1 and M3 pulse delta in one 10 ms window (signed).
  float pulses10ms = ((float)m1 + (float)m3) * 0.5f;
  if (ENCODER_PULSES_PER_WHEEL_REV <= 0.1f) {
    return;
  }

  float wheelCircCm = 3.14159265358979323846f * SPEED_WHEEL_DIAMETER_CM;
  float revInWindow = pulses10ms / ENCODER_PULSES_PER_WHEEL_REV;
  float cmInWindow = revInWindow * wheelCircCm;
  float instCms = cmInWindow / MTEP_WINDOW_S;

  g_speedCms = 0.7f * g_speedCms + 0.3f * instCms;
  g_mspdOnline = true;
  g_lastMspdMs = millis();
}

void drawLcdWifiBanner() {
  lcd.fillRect(8, 50, 226, 22, ST77XX_WHITE);
  lcd.setTextSize(2);
  lcd.setCursor(10, 52);

  wifi_mode_t mode = WiFi.getMode();
  if (mode == WIFI_STA) {
    if (WiFi.status() == WL_CONNECTED) {
      lcd.setTextColor(ST77XX_GREEN);
      lcd.print("网络已连接");
    } else {
      lcd.setTextColor(ST77XX_RED);
      lcd.print("WiFi断开");
    }
  } else if (mode == WIFI_AP || mode == WIFI_AP_STA) {
    lcd.setTextColor(ST77XX_BLUE);
    lcd.print(WiFi.softAPIP().toString());
  } else {
    lcd.setTextColor(ST77XX_RED);
    lcd.print("WiFi未就绪");
  }
}

void drawLcdSpeedPage(float cm) {
  lcd.fillRect(10, 90, 220, 120, ST77XX_WHITE);
  lcd.setCursor(10, 95);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_BLUE);

  if (cm < 0) {
    lcd.print("len: -- cm");
  } else {
    lcd.print("len: ");
    lcd.print((int)(cm + 0.5f));
    lcd.print(" cm");
  }

  if (g_qmaReady && g_hasImuSample) {
    lcd.setCursor(10, 135);
    lcd.print("Pitch:");
    lcd.print(g_imuPitchDeg, 1);
    lcd.print("   ");

    lcd.setCursor(10, 165);
    lcd.print("Roll :");
    lcd.print(g_imuRollDeg, 1);
    lcd.print("   ");
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
  if (g_mspdOnline) {
    lcd.print(g_speedCms, 1);
    lcd.print("cm/s ");
  } else {
    lcd.print("-- cm/s ");
  }

  lcd.setTextColor(ST77XX_BLUE);
  lcd.print("A:");
  if (g_qmaReady) {
    lcd.print(g_imuAccelAbs, 2);
  } else {
    lcd.print("--");
  }
  lcd.print("m/s2");
}

void drawLcdConnPage() {
  lcd.fillRect(10, 90, 220, 120, ST77XX_WHITE);
  lcd.setTextSize(2);
  lcd.setTextColor(ST77XX_BLUE);

  lcd.setCursor(10, 95);
  lcd.print("Conn Check");
  lcd.setCursor(10, 125);
  lcd.print("IP:");
  lcd.print(wifiStationOrApIp());

  lcd.setCursor(10, 155);
  if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
    lcd.print("AP:");
    lcd.print(AP_SSID);
  } else if (WiFi.status() == WL_CONNECTED) {
    lcd.print("STA:OK");
  } else {
    lcd.print("STA:OFFLINE");
  }

  lcd.setCursor(10, 185);
  lcd.print("KEY0:Switch");
}

bool xl9555PinRead(uint16_t pinMask) {
  const bool isPort1 = pinMask > 0xFF;
  const uint8_t bitMask = (uint8_t)(pinMask >> (isPort1 ? 8 : 0));
  const uint8_t reg = isPort1 ? XL9555_INPUT_PORT1_REG : XL9555_INPUT_PORT0_REG;
  uint8_t inVal = xl9555ReadReg(reg);
  return (inVal & bitMask) != 0;
}

void pollKey0SwitchPage() {
  bool rawPressed = !xl9555PinRead(XL9555_KEY0);
  unsigned long now = millis();

  if (rawPressed != g_key0RawPressed) {
    g_key0RawPressed = rawPressed;
    g_key0DebounceMs = now;
  }

  if (now - g_key0DebounceMs < KEY_DEBOUNCE_MS) return;
  if (g_key0StablePressed == g_key0RawPressed) return;

  g_key0StablePressed = g_key0RawPressed;
  if (g_key0StablePressed) {
    g_lcdPageMode = (g_lcdPageMode == LCD_PAGE_SPEED) ? LCD_PAGE_CONN : LCD_PAGE_SPEED;
  }
}

void pollMotorFeedback() {
  if (g_mspdOnline && (millis() - g_lastMspdMs > MSPD_TIMEOUT_MS)) {
    g_mspdOnline = false;
  }

  while (Serial2.available() > 0) {
    char ch = (char)Serial2.read();
    if (ch == '#') {
      g_motorRxLine += ch;
      processMotorLine(g_motorRxLine);
      g_motorRxLine = "";
      continue;
    }

    if (ch == '\n' || ch == '\r') {
      continue;
    }

    g_motorRxLine += ch;
    if (g_motorRxLine.length() > 96) {
      g_motorRxLine = "";
    }
  }
}

void updateFastSensors() {
  unsigned long nowMs = millis();

  // Ultrasonic uses pulseIn (blocking), so keep interval moderate.
  if (nowMs - g_lastUltraSampleMs >= ULTRA_SAMPLE_INTERVAL_MS) {
    g_lastUltraSampleMs = nowMs;
    g_distanceCm = readDistanceCm();
  }

  // IMU I2C read can run faster.
  if (g_qmaReady && (nowMs - g_lastImuSampleMs >= IMU_SAMPLE_INTERVAL_MS)) {
    g_lastImuSampleMs = nowMs;
    if (readImuData(g_imuPitchDeg, g_imuRollDeg, g_imuAccelAbs)) {
      g_hasImuSample = true;
    }
  }
}

void updateLcdDistance() {
  if (millis() - g_lastLcdUpdateMs < LCD_UPDATE_INTERVAL_MS) {
    return;
  }
  g_lastLcdUpdateMs = millis();
  float cm = g_distanceCm;

  drawLcdWifiBanner();
  if (g_lcdPageMode == LCD_PAGE_SPEED) {
    drawLcdSpeedPage(cm);
  } else {
    drawLcdConnPage();
  }
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
  xl9555IoConfig(XL9555_KEY0, false);
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
  // WiFi 一行在 updateLcdDistance 里刷新（STA 显示「网络已连接」，热点显示 AP IP）
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
    .telemetry { margin-top: 10px; font-size: 12px; color: #1f2937; line-height: 1.7; }
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32-S3 后驱小车控制</h1>
    <div class="hint">WiFi 实时调试: <a href="/debug">打开 /debug</a> · JSON <a href="/api/live">/api/live</a> · TCP 端口 8888</div>
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
      <button class="muted" onclick="getImu()">获取IMU</button>
      <button class="muted" onclick="getVA()">获取速度+加速度</button>
      <button class="muted" id="toggleAutoBtn" onclick="toggleTelemetry()">自动刷新:开</button>
    </div>
    <div class="status" id="status"></div>
    <div class="telemetry">
      <div>超声波: <span id="lenView">--</span></div>
      <div>IMU: <span id="imuView">--</span></div>
      <div>速度+加速度: <span id="vaView">--</span></div>
    </div>
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
      document.getElementById("lenView").textContent = txt;
      document.getElementById("status").textContent = "超声波: " + txt;
    }

    async function getImu() {
      const cmdRes = await fetch("/api/cmd?cmd=get_imu");
      const txt = await cmdRes.text();
      document.getElementById("imuView").textContent = txt;
      document.getElementById("status").textContent = "IMU: " + txt;
    }

    async function getVA() {
      const cmdRes = await fetch("/api/cmd?cmd=get_va");
      const txt = await cmdRes.text();
      document.getElementById("vaView").textContent = txt;
      document.getElementById("status").textContent = "V+A: " + txt;
    }

    async function refreshTelemetry() {
      try {
        const [lenRes, imuRes, vaRes] = await Promise.all([
          fetch("/api/cmd?cmd=get_len"),
          fetch("/api/cmd?cmd=get_imu"),
          fetch("/api/cmd?cmd=get_va")
        ]);
        const lenTxt = await lenRes.text();
        const imuTxt = await imuRes.text();
        const vaTxt = await vaRes.text();
        document.getElementById("lenView").textContent = lenTxt;
        document.getElementById("imuView").textContent = imuTxt;
        document.getElementById("vaView").textContent = vaTxt;
      } catch (e) {
        document.getElementById("status").textContent = "自动刷新失败";
      }
    }

    let telemetryTimer = null;
    let telemetryEnabled = true;

    function startTelemetry() {
      if (telemetryTimer) return;
      telemetryTimer = setInterval(refreshTelemetry, 500);
    }

    function stopTelemetry() {
      if (!telemetryTimer) return;
      clearInterval(telemetryTimer);
      telemetryTimer = null;
    }

    function toggleTelemetry() {
      telemetryEnabled = !telemetryEnabled;
      if (telemetryEnabled) {
        startTelemetry();
        document.getElementById("toggleAutoBtn").textContent = "自动刷新:开";
        refreshTelemetry();
      } else {
        stopTelemetry();
        document.getElementById("toggleAutoBtn").textContent = "自动刷新:关";
      }
    }

    sliders.forEach(id => document.getElementById(id).addEventListener("input", refreshValues));
    document.querySelectorAll("input[name=mode]").forEach(el => el.addEventListener("change", updateRangeForMode));
    updateRangeForMode();
    refreshTelemetry();
    startTelemetry();
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
    float cm = g_distanceCm;
    if (cm < 0.0f) {
      server.send(200, "text/plain; charset=utf-8", "len:-1cm");
      return;
    }
    int cmInt = (int)(cm + 0.5f);
    server.send(200, "text/plain; charset=utf-8", "len:" + String(cmInt) + "cm");
    return;
  }

  if (cmd == "get_spd" || cmd == "spd" || cmd == "speed") {
    if (!g_mspdOnline) {
      server.send(200, "text/plain; charset=utf-8", "spd:offline");
      return;
    }
    server.send(200, "text/plain; charset=utf-8", "spd:" + String(g_speedCms, 1) + "cm/s");
    return;
  }

  if (cmd == "get_imu" || cmd == "imu") {
    if (!g_qmaReady || !g_hasImuSample) {
      server.send(200, "text/plain; charset=utf-8", "imu:offline");
      return;
    }
    String s = "pitch:" + String(g_imuPitchDeg, 1) +
               ",roll:" + String(g_imuRollDeg, 1) +
               ",acc:" + String(g_imuAccelAbs, 2) + "m/s2";
    server.send(200, "text/plain; charset=utf-8", s);
    return;
  }

  if (cmd == "get_va" || cmd == "va") {
    String vPart = g_mspdOnline ? String(g_speedCms, 1) + "cm/s" : "offline";
    String aPart = (g_qmaReady && g_hasImuSample) ? String(g_imuAccelAbs, 2) + "m/s2" : "offline";
    server.send(200, "text/plain; charset=utf-8", "v:" + vPart + ",a:" + aPart);
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

void initArduinoOta() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (OTA_PASSWORD != nullptr && strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] start update");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] finished, rebooting");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    unsigned int pct = total ? (progress * 100 / total) : 0;
    Serial.printf("[OTA] %u%%\r", pct);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] error=%u ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("AUTH");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("BEGIN");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("CONNECT");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("RECEIVE");
    } else if (error == OTA_END_ERROR) {
      Serial.println("END");
    } else {
      Serial.println("?");
    }
  });

  ArduinoOTA.begin();
  Serial.printf("[OTA] ArduinoOTA hostname=%s IP=%s password=%s\r\n",
                OTA_HOSTNAME, wifiStationOrApIp().c_str(),
                (OTA_PASSWORD && strlen(OTA_PASSWORD)) ? "(set)" : "(none)");
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
  g_distanceCm = readDistanceCm();
  if (g_qmaReady) {
    g_hasImuSample = readImuData(g_imuPitchDeg, g_imuRollDeg, g_imuAccelAbs);
  }

  Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_UART_RX, MOTOR_UART_TX);
  delay(50);
  // Second flag: $MTEP:M1,M2,M3,M4# (pulse delta per 10 ms). Third flag is $MSPD (needs $wdiameter on board).
  sendMotorCommand("$upload:0,1,0#");
  Serial.println("Motor upload: MTEP ON (10ms pulses), MSPD OFF");

  bool wifiOk = connectWifi();
  if (!wifiOk) {
    startAccessPointFallback();
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/debug", HTTP_GET, handleDebugPage);
  server.on("/api/live", HTTP_GET, handleLiveJson);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/cmd", HTTP_GET, handleCmd);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.begin();
  g_debugTcp.begin();

  Serial.println("Web server started.");
  if (WiFi.getMode() == WIFI_AP) {
    Serial.println("Open in phone browser: http://192.168.4.1/");
  } else {
    Serial.print("Open in phone browser: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
  }
  Serial.printf("WiFi debug: http://%s/debug   JSON: http://%s/api/live   TCP:%u (JSON lines)\r\n",
                wifiStationOrApIp().c_str(), wifiStationOrApIp().c_str(), (unsigned)DEBUG_TCP_PORT);

  initArduinoOta();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  pollKey0SwitchPage();
  pollMotorFeedback();
  pollDebugTcp();
  updateFastSensors();
  updateLcdDistance();
}
