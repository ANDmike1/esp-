#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <stdio.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ArduinoOTA.h>
#if defined(ESP32)
#include "driver/gpio.h"
#endif

// ===== newboard 5.18：AT8236×2 直驱 + MG310 霍尔 AB，无 UART 电机板 =====
static const char FIRMWARE_VERSION[] = "newboard-5.18";

// ===== Wi-Fi config =====
const char *WIFI_SSID = "520卧龙凤雏";
const char *WIFI_PASS = "gsuxnfeDf26472";
const char *AP_SSID = "ESP32S3-MOTOR";
const char *AP_PASS = "12345678";

// ===== ArduinoOTA（与 IDE 网络上传密码一致）=====
const char *OTA_HOSTNAME = "xiaoche-ota";
const char *OTA_PASSWORD = "xiaocheota";

// ===== 新电机驱动板：每路 AT8236 用 IN1/IN2；霍尔仅 AB 相（四线电机 ×2）=====
// 引脚按你板子实际引出配置：4~7 电机，9~10 / 15~16 霍尔，17~18 超声。
// 可用引出：4,5,6,7,9,10,14~20,35~39,45~48,3,46 等（勿与下列重复）。
static const int M1_IN1_PIN = 4;
static const int M1_IN2_PIN = 5;
static const int M3_IN1_PIN = 6;
static const int M3_IN2_PIN = 7;
static const int ENC_M1_A_PIN = 9;
static const int ENC_M1_B_PIN = 10;
static const int ENC_M3_A_PIN = 15;
static const int ENC_M3_B_PIN = 16;

static const uint32_t MOTOR_PWM_FREQ_HZ = 25000;
static const uint8_t MOTOR_PWM_BITS = 8;
static const uint16_t MOTOR_PWM_MAX = (1u << MOTOR_PWM_BITS) - 1u;
static const int MOTOR_PWM_CMD_MAX = 3600;

// 软件层：限频 / 反向经零 / 斜坡，减轻 AT8236 应力
static const unsigned long MOTOR_CMD_MIN_GAP_MS = 45;           // 相邻电机帧最小间隔
static const unsigned long MOTOR_REVERSAL_ZERO_DWELL_MS = 45;  // 反向前先全零，关断后等待时间
static const int MOTOR_SLEW_MAX_DELTA_PWM = 450;               // PWM 模式单帧 M1/M3 最大变化量
static const int MOTOR_SLEW_MAX_DELTA_SPD = 150;               // 速度模式单帧最大变化量
static const int MOTOR_SLEW_MAX_STEPS = 40;                  // 单次 HTTP 请求内最多分步次数

static unsigned long g_lastMotorCmdMs = 0;
static int g_lastMotorM1Sent = 0;
static int g_lastMotorM3Sent = 0;

// ===== HY-SRF05 ultrasonic（已避开电机/编码器脚）=====
static const int ULTRA_TRIG_PIN = 17;
static const int ULTRA_ECHO_PIN = 18;
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
unsigned long g_lastMspdMs = 0;
const unsigned long MSPD_TIMEOUT_MS = 1200;

static volatile int32_t g_encCountM1 = 0;
static volatile int32_t g_encCountM3 = 0;
static int32_t g_encSnapM1 = 0;
static int32_t g_encSnapM3 = 0;
static int32_t g_encDeltaM1 = 0;
static int32_t g_encDeltaM3 = 0;
static int g_motorPwmM1 = 0;
static int g_motorPwmM3 = 0;
static bool g_ledcOnPin[48] = {false};
static unsigned long g_lastEncSampleMs = 0;
static const unsigned long ENC_SAMPLE_MS = 10;

static const float SPD_PI_KP = 2.0f;           // 过大易判「超速」后把 PWM 拉到 0
static const float SPD_TO_PWM_SCALE = 3.6f;
static const int SPD_MIN_PWM = 500;          // |目标|>0 且编码器几乎不动时保底（克服静摩擦）
static const int SPD_MAX_PI_TRIM = 800;      // 限制 PI 抵消前馈的幅度
static const int SPD_STALL_ENC_DELTA = 2;    // 10ms 内脉冲数低于此视为该轮未转

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

// QMA6100P 在本工程中仅读加速度 → Pitch/Roll；水平航向由后轮编码器差速(MTEP)积分估计，每次下发指令对齐参考。
static float g_yawEstDeg = 0.0f;
static float g_yawRefDeg = 0.0f;
static int g_drvM1 = 0;
static int g_drvM3 = 0;
static int g_drvTurnL = 0;
static int g_drvTurnR = 0;
static bool g_drvIsSpd = false;
static bool g_headingHold = false;
static unsigned long g_lastHeadingCtrlMs = 0;
static const unsigned long HEADING_CTRL_MS = 80;
static const float YAW_DEG_PER_MTEP_PDIFF = 0.018f;
static const float HEADING_KP_PWM = 3.5f;
static const float HEADING_KP_SPD = 1.2f;
static const float TURN_PWM_GAIN = 900.0f;
static const float TURN_SPD_GAIN = 250.0f;

// 直行左右不对称微调：仅 tl=tr=0 时叠到 udiff。正值略增大 M3 相对 M1，用于抵消前进右偏等；后退默认 0（本车后退已直）。
static const float STRAIGHT_TRIM_FWD_PWM = 35.0f;
static const float STRAIGHT_TRIM_REV_PWM = 0.0f;
static const float STRAIGHT_TRIM_FWD_SPD = 12.0f;
static const float STRAIGHT_TRIM_REV_SPD = 0.0f;

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

// ===== USB 串口：默认静默（g_serialVerbose=false）；发 XCDBG1 打开全部调试，XCDBG0 恢复静默 =====
static bool g_serialVerbose = false;  // 上电默认静默，见 README.md
static unsigned long g_lastQuietRunLogMs = 0;
static const unsigned long QUIET_RUN_LOG_INTERVAL_MS = 2000;
static String s_usbDbgLineBuf;

#define SERIAL_VF(...) do { if (g_serialVerbose) Serial.printf(__VA_ARGS__); } while (0)

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

static float headingWrapDeg(float d) {
  while (d > 180.0f) {
    d -= 360.0f;
  }
  while (d < -180.0f) {
    d += 360.0f;
  }
  return d;
}

String wifiStationOrApIp() {
  wifi_mode_t m = WiFi.getMode();
  if (m == WIFI_AP || m == WIFI_AP_STA) {
    return WiFi.softAPIP().toString();
  }
  return WiFi.localIP().toString();
}

// 大页面字符串必须是 static（或全局），不能是函数内非 static 的 char[]，否则会整段复制到 loop 栈上导致
// Stack canary watchpoint triggered (loopTask)。
static char g_liveJsonBuf[512];

static const char *wifiModeCString() {
  wifi_mode_t wm = WiFi.getMode();
  if (wm == WIFI_AP) {
    return "AP";
  }
  if (wm == WIFI_AP_STA) {
    return "AP_STA";
  }
  return "STA";
}

static void wifiIpToBuf(char *out, size_t n) {
  wifi_mode_t m = WiFi.getMode();
  IPAddress ip = (m == WIFI_AP || m == WIFI_AP_STA) ? WiFi.softAPIP() : WiFi.localIP();
  snprintf(out, n, "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3]);
}

void buildLiveJsonTo(char *dst, size_t cap) {
  char ip[20];
  wifiIpToBuf(ip, sizeof ip);
  if (g_distanceCm >= 0.0f) {
    snprintf(dst, cap,
             "{\"uptime_ms\":%lu,\"heap\":%u,\"wifi_mode\":\"%s\",\"ip\":\"%s\","
             "\"spd_ok\":%s,\"spd_cms\":%.2f,\"len_cm\":%.1f,"
             "\"imu_ok\":%s,\"pitch\":%.2f,\"roll\":%.2f,\"acc\":%.3f,"
             "\"yaw_est\":%.2f,\"yaw_ref\":%.2f,\"hdg_hold\":%s}",
             (unsigned long)millis(), (unsigned)ESP.getFreeHeap(), wifiModeCString(), ip,
             g_mspdOnline ? "true" : "false", g_speedCms, g_distanceCm,
             (g_qmaReady && g_hasImuSample) ? "true" : "false",
             g_imuPitchDeg, g_imuRollDeg, g_imuAccelAbs,
             g_yawEstDeg, g_yawRefDeg, g_headingHold ? "true" : "false");
  } else {
    snprintf(dst, cap,
             "{\"uptime_ms\":%lu,\"heap\":%u,\"wifi_mode\":\"%s\",\"ip\":\"%s\","
             "\"spd_ok\":%s,\"spd_cms\":%.2f,\"len_cm\":-1,"
             "\"imu_ok\":%s,\"pitch\":%.2f,\"roll\":%.2f,\"acc\":%.3f,"
             "\"yaw_est\":%.2f,\"yaw_ref\":%.2f,\"hdg_hold\":%s}",
             (unsigned long)millis(), (unsigned)ESP.getFreeHeap(), wifiModeCString(), ip,
             g_mspdOnline ? "true" : "false", g_speedCms,
             (g_qmaReady && g_hasImuSample) ? "true" : "false",
             g_imuPitchDeg, g_imuRollDeg, g_imuAccelAbs,
             g_yawEstDeg, g_yawRefDeg, g_headingHold ? "true" : "false");
  }
}

void handleLiveJson() {
  buildLiveJsonTo(g_liveJsonBuf, sizeof(g_liveJsonBuf));
  server.send(200, "application/json; charset=utf-8", g_liveJsonBuf);
}

void handleDebugPage() {
  static const char html[] PROGMEM = R"HTML(
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
          ["加速度偏差 (m/s²)", j.acc],
          ["航向估计 yaw_est (°)", j.yaw_est],
          ["航向参考 yaw_ref (°)", j.yaw_ref],
          ["航向保持", j.hdg_hold]
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
    buildLiveJsonTo(g_liveJsonBuf, sizeof(g_liveJsonBuf));
    g_debugTcpClient.println(g_liveJsonBuf);
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
    buildLiveJsonTo(g_liveJsonBuf, sizeof(g_liveJsonBuf));
    g_debugTcpClient.println(g_liveJsonBuf);
  }
}

static const int8_t QUAD_LUT[16] = {0, 1, 0, -1, -1, 0, 1, 0, 1, 0, 0, -1, 0, -1, -1, 0};

static volatile uint8_t g_encAbM1 = 0;
static volatile uint8_t g_encAbM3 = 0;

static void encApplyDelta(volatile int32_t *counter, volatile uint8_t *lastAb, int pinA, int pinB) {
  const uint8_t cur = (uint8_t)((digitalRead(pinA) ? 2 : 0) | (digitalRead(pinB) ? 1 : 0));
  const uint8_t idx = (uint8_t)(((*lastAb) << 2) | cur);
  *counter += QUAD_LUT[idx & 0x0F];
  *lastAb = cur;
}

void IRAM_ATTR encM1Isr() {
  encApplyDelta(&g_encCountM1, &g_encAbM1, ENC_M1_A_PIN, ENC_M1_B_PIN);
}

void IRAM_ATTR encM3Isr() {
  encApplyDelta(&g_encCountM3, &g_encAbM3, ENC_M3_A_PIN, ENC_M3_B_PIN);
}

static void motorLedcDetachPin(int pin) {
  if (pin < 0 || pin >= 48 || !g_ledcOnPin[pin]) {
    return;
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach((uint8_t)pin);
#endif
  g_ledcOnPin[pin] = false;
}

static void motorPinForce(int pin, int level) {
  motorLedcDetachPin(pin);
#if defined(ESP32)
  gpio_reset_pin((gpio_num_t)pin);
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
  gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3);
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_FLOATING);
  gpio_set_level((gpio_num_t)pin, level ? 1 : 0);
#else
  pinMode(pin, OUTPUT);
  digitalWrite(pin, level ? HIGH : LOW);
#endif
}

static void motorPinPwm(int pin, uint16_t duty) {
  if (duty >= MOTOR_PWM_MAX) {
    motorPinForce(pin, 1);
    return;
  }
  if (duty == 0) {
    motorPinForce(pin, 0);
    return;
  }
  if (pin >= 0 && pin < 48 && !g_ledcOnPin[pin]) {
    pinMode(pin, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach((uint8_t)pin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
#else
    static int ledcCh = 0;
    ledcSetup(ledcCh, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
    ledcAttachPin(pin, ledcCh);
    ledcCh = (ledcCh + 1) % 8;
#endif
    g_ledcOnPin[pin] = true;
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite((uint8_t)pin, duty);
#else
  ledcWrite(pin, duty);
#endif
}

static uint16_t motorCmdToDuty(int cmd) {
  const int a = abs(cmd);
  const int clamped = min(a, MOTOR_PWM_CMD_MAX);
  return (uint16_t)((uint32_t)MOTOR_PWM_MAX * (uint32_t)clamped / (uint32_t)MOTOR_PWM_CMD_MAX);
}

static void motorWheelCoast(int in1, int in2) {
  motorPinForce(in1, 0);
  motorPinForce(in2, 0);
}

static void motorWheelDrive(int in1, int in2, int signedCmd) {
  if (signedCmd == 0) {
    motorWheelCoast(in1, in2);
    return;
  }
  const uint16_t duty = motorCmdToDuty(signedCmd);
  if (signedCmd > 0) {
    motorPinForce(in2, 0);
    motorPinPwm(in1, duty);
  } else {
    motorPinForce(in1, 0);
    motorPinPwm(in2, duty);
  }
}

static void motorApplyPwmDirect(int m1, int m3) {
  motorWheelDrive(M1_IN1_PIN, M1_IN2_PIN, m1);
  motorWheelDrive(M3_IN1_PIN, M3_IN2_PIN, m3);
  g_motorPwmM1 = m1;
  g_motorPwmM3 = m3;
}

static int speedCmdToPwm(int spdCmd, int32_t encDelta) {
  if (spdCmd == 0) {
    return 0;
  }

  const int feed = clampValue((int)lroundf((float)spdCmd * SPD_TO_PWM_SCALE), -MOTOR_PWM_CMD_MAX,
                              MOTOR_PWM_CMD_MAX);

  float wheelCircCm = 3.14159265358979323846f * SPEED_WHEEL_DIAMETER_CM;
  float revInWindow = (float)encDelta / ENCODER_PULSES_PER_WHEEL_REV;
  float cmInWindow = revInWindow * wheelCircCm;
  float instCms = cmInWindow / MTEP_WINDOW_S;
  const float measSpd = instCms * 10.0f;
  const float err = (float)spdCmd - measSpd;
  float pi = SPD_PI_KP * err;
  if (pi > (float)SPD_MAX_PI_TRIM) {
    pi = (float)SPD_MAX_PI_TRIM;
  }
  if (pi < -(float)SPD_MAX_PI_TRIM) {
    pi = -(float)SPD_MAX_PI_TRIM;
  }

  int pwm = clampValue((int)lroundf((float)feed + pi), -MOTOR_PWM_CMD_MAX, MOTOR_PWM_CMD_MAX);

  // 该轮命令转但几乎无脉冲：保持前馈，避免 PI 把 PWM 拉到 0（你描述的 B 不转、手拧后才动）
  if (abs(encDelta) < SPD_STALL_ENC_DELTA) {
    if (abs(pwm) < SPD_MIN_PWM) {
      pwm = (spdCmd > 0) ? SPD_MIN_PWM : -SPD_MIN_PWM;
    }
  }
  return pwm;
}

static void motorApplySpeedDirect(int m1, int m3) {
  const int p1 = speedCmdToPwm(m1, g_encDeltaM1);
  const int p3 = speedCmdToPwm(m3, g_encDeltaM3);
  motorApplyPwmDirect(p1, p3);
}

static void processEncoderDelta(int32_t m1, int32_t m3) {
  g_encDeltaM1 = m1;
  g_encDeltaM3 = m3;

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

  const float pdiff = (float)m3 - (float)m1;
  g_yawEstDeg += YAW_DEG_PER_MTEP_PDIFF * pdiff;
  g_yawEstDeg = headingWrapDeg(g_yawEstDeg);
}

static void pollEncoderSample() {
  const unsigned long now = millis();
  if (now - g_lastEncSampleMs < ENC_SAMPLE_MS) {
    return;
  }
  g_lastEncSampleMs = now;

  int32_t c1 = 0;
  int32_t c3 = 0;
  noInterrupts();
  c1 = g_encCountM1;
  c3 = g_encCountM3;
  interrupts();

  const int32_t d1 = c1 - g_encSnapM1;
  const int32_t d3 = c3 - g_encSnapM3;
  g_encSnapM1 = c1;
  g_encSnapM3 = c3;
  processEncoderDelta(d1, d3);
}

static void initMotorEncoders() {
  pinMode(ENC_M1_A_PIN, INPUT_PULLUP);
  pinMode(ENC_M1_B_PIN, INPUT_PULLUP);
  pinMode(ENC_M3_A_PIN, INPUT_PULLUP);
  pinMode(ENC_M3_B_PIN, INPUT_PULLUP);
  g_encAbM1 = (uint8_t)((digitalRead(ENC_M1_A_PIN) ? 2 : 0) | (digitalRead(ENC_M1_B_PIN) ? 1 : 0));
  g_encAbM3 = (uint8_t)((digitalRead(ENC_M3_A_PIN) ? 2 : 0) | (digitalRead(ENC_M3_B_PIN) ? 1 : 0));
  attachInterrupt(digitalPinToInterrupt(ENC_M1_A_PIN), encM1Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M1_B_PIN), encM1Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_A_PIN), encM3Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_M3_B_PIN), encM3Isr, CHANGE);

  pinMode(M1_IN1_PIN, OUTPUT);
  pinMode(M1_IN2_PIN, OUTPUT);
  pinMode(M3_IN1_PIN, OUTPUT);
  pinMode(M3_IN2_PIN, OUTPUT);
  motorWheelCoast(M1_IN1_PIN, M1_IN2_PIN);
  motorWheelCoast(M3_IN1_PIN, M3_IN2_PIN);
  g_encSnapM1 = 0;
  g_encSnapM3 = 0;
  g_lastEncSampleMs = millis();
}

void drawLcdWifiBanner() {
  lcd.fillRect(8, 50, 226, 22, ST77XX_WHITE);
  lcd.setTextSize(2);
  lcd.setCursor(10, 52);

  wifi_mode_t mode = WiFi.getMode();
  if (mode == WIFI_STA) {
    if (WiFi.status() == WL_CONNECTED) {
      lcd.setTextColor(ST77XX_GREEN);
      lcd.print("WiFi: CONNECTED");
    } else {
      lcd.setTextColor(ST77XX_RED);
      lcd.print("WiFi: DISCONNECT");
    }
  } else if (mode == WIFI_AP || mode == WIFI_AP_STA) {
    lcd.setTextColor(ST77XX_BLUE);
    lcd.print(WiFi.softAPIP().toString());
  } else {
    lcd.setTextColor(ST77XX_RED);
    lcd.print("WiFi: NOT READY");
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
  pollEncoderSample();
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
  // WiFi status row is refreshed in updateLcdDistance.
  return true;
}

int clampValue(int value, int minV, int maxV) {
  if (value < minV) return minV;
  if (value > maxV) return maxV;
  return value;
}

static void computeMixedMotorOutputs(int want_m1, int want_m3, int tl_deg, int tr_deg, bool isSpd, int *out_m1,
                                     int *out_m3) {
  const int minV = isSpd ? -1000 : -3600;
  const int maxV = isSpd ? 1000 : 3600;
  const float avg = ((float)want_m1 + (float)want_m3) * 0.5f;
  float udiff = ((float)want_m3 - (float)want_m1) * 0.5f;
  const float turnMag = ((float)tr_deg - (float)tl_deg) / 180.0f;
  const float yawErr = headingWrapDeg(g_yawRefDeg - g_yawEstDeg);
  if (isSpd) {
    udiff += turnMag * TURN_SPD_GAIN;
    // 仅当两轮都有近期脉冲时才用航向 PI，避免「一轮死转、一轮猛转」时把另一路目标打成 0
    const bool encBoth =
        abs(g_encDeltaM1) >= SPD_STALL_ENC_DELTA && abs(g_encDeltaM3) >= SPD_STALL_ENC_DELTA;
    if (encBoth) {
      udiff += yawErr * HEADING_KP_SPD;
    }
  } else {
    udiff += turnMag * TURN_PWM_GAIN;
    udiff += yawErr * HEADING_KP_PWM;
  }
  if (tl_deg == 0 && tr_deg == 0) {
    const float mid = ((float)want_m1 + (float)want_m3) * 0.5f;
    if (mid > 0.0f) {
      udiff += isSpd ? STRAIGHT_TRIM_FWD_SPD : STRAIGHT_TRIM_FWD_PWM;
    } else if (mid < 0.0f) {
      udiff += isSpd ? STRAIGHT_TRIM_REV_SPD : STRAIGHT_TRIM_REV_PWM;
    }
  }
  const float m1f = avg - udiff;
  const float m3f = avg + udiff;
  *out_m1 = clampValue((int)lroundf(m1f), minV, maxV);
  *out_m3 = clampValue((int)lroundf(m3f), minV, maxV);
}

String buildCommand(const String &type, int m1, int m2, int m3, int m4) {
  return "$" + type + ":" + String(m1) + "," + String(m2) + "," + String(m3) + "," + String(m4) + "#";
}

static void motorCommandWaitGap() {
  if (g_lastMotorCmdMs == 0) {
    return;
  }
  const unsigned long now = millis();
  if (now - g_lastMotorCmdMs < MOTOR_CMD_MIN_GAP_MS) {
    delay(MOTOR_CMD_MIN_GAP_MS - (now - g_lastMotorCmdMs));
  }
}

static bool motorOppositeSignNonZero(int from, int to) {
  if (from == 0 || to == 0) {
    return false;
  }
  return (from > 0) != (to > 0);
}

static int motorStepToward(int from, int to, int maxDelta) {
  const int d = to - from;
  if (d > maxDelta) {
    return from + maxDelta;
  }
  if (d < -maxDelta) {
    return from - maxDelta;
  }
  return to;
}

static void motorSendDriveFrame(const char *typeStr, int m1, int m3) {
  motorCommandWaitGap();
  if (m1 == 0 && m3 == 0) {
    motorApplyPwmDirect(0, 0);
  } else if (strcmp(typeStr, "spd") == 0) {
    motorApplySpeedDirect(m1, m3);
  } else {
    motorApplyPwmDirect(m1, m3);
  }
  g_lastMotorCmdMs = millis();
  g_lastMotorM1Sent = m1;
  g_lastMotorM3Sent = m3;
  if (g_serialVerbose || (strcmp(typeStr, "spd") == 0)) {
    Serial.printf("[motor] %s tgt M1=%d M3=%d | enc_d %ld,%ld | pwm %d,%d\n", typeStr, m1, m3,
                  (long)g_encDeltaM1, (long)g_encDeltaM3, g_motorPwmM1, g_motorPwmM3);
  }
}

static void tickHeadingHold() {
  if (!g_headingHold) {
    return;
  }
  const unsigned long now = millis();
  if (now - g_lastHeadingCtrlMs < HEADING_CTRL_MS) {
    return;
  }
  g_lastHeadingCtrlMs = now;

  int m1o = 0;
  int m3o = 0;
  computeMixedMotorOutputs(g_drvM1, g_drvM3, g_drvTurnL, g_drvTurnR, g_drvIsSpd, &m1o, &m3o);
  const bool spdStuckZero = g_drvIsSpd && (m1o != 0 || m3o != 0) && g_motorPwmM1 == 0 && g_motorPwmM3 == 0;
  if (m1o == g_lastMotorM1Sent && m3o == g_lastMotorM3Sent && !spdStuckZero) {
    return;
  }

  motorSendDriveFrame(g_drvIsSpd ? "spd" : "pwm", m1o, m3o);
}

static void applyMotorDriveWithGuards(bool isSpd, int m1_tgt, int m3_tgt) {
  const char *typeStr = isSpd ? "spd" : "pwm";
  const int slew = isSpd ? MOTOR_SLEW_MAX_DELTA_SPD : MOTOR_SLEW_MAX_DELTA_PWM;

  const bool rev = motorOppositeSignNonZero(g_lastMotorM1Sent, m1_tgt) ||
                   motorOppositeSignNonZero(g_lastMotorM3Sent, m3_tgt);
  if (rev) {
    motorSendDriveFrame(typeStr, 0, 0);
    delay(MOTOR_REVERSAL_ZERO_DWELL_MS);
  }

  for (int step = 0; step < MOTOR_SLEW_MAX_STEPS; ++step) {
    if (g_lastMotorM1Sent == m1_tgt && g_lastMotorM3Sent == m3_tgt) {
      return;
    }
    const int n1 = motorStepToward(g_lastMotorM1Sent, m1_tgt, slew);
    const int n3 = motorStepToward(g_lastMotorM3Sent, m3_tgt, slew);
    if (n1 == g_lastMotorM1Sent && n3 == g_lastMotorM3Sent) {
      motorSendDriveFrame(typeStr, m1_tgt, m3_tgt);
      return;
    }
    motorSendDriveFrame(typeStr, n1, n3);
    if (n1 == m1_tgt && n3 == m3_tgt) {
      return;
    }
    delay(MOTOR_CMD_MIN_GAP_MS);
  }

  if (g_lastMotorM1Sent != m1_tgt || g_lastMotorM3Sent != m3_tgt) {
    motorSendDriveFrame(typeStr, m1_tgt, m3_tgt);
    Serial.println("[motor] guard: slew step cap, forced final frame");
  }
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
  static const char html[] PROGMEM = R"HTML(
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
    .row label.wlab { width: 52px; }
    .row input[type=number].numin { width: 4.75rem; padding: 6px 8px; font-size: 14px; border-radius: 6px; border: 1px solid #cbd5e1; flex-shrink: 0; }
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
    <div class="hint">PWM -3600~3600，速度 -1000~1000。newboard-5.18：AT8236 直驱 + 霍尔 AB；航向编码器差速；左转/右转 0~180。</div>
  </div>

  <div class="card">
    <h2>后驱电机 (M1 / M3)</h2>
    <div class="row"><label>M1</label><input type="number" id="n1" class="numin" value="0" step="1" inputmode="numeric"><input id="m1" type="range" min="-3600" max="3600" value="0"><span class="value" id="v1">0</span></div>
    <div class="row"><label>M3</label><input type="number" id="n3" class="numin" value="0" step="1" inputmode="numeric"><input id="m3" type="range" min="-3600" max="3600" value="0"><span class="value" id="v3">0</span></div>
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

  <div class="card">
    <h2>转向角 (0~180)</h2>
    <div class="hint">左转 / 右转各一条；只输入数字，不配单位。与上方 M1、M3 一并点击「发送」下发。</div>
    <div class="row"><label class="wlab">左转</label><input type="number" id="ntl" class="numin" value="0" min="0" max="180" step="1" inputmode="numeric"><input id="tl" type="range" min="0" max="180" value="0"><span class="value" id="vtl">0</span></div>
    <div class="row"><label class="wlab">右转</label><input type="number" id="ntr" class="numin" value="0" min="0" max="180" step="1" inputmode="numeric"><input id="tr" type="range" min="0" max="180" value="0"><span class="value" id="vtr">0</span></div>
  </div>

  <script>
    const sliders = ["m1","m3"];
    const nums = ["n1","n3"];
    const valueViews = ["v1","v3"];
    const turnSliders = ["tl","tr"];
    const turnNums = ["ntl","ntr"];
    const turnViews = ["vtl","vtr"];

    function modeValue() {
      return document.querySelector("input[name=mode]:checked").value;
    }

    function currentMax() {
      return modeValue() === "spd" ? 1000 : 3600;
    }

    function updateRangeForMode() {
      const max = currentMax();
      sliders.forEach((id, i) => {
        const el = document.getElementById(id);
        el.min = -max;
        el.max = max;
        el.value = Math.max(-max, Math.min(max, parseInt(el.value, 10) || 0));
        const numEl = document.getElementById(nums[i]);
        numEl.min = -max;
        numEl.max = max;
        numEl.value = el.value;
      });
      refreshValues();
    }

    function syncFromNum(numId) {
      const i = nums.indexOf(numId);
      if (i < 0) return;
      const max = currentMax();
      let v = parseInt(document.getElementById(numId).value, 10);
      if (isNaN(v)) v = 0;
      v = Math.max(-max, Math.min(max, v));
      document.getElementById(sliders[i]).value = v;
      document.getElementById(numId).value = v;
      document.getElementById(valueViews[i]).textContent = String(v);
    }

    function refreshValues() {
      sliders.forEach((id, i) => {
        document.getElementById(nums[i]).value = document.getElementById(id).value;
        document.getElementById(valueViews[i]).textContent = document.getElementById(id).value;
      });
    }

    function refreshTurnValues() {
      turnSliders.forEach((id, i) => {
        document.getElementById(turnNums[i]).value = document.getElementById(id).value;
        document.getElementById(turnViews[i]).textContent = document.getElementById(id).value;
      });
    }

    function syncTurnFromNum(numId) {
      const i = turnNums.indexOf(numId);
      if (i < 0) return;
      let v = parseInt(document.getElementById(numId).value, 10);
      if (isNaN(v)) v = 0;
      v = Math.max(0, Math.min(180, v));
      document.getElementById(turnSliders[i]).value = v;
      document.getElementById(numId).value = v;
      document.getElementById(turnViews[i]).textContent = String(v);
    }

    function setTurnAll(tl, tr) {
      document.getElementById("tl").value = tl;
      document.getElementById("tr").value = tr;
      document.getElementById("ntl").value = tl;
      document.getElementById("ntr").value = tr;
      refreshTurnValues();
    }

    function setAll(v1, v3) {
      document.getElementById("m1").value = v1;
      document.getElementById("m3").value = v3;
      document.getElementById("n1").value = v1;
      document.getElementById("n3").value = v3;
      refreshValues();
    }

    function presetForward() { const s = modeValue()==="spd" ? -300 : -1200; setAll(s, s); sendNow(); }
    function presetBackward(){ const s = modeValue()==="spd" ? 300 : 1200; setAll(s, s); sendNow(); }

    function stopAll() {
      setAll(0,0);
      setTurnAll(0,0);
      sendNow();
    }

    async function sendNow() {
      syncFromNum("n1");
      syncFromNum("n3");
      syncTurnFromNum("ntl");
      syncTurnFromNum("ntr");
      const payload = {
        mode: modeValue(),
        m1: parseInt(document.getElementById("m1").value, 10),
        m3: parseInt(document.getElementById("m3").value, 10),
        tl: parseInt(document.getElementById("tl").value, 10),
        tr: parseInt(document.getElementById("tr").value, 10)
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
    nums.forEach(nid => {
      const el = document.getElementById(nid);
      el.addEventListener("change", () => syncFromNum(nid));
      el.addEventListener("blur", () => syncFromNum(nid));
      el.addEventListener("keydown", (e) => {
        if (e.key === "Enter") { e.preventDefault(); syncFromNum(nid); sendNow(); }
      });
    });
    turnSliders.forEach(id => document.getElementById(id).addEventListener("input", refreshTurnValues));
    turnNums.forEach(nid => {
      const el = document.getElementById(nid);
      el.addEventListener("change", () => syncTurnFromNum(nid));
      el.addEventListener("blur", () => syncTurnFromNum(nid));
      el.addEventListener("keydown", (e) => {
        if (e.key === "Enter") { e.preventDefault(); syncTurnFromNum(nid); sendNow(); }
      });
    });
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

  int m1 = clampValue(getInt("m1", 0), minV, maxV);
  int m3 = clampValue(getInt("m3", 0), minV, maxV);
  const int tl = clampValue(getInt("tl", 0), 0, 180);
  const int tr = clampValue(getInt("tr", 0), 0, 180);
  const int m2 = 0;
  const int m4 = 0;

  g_drvM1 = m1;
  g_drvM3 = m3;
  g_drvTurnL = tl;
  g_drvTurnR = tr;
  g_drvIsSpd = isSpd;

  g_yawRefDeg = g_yawEstDeg;

  int out1 = 0;
  int out3 = 0;
  computeMixedMotorOutputs(m1, m3, tl, tr, isSpd, &out1, &out3);

  g_headingHold = (out1 != 0 || out3 != 0 || tl != 0 || tr != 0);

  applyMotorDriveWithGuards(isSpd, out1, out3);
  const String command = buildCommand(isSpd ? "spd" : "pwm", g_lastMotorM1Sent, m2, g_lastMotorM3Sent, m4);

  Serial.printf("[drive] in m1=%d m3=%d -> out %d,%d | pwm_gpio %d,%d\n", m1, m3, out1, out3, g_motorPwmM1,
                g_motorPwmM3);

  String resp = "{\"ok\":true,\"command\":\"" + command + "\",\"out_m1\":" + String(out1) +
                ",\"out_m3\":" + String(out3) + ",\"pwm_m1\":" + String(g_motorPwmM1) +
                ",\"pwm_m3\":" + String(g_motorPwmM3) + "}";
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

  if (cmd == "motor_raw") {
    const int p1 = server.hasArg("m1") ? server.arg("m1").toInt() : 0;
    const int p3 = server.hasArg("m3") ? server.arg("m3").toInt() : 0;
    motorApplyPwmDirect(clampValue(p1, -MOTOR_PWM_CMD_MAX, MOTOR_PWM_CMD_MAX),
                        clampValue(p3, -MOTOR_PWM_CMD_MAX, MOTOR_PWM_CMD_MAX));
    g_lastMotorM1Sent = p1;
    g_lastMotorM3Sent = p3;
    g_headingHold = false;
    server.send(200, "text/plain; charset=utf-8",
                "motor_raw pwm_m1=" + String(g_motorPwmM1) + " pwm_m3=" + String(g_motorPwmM3));
    return;
  }

  if (cmd == "get_yaw" || cmd == "yaw") {
    String s = "yaw_est:" + String(g_yawEstDeg, 2) + ",yaw_ref:" + String(g_yawRefDeg, 2) +
               ",hold:" + String(g_headingHold ? 1 : 0);
    server.send(200, "text/plain; charset=utf-8", s);
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

static void dumpSerialVerboseSnapshot() {
  Serial.printf("  heap=%u WiFi_mode=%d IP=%s\n", (unsigned)ESP.getFreeHeap(), (int)WiFi.getMode(),
                wifiStationOrApIp().c_str());
  Serial.printf("  motor cmd M1=%d M3=%d pwm_act %d,%d enc=%d\n", g_lastMotorM1Sent, g_lastMotorM3Sent,
                g_motorPwmM1, g_motorPwmM3, g_mspdOnline ? 1 : 0);
  Serial.printf("  imu_ok=%d len_cm=%.1f v_cms=%.2f\n", (g_qmaReady && g_hasImuSample) ? 1 : 0,
                g_distanceCm, g_speedCms);
  Serial.printf("  yaw_est=%.2f yaw_ref=%.2f hdg_hold=%d\n", g_yawEstDeg, g_yawRefDeg, g_headingHold ? 1 : 0);
}

void pollSerialVerboseSwitch() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      s_usbDbgLineBuf.trim();
      if (s_usbDbgLineBuf.length() > 0) {
        if (s_usbDbgLineBuf.equalsIgnoreCase("XCDBG1")) {
          g_serialVerbose = true;
          Serial.println("[serial] verbose ON");
          dumpSerialVerboseSnapshot();
        } else if (s_usbDbgLineBuf.equalsIgnoreCase("XCDBG0")) {
          g_serialVerbose = false;
          Serial.println("[serial] verbose OFF");
        }
      }
      s_usbDbgLineBuf = "";
    } else if (s_usbDbgLineBuf.length() < 48) {
      s_usbDbgLineBuf += c;
    }
  }
}

static void maybePrintQuietRunLog() {
  if (g_serialVerbose) {
    return;
  }
  const unsigned long now = millis();
  if (now - g_lastQuietRunLogMs < QUIET_RUN_LOG_INTERVAL_MS) {
    return;
  }
  g_lastQuietRunLogMs = now;
  if (g_distanceCm < 0.0f) {
    Serial.printf("[run] len=-- v=%.1f enc=%d h=%u\n", g_speedCms, g_mspdOnline ? 1 : 0,
                  (unsigned)ESP.getFreeHeap());
  } else {
    Serial.printf("[run] len=%.0f v=%.1f enc=%d h=%u\n", g_distanceCm, g_speedCms, g_mspdOnline ? 1 : 0,
                  (unsigned)ESP.getFreeHeap());
  }
}

bool connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  SERIAL_VF("Connecting to Wi-Fi: %s\n", WIFI_SSID);
  unsigned long startMs = millis();
  const unsigned long timeoutMs = 15000;

  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    delay(500);
    if (g_serialVerbose) {
      Serial.print(".");
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    SERIAL_VF("\nWi-Fi connected. IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[run] WiFi STA IP=%s\n", WiFi.localIP().toString().c_str());
    return true;
  }

  SERIAL_VF("\nWi-Fi connect failed.\n");
  Serial.println("[run] WiFi STA connect failed");
  return false;
}

void startAccessPointFallback() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  if (!ok) {
    Serial.println("[run] AP start failed");
    SERIAL_VF("AP start failed.\n");
    return;
  }

  SERIAL_VF("AP started. SSID: %s\n", AP_SSID);
  SERIAL_VF("AP password: %s\n", AP_PASS);
  SERIAL_VF("AP IP: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("[run] WiFi AP IP=%s ssid=%s\n", WiFi.softAPIP().toString().c_str(), AP_SSID);
}

void initArduinoOta() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (OTA_PASSWORD != nullptr && strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    if (g_serialVerbose) {
      Serial.println("[OTA] start update");
    }
  });
  ArduinoOTA.onEnd([]() {
    if (g_serialVerbose) {
      Serial.println("\n[OTA] finished, rebooting");
    }
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (!g_serialVerbose) {
      return;
    }
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
  SERIAL_VF("[OTA] ArduinoOTA hostname=%s IP=%s password=%s\r\n", OTA_HOSTNAME, wifiStationOrApIp().c_str(),
             (OTA_PASSWORD && strlen(OTA_PASSWORD)) ? "(set)" : "(none)");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("FW %s | USB: XCDBG1=full log XCDBG0=quiet\r\n", FIRMWARE_VERSION);

  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  SERIAL_VF("HY-SRF05 pins: TRIG=GPIO%d, ECHO=GPIO%d (moved off motor 4~7)\n", ULTRA_TRIG_PIN,
            ULTRA_ECHO_PIN);

  bool lcdOk = initLcd();
  SERIAL_VF("LCD pins: CS=GPIO%d DC=GPIO%d SCK=GPIO%d MOSI=GPIO%d | XL9555 RST=IO1_2 PWR=IO1_3\n",
            LCD_CS_PIN, LCD_DC_PIN, LCD_SCK_PIN, LCD_MOSI_PIN);
  SERIAL_VF("LCD init: %s\n", lcdOk ? "OK" : "FAILED");
  g_qmaReady = qma6100pInit();
  SERIAL_VF("QMA6100P init: %s\n", g_qmaReady ? "OK" : "FAILED");
  g_distanceCm = readDistanceCm();
  if (g_qmaReady) {
    g_hasImuSample = readImuData(g_imuPitchDeg, g_imuRollDeg, g_imuAccelAbs);
  }

  initMotorEncoders();  // 在 LCD 之后；编码器脚不可与 LCD_SCK/MOSI(12/11) 共用
  g_lastMotorCmdMs = millis();
  g_lastMotorM1Sent = 0;
  g_lastMotorM3Sent = 0;
  SERIAL_VF("Motor: AT8236 GPIO IN + Hall AB enc (10ms)\n");
  SERIAL_VF("  M1 IN %d,%d  M3 IN %d,%d  ENC %d,%d / %d,%d\n", M1_IN1_PIN, M1_IN2_PIN, M3_IN1_PIN,
            M3_IN2_PIN, ENC_M1_A_PIN, ENC_M1_B_PIN, ENC_M3_A_PIN, ENC_M3_B_PIN);

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

  SERIAL_VF("Web server started.\n");
  if (WiFi.getMode() == WIFI_AP) {
    SERIAL_VF("Open in phone browser: http://192.168.4.1/\n");
  } else {
    SERIAL_VF("Open in phone browser: http://%s/\n", WiFi.localIP().toString().c_str());
  }
  SERIAL_VF("WiFi debug: http://%s/debug   JSON: http://%s/api/live   TCP:%u (JSON lines)\r\n",
            wifiStationOrApIp().c_str(), wifiStationOrApIp().c_str(), (unsigned)DEBUG_TCP_PORT);
  Serial.printf("[run] HTTP http://%s/  debug /debug  TCP:%u\n", wifiStationOrApIp().c_str(),
                (unsigned)DEBUG_TCP_PORT);

  g_lastQuietRunLogMs = millis();
  initArduinoOta();
}

void loop() {
  pollSerialVerboseSwitch();
  maybePrintQuietRunLog();
  tickHeadingHold();
  ArduinoOTA.handle();
  server.handleClient();
  pollKey0SwitchPage();
  pollMotorFeedback();
  pollDebugTcp();
  updateFastSensors();
  updateLcdDistance();
}
