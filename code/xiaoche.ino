#include <WiFi.h>
#include <WebServer.h>

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

WebServer server(80);

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
}
