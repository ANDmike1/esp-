/**
 * MG310 + AT8236 — 联调 / 示波器排查版
 *
 * 现象「M+、M- 波形一样、峰值 0」= 电机两端无压差，常见原因：
 *   1) IN1=IN2=0（滑行）或 IN1=IN2=1（刹车）→ OUT1、OUT2 同为低或高阻
 *   2) ESP32 两路 ledc 同时 attach 导致 IN 脚状态异常
 *   3) GPIO 未接到 AT8236 的 IN1/IN2，或 M+/M- 并未接到 OUT1/OUT2
 *
 * 示波器建议：
 *   - CH1: M+ 对 GND；CH2: M- 对 GND（先看各自对地，不要一上来只比两路）
 *   - 全速正转 g 时：应 OUT1≈12V、OUT2≈0V（或相反，取决于线序）
 *   - Math(Ch1-Ch2) 才是电机端电压
 *
 * 串口 115200：
 *   g / f  纯 GPIO 正转全速 IN1=1 IN2=0（无 PWM，优先排查）
 *   G  纯 GPIO 反转全速 IN1=0 IN2=1
 *   b  刹车 IN1=IN2=1（M+/M- 都会是低，波形相同 — 正常）
 *   c  滑行 IN1=IN2=0
 *   d70 / 1  PWM 调速
 *   s  停止
 *   p  只测 GPIO5：先拉低 3s 再拉高 3s（查 IN2 是否被抬高/与 IN1 短路）
 *   h  帮助
 *
 * 读回 IN1=1 IN2=1 → AT8236 为刹车(11)，M+/M- 会一样；程序要的是 10。
 */

#if defined(ESP32)
#include "driver/gpio.h"
#endif

static const int MOTOR_IN1_PIN = 4;
static const int MOTOR_IN2_PIN = 5;

static const uint32_t PWM_FREQ_HZ = 25000;
static const uint8_t PWM_RES_BITS = 8;
static const uint16_t PWM_MAX = (1u << PWM_RES_BITS) - 1u;

static const uint8_t DEFAULT_MAX_DUTY_PERCENT = 100;

static const unsigned long REVERSAL_COAST_MS = 50;
static const int SLEW_STEP_PERCENT = 5;
static const unsigned long SLEW_INTERVAL_MS = 40;

static int g_targetSigned = 0;
static int g_appliedSigned = 0;
static unsigned long g_lastSlewMs = 0;
static int g_pwmActivePin = -1;  // 当前挂 ledc 的脚，另一脚必为 GPIO

// ---------------------------------------------------------------------------
static void detachPwmIfAny() {
  if (g_pwmActivePin < 0) {
    return;
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach((uint8_t)g_pwmActivePin);
#endif
  g_pwmActivePin = -1;
}

static void pinGpioForce(int pin, int level) {
  if (pin == g_pwmActivePin) {
    detachPwmIfAny();
  }
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

static void pinGpioLow(int pin) {
  pinGpioForce(pin, 0);
}

static void pinGpioHigh(int pin) {
  pinGpioForce(pin, 1);
}

static void printInReadback(const char *tag) {
  const int r1 = digitalRead(MOTOR_IN1_PIN);
  const int r2 = digitalRead(MOTOR_IN2_PIN);
  Serial.printf("%s 读回 IN1(GPIO%d)=%d  IN2(GPIO%d)=%d", tag, MOTOR_IN1_PIN, r1, MOTOR_IN2_PIN, r2);
  if (r1 == 1 && r2 == 1) {
    Serial.print("  *** 11刹车！查：IN1/IN2是否短路、GPIO5能否拉低、驱动板 IN2 上拉");
  } else if (r1 == 1 && r2 == 0) {
    Serial.print("  OK(10正转)");
  } else if (r1 == 0 && r2 == 1) {
    Serial.print("  OK(01反转)");
  } else {
    Serial.print("  (00滑行)");
  }
  Serial.println();
}

static void pinPwmDuty(int pin, uint16_t duty) {
  if (duty >= PWM_MAX) {
    pinGpioHigh(pin);
    return;
  }
  if (duty == 0) {
    pinGpioLow(pin);
    return;
  }
  if (g_pwmActivePin != pin) {
    detachPwmIfAny();
    pinMode(pin, OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach((uint8_t)pin, PWM_FREQ_HZ, PWM_RES_BITS);
    g_pwmActivePin = pin;
#else
    const int ch = (pin == MOTOR_IN1_PIN) ? 0 : 1;
    ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(pin, ch);
    g_pwmActivePin = pin;
#endif
  }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite((uint8_t)pin, duty);
#else
  const int ch = (pin == MOTOR_IN1_PIN) ? 0 : 1;
  ledcWrite(ch, duty);
#endif
}

static uint16_t percentToPwm(uint8_t pct) {
  if (pct == 0) {
    return 0;
  }
  const uint32_t v = (uint32_t)PWM_MAX * (uint32_t)pct / 100u;
  return (uint16_t)min(v, (uint32_t)PWM_MAX);
}

/** AT8236 真值表：00 滑行 10 正转 01 反转 11 刹车（M+/M- 同为低） */
static void motorCoast() {
  detachPwmIfAny();
  pinGpioLow(MOTOR_IN1_PIN);
  pinGpioLow(MOTOR_IN2_PIN);
  g_appliedSigned = 0;
  g_targetSigned = 0;
}

static void motorBrake() {
  detachPwmIfAny();
  pinGpioHigh(MOTOR_IN1_PIN);
  pinGpioHigh(MOTOR_IN2_PIN);
  g_appliedSigned = 0;
  g_targetSigned = 0;
}

static void motorGpioForward() {
  detachPwmIfAny();
  pinGpioLow(MOTOR_IN2_PIN);
  delayMicroseconds(50);
  pinGpioHigh(MOTOR_IN1_PIN);
  g_appliedSigned = 100;
  g_targetSigned = 100;
  Serial.println("GPIO 10 保持: 先 IN2=L 再 IN1=H（发 s 才停）");
  printInReadback("  ");
}

static void motorGpioReverse() {
  detachPwmIfAny();
  pinGpioLow(MOTOR_IN1_PIN);
  pinGpioHigh(MOTOR_IN2_PIN);
  g_appliedSigned = -100;
  g_targetSigned = -100;
  Serial.println("GPIO 01 保持: IN1=L IN2=H（发 s 才停）");
}

static void motorApplySigned(int signedPercent) {
  if (signedPercent == 0) {
    motorCoast();
    return;
  }

  const int mag = constrain(abs(signedPercent), 0, (int)DEFAULT_MAX_DUTY_PERCENT);
  const uint16_t pwm = percentToPwm((uint8_t)mag);

  if (signedPercent > 0) {
    pinGpioLow(MOTOR_IN2_PIN);
    pinPwmDuty(MOTOR_IN1_PIN, pwm);
  } else {
    pinGpioLow(MOTOR_IN1_PIN);
    pinPwmDuty(MOTOR_IN2_PIN, pwm);
  }
  g_appliedSigned = signedPercent;
}

static void motorSetTargetSigned(int target) {
  g_targetSigned = constrain(target, -(int)DEFAULT_MAX_DUTY_PERCENT, (int)DEFAULT_MAX_DUTY_PERCENT);
}

static bool oppositeNonZero(int from, int to) {
  return (from > 0 && to < 0) || (from < 0 && to > 0);
}

static void motorSlewTick() {
  const unsigned long now = millis();
  if (now - g_lastSlewMs < SLEW_INTERVAL_MS || g_appliedSigned == g_targetSigned) {
    return;
  }
  g_lastSlewMs = now;

  if (oppositeNonZero(g_appliedSigned, g_targetSigned)) {
    motorCoast();
    delay(REVERSAL_COAST_MS);
    g_lastSlewMs = millis();
    return;
  }

  int next = g_appliedSigned;
  if (g_targetSigned > g_appliedSigned) {
    next = min(g_appliedSigned + SLEW_STEP_PERCENT, g_targetSigned);
  } else {
    next = max(g_appliedSigned - SLEW_STEP_PERCENT, g_targetSigned);
  }
  motorApplySigned(next);
}

static void printHelp() {
  Serial.println();
  Serial.println("=== AT8236 排查（无自动演示）===");
  Serial.printf("IN1=GPIO%d IN2=GPIO%d\n", MOTOR_IN1_PIN, MOTOR_IN2_PIN);
  Serial.println("g/f/G  GPIO全速  b=刹车  c=滑行  p=单测GPIO5  i=读脚  s=停");
  Serial.println("d70  PWM70%   1=PWM100%");
  Serial.println("若 g 后 M+、M- 对地仍相同：查硬件/引脚/OUT 接线");
  Serial.println();
}

static void handleSerialLine(const String &line) {
  if (line.length() == 0) {
    return;
  }
  const char c0 = line.charAt(0);

  if (c0 == 'h' || c0 == 'H' || c0 == '?') {
    printHelp();
    return;
  }
  if (c0 == 's' || c0 == 'S' || line == "0") {
    motorSetTargetSigned(0);
    motorCoast();
    Serial.println("-> 滑行 00");
    return;
  }
  if (c0 == 'g' || c0 == 'f' || c0 == 'F') {
    motorGpioForward();
    return;
  }
  if (c0 == 'p' || c0 == 'P') {
    detachPwmIfAny();
    pinGpioLow(MOTOR_IN1_PIN);
    Serial.println("仅 GPIO5：3s 低 -> 3s 高（用表/示波器测 ESP32 第5脚，勿只看串口读回）");
    pinGpioLow(MOTOR_IN2_PIN);
    printInReadback("L");
    delay(3000);
    pinGpioHigh(MOTOR_IN2_PIN);
    printInReadback("H");
    delay(3000);
    pinGpioLow(MOTOR_IN2_PIN);
    printInReadback("end");
    return;
  }
  if (c0 == 'i' || c0 == 'I') {
    Serial.printf("target=%d applied=%d  ", g_targetSigned, g_appliedSigned);
    printInReadback("i");
    Serial.println("万用表：GPIO4/5 对 GND；驱动板 IN1/IN2 电阻应开路(非0Ω)");
    return;
  }
  if (c0 == 'G') {
    motorGpioReverse();
    return;
  }
  if (c0 == 'b' || c0 == 'B') {
    motorBrake();
    Serial.println("-> 刹车 11（M+ M- 同为低，波形相同是正常的）");
    return;
  }
  if (c0 == 'c' || c0 == 'C') {
    motorCoast();
    Serial.println("-> 滑行 00");
    return;
  }
  if (c0 == '1' || c0 == 'm' || c0 == 'M') {
    g_targetSigned = 100;
    motorApplySigned(100);
    Serial.println("-> PWM 100% 正转");
    return;
  }
  if (c0 == 'd' || c0 == 'D') {
    int v = line.substring(1).toInt();
    if (line.length() >= 2 && line.charAt(1) == '-') {
      v = -line.substring(2).toInt();
    }
    v = constrain(v, -100, 100);
    g_targetSigned = v;
    motorApplySigned(v);
    Serial.printf("-> PWM %d%%\n", v);
    return;
  }

  Serial.println("未知命令，输入 h");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pinGpioLow(MOTOR_IN1_PIN);
  pinGpioLow(MOTOR_IN2_PIN);
  printHelp();
  Serial.println("请先测 VM，再串口发 g 做全速 GPIO 测试");
}

void loop() {
  motorSlewTick();

  static String line;
  while (Serial.available() > 0) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      handleSerialLine(line);
      line = "";
    } else {
      line += ch;
      if (line.length() > 24) {
        line = "";
      }
    }
  }
}
