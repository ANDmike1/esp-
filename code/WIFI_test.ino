#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <ArduinoOTA.h>

// ===== Wi-Fi config =====
const char *WIFI_SSID = "520卧龙凤雏";
const char *WIFI_PASS = "gsuxnfeDf26472";
const char *AP_SSID = "ESP32S3-OTA-TEST";
const char *AP_PASS = "12345678";

// ===== ArduinoOTA config =====
const char *OTA_HOSTNAME = "esp32s3-ota-test";
const char *OTA_PASSWORD = "xiaocheota";

// ===== ATK-MD0130 (ST7789, 240x240) =====
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

bool xl9555WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(EXIO_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool xl9555ReadReg(uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(EXIO_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)EXIO_ADDR, 1) != 1) return false;
  value = (uint8_t)Wire.read();
  return true;
}

bool xl9555IoConfig(uint16_t pinMask, bool output) {
  uint8_t cfg0 = 0;
  uint8_t cfg1 = 0;
  if (!xl9555ReadReg(XL9555_CONFIG_PORT0_REG, cfg0)) return false;
  if (!xl9555ReadReg(XL9555_CONFIG_PORT1_REG, cfg1)) return false;

  for (int bit = 0; bit < 16; ++bit) {
    if (!(pinMask & (1U << bit))) continue;
    if (bit < 8) {
      if (output) cfg0 &= ~(1U << bit);
      else cfg0 |= (1U << bit);
    } else {
      const int b = bit - 8;
      if (output) cfg1 &= ~(1U << b);
      else cfg1 |= (1U << b);
    }
  }
  return xl9555WriteReg(XL9555_CONFIG_PORT0_REG, cfg0) &&
         xl9555WriteReg(XL9555_CONFIG_PORT1_REG, cfg1);
}

bool xl9555PinSet(uint16_t pinMask, bool high) {
  uint8_t out0 = 0;
  uint8_t out1 = 0;
  if (!xl9555ReadReg(XL9555_OUTPUT_PORT0_REG, out0)) return false;
  if (!xl9555ReadReg(XL9555_OUTPUT_PORT1_REG, out1)) return false;

  for (int bit = 0; bit < 16; ++bit) {
    if (!(pinMask & (1U << bit))) continue;
    if (bit < 8) {
      if (high) out0 |= (1U << bit);
      else out0 &= ~(1U << bit);
    } else {
      const int b = bit - 8;
      if (high) out1 |= (1U << b);
      else out1 &= ~(1U << b);
    }
  }
  return xl9555WriteReg(XL9555_OUTPUT_PORT0_REG, out0) &&
         xl9555WriteReg(XL9555_OUTPUT_PORT1_REG, out1);
}

void lcdPrintLine(int y, uint16_t color, const String &text) {
  lcd.fillRect(8, y, 224, 24, ST77XX_BLACK);
  lcd.setCursor(10, y + 4);
  lcd.setTextColor(color);
  lcd.print(text);
}

void initLcd() {
  Wire.begin(EXIO_SDA_PIN, EXIO_SCL_PIN);

  xl9555IoConfig(XL9555_LCD_RST, true);
  xl9555IoConfig(XL9555_LCD_PWR, true);
  xl9555PinSet(XL9555_LCD_PWR, true);

  xl9555PinSet(XL9555_LCD_RST, false);
  delay(20);
  xl9555PinSet(XL9555_LCD_RST, true);
  delay(120);

  lcdSpi.begin(LCD_SCK_PIN, -1, LCD_MOSI_PIN, LCD_CS_PIN);
  lcd.init(240, 240);
  lcd.setRotation(0);
  lcd.fillScreen(ST77XX_BLACK);
  lcd.setTextWrap(false);
  lcd.setTextSize(2);

  lcd.setCursor(10, 10);
  lcd.setTextColor(ST77XX_CYAN);
  lcd.print("ESP32-S3 OTA TEST");
}

void connectWifiWithWait() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  lcdPrintLine(44, ST77XX_YELLOW, "Connecting WiFi...");
  Serial.print("Connecting WiFi");

  const unsigned long startMs = millis();
  const unsigned long timeoutMs = 30000;
  int dots = 0;

  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    delay(300);
    Serial.print(".");
    dots = (dots + 1) % 4;
    String line = "Connecting WiFi";
    for (int i = 0; i < dots; ++i) line += ".";
    lcdPrintLine(44, ST77XX_YELLOW, line);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    lcdPrintLine(44, ST77XX_GREEN, "WiFi Connected");
    lcdPrintLine(72, ST77XX_WHITE, "IP: " + WiFi.localIP().toString());
    Serial.println("WiFi connected.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    return;
  }

  // Keep waiting behavior by switching to AP mode if STA cannot connect.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  lcdPrintLine(44, ST77XX_ORANGE, "STA Timeout, AP Ready");
  lcdPrintLine(72, ST77XX_WHITE, "AP IP: " + WiFi.softAPIP().toString());
  Serial.println("STA timeout, AP mode started.");
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

void setupOta() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    lcdPrintLine(100, ST77XX_YELLOW, "OTA Updating...");
    Serial.println("OTA start");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int p = (total == 0) ? 0 : (int)((progress * 100U) / total);
    lcdPrintLine(128, ST77XX_WHITE, "Progress: " + String(p) + "%");
  });
  ArduinoOTA.onEnd([]() {
    lcdPrintLine(100, ST77XX_GREEN, "OTA Done. Rebooting");
    Serial.println("OTA end");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    lcdPrintLine(100, ST77XX_RED, "OTA Error: " + String((int)error));
    Serial.printf("OTA error[%u]\n", error);
  });

  ArduinoOTA.begin();
  lcdPrintLine(156, ST77XX_GREEN, "OTA Ready");
  Serial.println("OTA ready.");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  initLcd();
  connectWifiWithWait();  // Keep waiting connection logic.
  setupOta();

  lcdPrintLine(184, ST77XX_CYAN, "Upload via Network Port");
  Serial.println("Use Arduino IDE network port for OTA upload.");
}

void loop() {
  ArduinoOTA.handle();
  delay(2);
}
