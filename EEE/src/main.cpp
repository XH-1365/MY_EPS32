#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// 硬件I2C配置
#define OLED_SDA 41
#define OLED_SCL 42
U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

// WiFi配置（请替换为你的WiFi名称和密码）
const char* ssid = "C511";
const char* password = "c511c511";

// 天气API URL
const char* weatherUrl = "http://gfeljm.tianqiapi.com/api?unescape=1&version=v63&appid=19371195&appsecret=zZbocO1B";

// 128x64 XBM格式图像数据（示例数据，需替换为你的实际数据）
static const unsigned char imageData[] U8X8_PROGMEM = {

};
void setup() 
{
  Serial.begin(115200);
  // OLED初始化
  OLED.begin();
  OLED.clearBuffer();
  OLED.setFont(u8g2_font_ncenB08_tr);
  OLED.drawStr(0, 20, "Connecting WiFi...");
  OLED.sendBuffer();

  // 连接WiFi
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    attempts++;
    Serial.println("Attempting to connect to WiFi...");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected");
    OLED.clearBuffer();
    OLED.drawStr(0, 20, "WiFi Connected");
    OLED.sendBuffer();
    delay(2000);
  } else {
    Serial.println("WiFi Connection Failed");
    OLED.clearBuffer();
    OLED.drawStr(0, 20, "WiFi Failed");
    OLED.sendBuffer();
    delay(5000);
    return; // 退出setup，避免继续执行
  }

  // 获取天气数据
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(weatherUrl);
    int httpCode = http.GET();
    if (httpCode > 0) {
      String payload = http.getString();
      DynamicJsonDocument doc(2048);
      deserializeJson(doc, payload);
      String cityName = doc["city"];
      String temp = doc["tem"];
      String weather = doc["wea"];

      OLED.clearBuffer();
      OLED.drawStr(0, 20, ("City: " + cityName).c_str());
      OLED.drawStr(0, 40, ("Temp: " + temp + "C").c_str());
      OLED.drawStr(0, 60, ("Weather: " + weather).c_str());
      OLED.sendBuffer();
    } else {
      OLED.clearBuffer();
      OLED.drawStr(0, 20, "Failed to get weather");
      OLED.sendBuffer();
    }
    http.end();
  }
}

void loop() {

}
