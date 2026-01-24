#ifndef API_H
#define API_H

#include <U8g2lib.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

class API {
private:
    const char* ssid;
    const char* password;
    const char* weatherUrl;
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C* oled;
    WiFiUDP ntpUDP;
    NTPClient timeClient;
    unsigned long lastWeatherUpdate;
    String cityName;
    String temp;
    String weather;
    int scrollPos;

public:
    API(const char* s, const char* p, const char* w, U8G2_SSD1306_128X64_NONAME_F_HW_I2C* o);
    void init();
    void update();
private:
    void fetchWeather();
};

#endif