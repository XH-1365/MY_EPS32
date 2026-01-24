#include "api.h"

API::API(const char* s, const char* p, const char* w, U8G2_SSD1306_128X64_NONAME_F_HW_I2C* o) 
    : timeClient(ntpUDP, "ntp.aliyun.com", 8*3600, 60000) {  // UTC+8 for China, 使用阿里云 NTP
    ssid = s;
    password = p;
    weatherUrl = w;
    oled = o;
    lastWeatherUpdate = 0;
    scrollPos = 0;
    cityName = "Loading...";
    temp = "--";
    weather = "Waiting...";
}

void API::init() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== SYSTEM START ===");
    
    // OLED 初始化
    oled->begin();
    oled->clearBuffer();
    oled->setFont(u8g2_font_ncenB08_tr);
    oled->drawStr(0, 20, "Connecting WiFi...");
    oled->sendBuffer();

    // 设置 WiFi 模式
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // 连接 WiFi
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        attempts++;
        Serial.print(".");
        if (attempts % 10 == 0) Serial.println();
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✓ WiFi CONNECTED!");
        Serial.print("  IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("  RSSI: ");
        Serial.println(WiFi.RSSI());
        
        oled->clearBuffer();
        oled->drawStr(0, 20, "WiFi OK");
        oled->sendBuffer();
        delay(2000);

        // 初始化 NTP
        Serial.println("\n=== NTP INIT ===");
        timeClient.begin();
        delay(1000);
        if(timeClient.update()) {
            Serial.println("✓ NTP SUCCESS!");
            Serial.print("  Time: ");
            Serial.println(timeClient.getFormattedTime());
        } else {
            Serial.println("✗ NTP FAILED!");
        }

        // 获取初始天气数据
        fetchWeather();
    } else {
        Serial.println("✗ WiFi FAILED!");
        Serial.print("  Status: ");
        Serial.println(WiFi.status());
        oled->clearBuffer();
        oled->drawStr(0, 20, "WiFi Failed");
        oled->sendBuffer();
        delay(5000);
    }
}

void API::fetchWeather() {
    Serial.println("\n=== FETCHING WEATHER ===");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Requesting: ");
        Serial.println(weatherUrl);
        
        HTTPClient http;
        http.begin(weatherUrl);
        http.setTimeout(10000);  // 10秒超时
        http.addHeader("Accept", "application/json");
        int httpCode = http.GET();
        
        Serial.print("HTTP Code: ");
        Serial.println(httpCode);
        
        if (httpCode > 0) {
            String payload = http.getString();
            Serial.print("Response (first 200): ");
            Serial.println(payload.substring(0, 200));  // 打印前200字符
            Serial.println("Full payload:");
            Serial.println(payload); // 打印完整内容，便于调试
            
            DynamicJsonDocument doc(2048);
            DeserializationError error = deserializeJson(doc, payload);
            
            if (error) {
                Serial.print("JSON Parse Error: ");
                Serial.println(error.c_str());
                cityName = "JSON Error";
                temp = "--";
                weather = "Parse Failed";
            } else {
                // 兼容多种返回结构（顶层或 data[0]，以及 cityInfo/cityinfo）
                String parsedCity = "";
                String parsedTemp = "";
                String parsedWea = "";

                // city
                if (doc.containsKey("city")) {
                    parsedCity = doc["city"].as<String>();
                } else if (doc.containsKey("cityInfo") && doc["cityInfo"].containsKey("city")) {
                    parsedCity = doc["cityInfo"]["city"].as<String>();
                } else if (doc.containsKey("cityinfo") && doc["cityinfo"].containsKey("city")) {
                    parsedCity = doc["cityinfo"]["city"].as<String>();
                } else if (doc.containsKey("data") && doc["data"].is<JsonArray>() && doc["data"].size() > 0 && doc["data"][0].containsKey("city")) {
                    parsedCity = doc["data"][0]["city"].as<String>();
                }

                // temp（当前温度常为 tem；如无，则尝试 data[0].tem）
                if (doc.containsKey("tem")) {
                    parsedTemp = doc["tem"].as<String>();
                } else if (doc.containsKey("data") && doc["data"].is<JsonArray>() && doc["data"].size() > 0 && doc["data"][0].containsKey("tem")) {
                    parsedTemp = doc["data"][0]["tem"].as<String>();
                } else if (doc.containsKey("data") && doc["data"].is<JsonArray>() && doc["data"].size() > 0 && doc["data"][0].containsKey("tem1") && doc["data"][0].containsKey("tem2")) {
                    // 退而求其次：显示最高/最低
                    parsedTemp = String(doc["data"][0]["tem2"].as<String>()) + "~" + String(doc["data"][0]["tem1"].as<String>());
                }

                // weather（wea 或 data[0].wea 或 data[0].weather）
                if (doc.containsKey("wea")) {
                    parsedWea = doc["wea"].as<String>();
                } else if (doc.containsKey("data") && doc["data"].is<JsonArray>() && doc["data"].size() > 0) {
                    if (doc["data"][0].containsKey("wea")) {
                        parsedWea = doc["data"][0]["wea"].as<String>();
                    } else if (doc["data"][0].containsKey("weather")) {
                        parsedWea = doc["data"][0]["weather"].as<String>();
                    }
                }

                cityName = parsedCity;
                temp = parsedTemp;
                weather = parsedWea;
                
                Serial.println("✓ Weather Data:");
                Serial.print("  City: ");
                Serial.println(cityName);
                Serial.print("  Temp: ");
                Serial.println(temp);
                Serial.print("  Weather: ");
                Serial.println(weather);
                
                // 防止空值
                if (cityName == "") cityName = "Unknown";
                if (temp == "") temp = "--";
                if (weather == "") weather = "N/A";
            }
        } else {
            Serial.print("✗ HTTP Failed: ");
            Serial.println(httpCode);
            cityName = "HTTP Error";
            temp = "--";
            weather = String(httpCode);
        }
        http.end();
        lastWeatherUpdate = millis();
    } else {
        Serial.println("✗ WiFi not connected");
        cityName = "No WiFi";
        temp = "--";
        weather = "Offline";
    }
}

void API::update() {
    // 更新时间
    timeClient.update();
    String currentTime = timeClient.getFormattedTime();
    // 转换为12小时制并添加AM/PM
    int hour = timeClient.getHours();
    int minute = timeClient.getMinutes();
    int second = timeClient.getSeconds();
    String ampm = "AM";
    if (hour >= 12) {
        ampm = "PM";
        if (hour > 12) hour -= 12;
    }
    if (hour == 0) hour = 12;
    char time12[12];
    snprintf(time12, sizeof(time12), "%02d:%02d:%02d %s", hour, minute, second, ampm.c_str());

    // 检查是否需要更新天气（调试：每60秒，且首次强制获取）
    if (lastWeatherUpdate == 0 || millis() - lastWeatherUpdate > 60000) {  // 60 seconds for debug
        fetchWeather();
    }

    // 绘制显示
    oled->clearBuffer();
    oled->setFont(u8g2_font_ncenB08_tr);

    // 显示12小时制时间
    String TIME = "TIME: ";
    TIME += time12;
    oled->drawStr(15, 10, TIME.c_str());

    // 显示城市名（调试：取消滚动，直接绘制）
    String displayCity = "City: " + cityName;
    Serial.print("Display City: ");
    Serial.println(displayCity);
    oled->drawStr(0, 25, displayCity.c_str());

    // 显示温度
    Serial.print("Display Temp: ");
    Serial.println(temp);
    oled->drawStr(0, 40, ("Temp: " + temp + "C").c_str());

    // 显示天气
    Serial.print("Display Weather: ");
    Serial.println(weather);
    oled->drawStr(0, 55, ("Weather: " + weather).c_str());

    oled->sendBuffer();
}