#include "OledUI.h"
#include "Config.h"

#if OLED_ENABLE
  #include <Wire.h>
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>

  // 依赖你的状态接口（来自 Commands/Motion/Braille）
  #include "Motion.h"
  #include "Braille.h"

  static Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);

  static String g_ip = "0.0.0.0";
  static uint32_t lastMs = 0;

  void oledSetIp(const String& ipStr) { g_ip = ipStr; }

  void oledInit() {
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      // OLED 不存在也别让程序死掉
      Serial.println("[OLED] init failed");
      return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println("Braille XY Ready");
    display.println("OLED OK");
    display.display();
  }

  void oledUpdate() {
    // 5Hz 刷新：200ms一次，不卡主循环
    uint32_t now = millis();
    if (now - lastMs < 200) return;
    lastMs = now;

    // 读取状态
    float x = curXmm();
    float y = curYmm();
    bool mvx = isMovingX();
    bool mvy = isMovingY();
    bool prt = brailleIsPrinting();
    int  di  = brailleDotI();
    int  dn  = brailleDotN();

    display.clearDisplay();

    // 第一行：IP
    display.setCursor(0, 0);
    display.print("IP:");
    display.println(g_ip);

    // 第二三行：坐标
    display.setCursor(0, 16);
    display.print("X:");
    display.print(x, 2);
    display.print("mm ");
    display.print(mvx ? "RUN" : "STOP");

    display.setCursor(0, 28);
    display.print("Y:");
    display.print(y, 2);
    display.print("mm ");
    display.print(mvy ? "RUN" : "STOP");

    // 第四五行：打印状态
    display.setCursor(0, 44);
    display.print("PRINT:");
    display.print(prt ? "ON " : "OFF");

    display.setCursor(0, 56);
    display.print(di);
    display.print("/");
    display.print(dn);

    display.display();
  }

#else
  void oledInit() {}
  void oledUpdate() {}
  void oledSetIp(const String&) {}
#endif