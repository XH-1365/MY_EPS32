#include <Arduino.h>

// 示例：在 ESP32 上用 GPIO8 点亮/闪烁一个 LED。
// 注意：某些 ESP32 板的部分 GPIO（包括 GPIO8）可能被用于闪存或引导引脚，
// 在将外设接到 GPIO8 前请确认你的具体板子允许使用该引脚。

const int LED_PIN = 8; // 要使用的 GPIO
// 如果你想要常亮，把 blinkInterval 设为 0；若想闪烁，设为毫秒间隔（例如 500）
const unsigned long blinkInterval = 500; // 500ms 闪烁；改为 0 则常亮

unsigned long lastToggle = 0;
bool ledState = LOW;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  if (blinkInterval == 0) {
    // 常亮
    digitalWrite(LED_PIN, HIGH);
  } else {
    // 初始化为熄灭状态，之后在 loop 中按间隔切换
    digitalWrite(LED_PIN, ledState);
    lastToggle = millis();
  }
}

void loop() {
  if (blinkInterval == 0) return; // 常亮时不需要重复工作

  unsigned long now = millis();
  if (now - lastToggle >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
    lastToggle = now;
  }
}