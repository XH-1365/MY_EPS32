#include <Arduino.h>

#define AIN1 3
#define AIN2 4
#define PWMA 5

#define PWM_CH   0        // PWM 通道
#define PWM_FREQ 20000    // 20kHz，电机很安静
#define PWM_RES  8        // 8位分辨率：0~255

void setup() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    // 电机方向：正转
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    // 配置 PWM
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWMA, PWM_CH);

    // 初始速度（0~255）
    ledcWrite(PWM_CH, 120);   // 中速
}

void loop() {
    // 转
    ledcWrite(PWM_CH, 150);  // 转速
    delay(1000);             // 转 1 秒

    // 停
    ledcWrite(PWM_CH, 0);    // 停止
    delay(1000);             // 停 1 秒
}