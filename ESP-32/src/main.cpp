#include <Arduino.h>

/* ========= 任何轴电机 ========= */
#define Y_AIN1 5
#define Y_AIN2 18
#define Y_PWM  19

/* ========= AB 编码器 ========= */
#define Y_ENCA 33
#define Y_ENCB 32

/* ========= PWM ========= */
#define PWM_CHANNEL_Y 1
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

volatile long encoder_count = 0;

int y_speed = 180;

/* ========= AB中断 ========= */
void IRAM_ATTR encoderISR() {

    bool A = digitalRead(Y_ENCA);
    bool B = digitalRead(Y_ENCB);

    if (A == B) {
        encoder_count++;
    } else {
        encoder_count--;
    }
}

/* ========= 电机控制 ========= */
void yForward() {
    digitalWrite(Y_AIN1, HIGH);
    digitalWrite(Y_AIN2, LOW);
    ledcWrite(PWM_CHANNEL_Y, y_speed);
}

void yBackward() {
    digitalWrite(Y_AIN1, LOW);
    digitalWrite(Y_AIN2, HIGH);
    ledcWrite(PWM_CHANNEL_Y, y_speed);
}

void yStop() {
    digitalWrite(Y_AIN1, LOW);
    digitalWrite(Y_AIN2, LOW);
    ledcWrite(PWM_CHANNEL_Y, 0);
}

void setup() {
    Serial.begin(115200);

    pinMode(Y_AIN1, OUTPUT);
    pinMode(Y_AIN2, OUTPUT);

    pinMode(Y_ENCA, INPUT_PULLUP);
    pinMode(Y_ENCB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Y_ENCA), encoderISR, CHANGE);

    ledcSetup(PWM_CHANNEL_Y, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(Y_PWM, PWM_CHANNEL_Y);

    Serial.println("输入 f 正转, b 反转, s 停止");
}

void loop() {

    if (Serial.available()) {

        char cmd = Serial.read();

        if (cmd == 'f') yForward();
        if (cmd == 'b') yBackward();
        if (cmd == 's') yStop();
    }

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 500) {
        Serial.print("编码器计数: ");
        Serial.println(encoder_count);
        lastPrint = millis();
    }
}
