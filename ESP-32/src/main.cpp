#include <Arduino.h>

/* ========= 电机引脚 ========= */
#define AIN1 25
#define AIN2 26
#define PWM_PIN 27

/* ========= 霍尔 ========= */
#define HALL_PIN 34

/* ========= PWM ========= */
#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

/* ========= 参数 ========= */
float pulse_per_mm = 273.0;   // ★ 可自己调

volatile long pulse_count = 0;
long target_pulse = 0;

int motor_speed = 180;
int motor_direction = 1;  // 1 正方向  -1 反方向

/* ========= 霍尔中断 ========= */
void IRAM_ATTR hallISR() {
    pulse_count++;
}

/* ========= 电机控制 ========= */
void motorForward() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWM_CHANNEL, motor_speed);
}

void motorBackward() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    ledcWrite(PWM_CHANNEL, motor_speed);
}

void motorStop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
}

void setup() {
    Serial.begin(115200);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWM_PIN, PWM_CHANNEL);

    Serial.println("输入移动距离 (mm)，可输入负数：");
}

void loop() {

    /* ===== 串口输入 ===== */
    if (Serial.available()) {

        float distance = Serial.parseFloat();  

        if (distance == 0) return;

        pulse_count = 0;

        target_pulse = abs(distance) * pulse_per_mm;

        if (distance > 0) {
            motor_direction = 1;
            motorForward();
            Serial.println("正方向移动");
        } else {
            motor_direction = -1;
            motorBackward();
            Serial.println("反方向移动");
        }

        Serial.print("目标脉冲: ");
        Serial.println(target_pulse);
    }

    /* ===== 到达目标停止 ===== */
    if (target_pulse > 0 && pulse_count >= target_pulse) {
        motorStop();
        Serial.println("到达目标位置");
        target_pulse = 0;
    }
}
