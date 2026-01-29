#include <Arduino.h>

/* ========= 电机驱动引脚 ========= */
#define AIN1 3
#define AIN2 4
#define PWMA 5

/* ========= 霍尔编码器引脚 ========= */
#define HALL_A 7
#define HALL_B 6


/* ========= PWM 参数 ========= */
#define PWM_CH   0
#define PWM_FREQ 20000
#define PWM_RES  8

/* ========= 编码器参数 ========= */
// ⚠️ 先用一个常见值，后面可以改成你实测的
#define PULSE_PER_REV 520

/* ========= 全局变量 ========= */
volatile long encoderCount = 0;
volatile int lastA = 0;

/* ========= 霍尔中断 ========= */
void IRAM_ATTR hallISR() {
    int A = digitalRead(HALL_A);
    int B = digitalRead(HALL_B);

    if (A != lastA) {
        if (B != A) {
            encoderCount++;
        } else {
            encoderCount--;
        }
        lastA = A;
    }
}

void setup() {
    /* 电机控制引脚 */
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    /* 霍尔引脚 */
    pinMode(HALL_A, INPUT_PULLUP);
    pinMode(HALL_B, INPUT_PULLUP);

    /* 默认正转方向 */
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    /* PWM 配置 */
    ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWMA, PWM_CH);
    ledcWrite(PWM_CH, 0);

    /* 霍尔中断 */
    attachInterrupt(digitalPinToInterrupt(HALL_A), hallISR, CHANGE);

    Serial.begin(115200);
    Serial.println("Motor + Hall angle control start");
}

void loop() {
    /* ======= 目标角度（可改） ======= */
    float targetAngle = 180.0;   // 目标 180 度

    long targetCount = targetAngle / 360.0 * PULSE_PER_REV;

    if (encoderCount < targetCount) {
        // 继续转
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        ledcWrite(PWM_CH, 80);
    } else {
        // 到角度，停
        ledcWrite(PWM_CH, 0);
    }
    
    delay(50);
}
