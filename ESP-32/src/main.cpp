#include <Arduino.h>
#include <Wire.h>

/* ========= 电机引脚 ========= */
#define AIN1 25
#define AIN2 26
#define PWM_PIN 27

int pwm_value = 180;

/* ========= MPU6050 地址 ========= */
#define MPU_ADDR 0x68

void motorLeft() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM_PIN, pwm_value);
}

void motorRight() {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_PIN, pwm_value);
}

void motorStop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_PIN, 0);
}

void setup() {
    Serial.begin(115200);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);

    Wire.begin(21, 22);

    // 唤醒 MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    Serial.println("MPU6050 Ready");
}

void loop() {

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();

    float accX = ax / 16384.0;

    Serial.println(accX);

    if (accX > 0.3) {
        motorRight();
    }
    else if (accX < -0.3) {
        motorLeft();
    }
    else {
        motorStop();
    }

    delay(50);
}

