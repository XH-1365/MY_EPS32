#include "servo_controller.h"

ServoController::ServoController(int pin) {
    servoPin = pin;
}

void ServoController::init() {
    servo.attach(servoPin);
    servo.write(0);
}

void ServoController::rotate(int angle) {
    servo.write(angle);
    delay(1000);
}

void ServoController::sweep() {
    rotate(90);
    delay(1000);
    rotate(180);
}