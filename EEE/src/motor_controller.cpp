#include <Arduino.h>
#include "motor_controller.h"
#include "drv_pwm.h"

#define PWM_RESOLUTION_BITS 10

MotorController::MotorController(int gpio, int channel, int dirA, int dirB, int stby)
{
    gpioPin = gpio;
    pwmChannel = channel;
    dirPinA = dirA;
    dirPinB = dirB;
    stbyPin = stby;
}

void MotorController::init()
{
    drv_pwm_init(gpioPin, pwmChannel);
    if (dirPinA >= 0) pinMode(dirPinA, OUTPUT);
    if (dirPinB >= 0) pinMode(dirPinB, OUTPUT);
    if (stbyPin >= 0) pinMode(stbyPin, OUTPUT);
    // keep driver disabled by default for safety
    if (stbyPin >= 0) digitalWrite(stbyPin, LOW);
}

void MotorController::enable()
{
    if (stbyPin >= 0) digitalWrite(stbyPin, HIGH);
}

void MotorController::disable()
{
    if (stbyPin >= 0) digitalWrite(stbyPin, LOW);
    drv_pwm_set_duty(pwmChannel, 0);
}

void MotorController::setDirection(bool forward)
{
    if (dirPinA < 0 || dirPinB < 0) return;
    if (forward) {
        digitalWrite(dirPinA, HIGH);
        digitalWrite(dirPinB, LOW);
    } else {
        digitalWrite(dirPinA, LOW);
        digitalWrite(dirPinB, HIGH);
    }
}

void MotorController::setSpeedPercent(int percent, bool forward)
{
    // enable driver automatically when setting speed
    if (stbyPin >= 0) digitalWrite(stbyPin, HIGH);
    setDirection(forward);
    if (percent <= 0) {
        drv_pwm_set_duty(pwmChannel, 0);
        return;
    }
    if (percent > 100) percent = 100;
    uint32_t max = (1u << PWM_RESOLUTION_BITS) - 1u;
    uint32_t duty = (uint32_t)((percent * max) / 100);
    drv_pwm_set_duty(pwmChannel, duty);
}

void MotorController::stop()
{
    drv_pwm_set_duty(pwmChannel, 0);
    if (stbyPin >= 0) digitalWrite(stbyPin, LOW); // put driver in standby
}
