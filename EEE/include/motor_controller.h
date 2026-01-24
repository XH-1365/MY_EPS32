#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>

class MotorController {
public:
    // gpio: PWM pin; dirA/dirB: direction pins; stby: standby pin (can be -1 if unused)
    MotorController(int gpio, int channel, int dirA = -1, int dirB = -1, int stby = -1);
    void init();
    void enable();
    void disable();
    void setSpeedPercent(int percent, bool forward = true); // 0-100
    void stop();

private:
    int gpioPin;
    int pwmChannel;
    int dirPinA;
    int dirPinB;
    int stbyPin;
    void setDirection(bool forward);
};

#endif
