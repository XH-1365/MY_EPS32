#ifndef DRV_PWM_H
#define DRV_PWM_H

#include <stdint.h>
#include "driver/ledc.h"

void drv_pwm_init(int gpio, int channel);
void drv_pwm_set_duty(int channel, uint32_t duty);

#endif
