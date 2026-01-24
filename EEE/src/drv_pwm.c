#include "drv_pwm.h"
#include "driver/ledc.h"

#define PWM_FREQ      20000
#define PWM_RES       LEDC_TIMER_10_BIT

void drv_pwm_init(int gpio, int channel)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RES,
        .freq_hz          = PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)channel,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch);
}

void drv_pwm_set_duty(int channel, uint32_t duty)
{
    // clamp duty to resolution
    uint32_t max = (1 << PWM_RES) - 1;
    if (duty > max) duty = max;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)channel);
}
