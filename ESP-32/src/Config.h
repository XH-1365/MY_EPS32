#pragma once
#include <Arduino.h>

/* ===== Pins ===== */
// X motor
#define X_AIN1 25
#define X_AIN2 26
#define X_PWM  27

// Y motor
#define Y_AIN1 5
#define Y_AIN2 18
#define Y_PWM  19

// Encoders
#define X_HALL 34      // GPIO34 no pullup
#define Y_ENCA 33
#define Y_ENCB 32

/* ===== PWM ===== */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH_X 0
#define PWM_CH_Y 1

/* ===== Servo (LEDC) ===== */
#define SERVO_PIN 13
#define SERVO_CH  2
#define SERVO_FREQ 50
#define SERVO_RES 16
#define SERVO_UP_ANG     60
#define SERVO_DOWN_ANG  120
#define SERVO_DOWN_MS    60

/* ===== Calibration ===== */
#define PULSES_PER_MM_X 272.825f
#define PULSES_PER_MM_Y 6382.63f

/* ===== Move profile ===== */
#define PWM_FAST     255
#define PWM_SLOW_X   200
#define PWM_SLOW_Y   200
#define SLOW_ZONE_MM_X 6.0f
#define SLOW_ZONE_MM_Y 12.0f
#define DONE_MM_X 0.20f
#define DONE_MM_Y 0.20f
#define BRAKE_MS_X 200
#define BRAKE_MS_Y 260
#define Y_INVERT 0

/* ===== Braille layout ===== */
#define PAGE_ORIGIN_X_MM 0.0f
#define PAGE_ORIGIN_Y_MM 0.0f
#define DOT_DX_MM 2.0f
#define DOT_DY_MM 2.5f
#define CELL_DX_MM 5.0f
#define LINE_DY_MM 5.0f
#define PAGE_COLS 10
#define PAGE_ROWS 3

/* ===== OLED (SSD1306 I2C) ===== */
#define OLED_ENABLE 1
#define OLED_ADDR 0x3C     // 常见是 0x3C，少数是 0x3D
#define OLED_W 128
#define OLED_H 64

#define I2C_SDA 23
#define I2C_SCL 4