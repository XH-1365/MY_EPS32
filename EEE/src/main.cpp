#include <Arduino.h>
#include <U8g2lib.h>
#include "api.h"
#include "servo_controller.h"
#include "motor_controller.h"

// 硬件I2C配置
#define OLED_SDA 8
#define OLED_SCL 9

// 舵机引脚 - ESP32-C3 使用 GPIO2
#define SERVO_PIN 2
// 电机PWM引脚与LEDC通道
// 下面示例使用一个 TB6612 类的驱动板（参考你提供的图片）
// 接线示例：
// - VM / VCC: 电机电源(+)
// - GND: 公共地
// - PWMB (或 PWMA): PWM 输入 -> MOTOR_PIN
// - AIN1 / AIN2 (或 BIN1/BIN2): 方向输入 -> DIR_A_PIN / DIR_B_PIN
// - STBY: 使能引脚 -> STBY_PIN

#define MOTOR_PIN 4
#define MOTOR_CHANNEL 0
#define DIR_A_PIN 5
#define DIR_B_PIN 6
#define STBY_PIN 7

U8G2_SSD1306_128X64_NONAME_F_HW_I2C OLED(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);

// WiFi配置（请替换为你的WiFi名称和密码）
const char* ssid = "悦•space-2.4G";
const char* password = "";

// 天气API URL
const char* weatherUrl = "http://gfeljm.tianqiapi.com/api?unescape=1&version=v63&appid=19371195&appsecret=zZbocO1B";

API api(ssid, password, weatherUrl, &OLED);
ServoController servoController(SERVO_PIN);
// MotorController(gpio pwm pin, pwm channel, dirA, dirB, stby)
MotorController motor(MOTOR_PIN, MOTOR_CHANNEL, DIR_A_PIN, DIR_B_PIN, STBY_PIN);

void setup() 
{
    api.init();
    servoController.init();
    motor.init();
    servoController.sweep();
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("Motor control ready. Type 'help' for commands.");
}

void loop() {
    api.update();
    // Serial 控制接口：
    // help            -> 显示帮助
    // start           -> 启用电机（保持当前速度）
    // stop            -> 停止并禁用电机
    // speed <0-100>   -> 设定速度百分比并启用（默认正向）
    // dir <0|1>       -> 设置方向：0 反转, 1 正转

    static bool forward = true;
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "help") {
            Serial.println("Commands: help start stop speed <0-100> dir <0|1>");
        } else if (cmd == "start") {
            motor.enable();
            Serial.println("Motor enabled");
        } else if (cmd == "stop") {
            motor.stop();
            motor.disable();
            Serial.println("Motor stopped and disabled");
        } else if (cmd.startsWith("speed")) {
            int val = cmd.substring(6).toInt();
            motor.setSpeedPercent(val, forward);
            Serial.print("Set speed: "); Serial.println(val);
        } else if (cmd.startsWith("dir")) {
            int d = cmd.substring(4).toInt();
            forward = (d != 0);
            Serial.print("Direction set to "); Serial.println(forward ? "forward" : "reverse");
        } else if (cmd == "status") {
            int st = (STBY_PIN >= 0) ? digitalRead(STBY_PIN) : -1;
            int a = (DIR_A_PIN >= 0) ? digitalRead(DIR_A_PIN) : -1;
            int b = (DIR_B_PIN >= 0) ? digitalRead(DIR_B_PIN) : -1;
            Serial.print("STBY: "); Serial.println(st);
            Serial.print("DIR A: "); Serial.println(a);
            Serial.print("DIR B: "); Serial.println(b);
            Serial.print("Forward flag: "); Serial.println(forward ? "1" : "0");
        } else if (cmd.startsWith("stby")) {
            int v = cmd.substring(5).toInt();
            if (v == 0) { motor.disable(); Serial.println("STBY -> LOW (disabled)"); }
            else { motor.enable(); Serial.println("STBY -> HIGH (enabled)"); }
        } else {
            Serial.println("Unknown cmd. Type 'help'");
        }
    }
    delay(50);
}
