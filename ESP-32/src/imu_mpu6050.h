#pragma once
#include <Arduino.h>

struct ImuAngles {
  float x_deg = 0;
  float y_deg = 0;
  float z_deg = 0;
};

bool imuBegin();                 // Wire.begin + 6050寄存器初始化 + gyro校准 + zero
void imuUpdate();                // loop里高频调用（不阻塞）
void imuZero();                  // home时调用：角度清零
ImuAngles imuGetAngles();        // 取当前角度
void imuPrintOnce();             // 串口打印一次
void imuSetStream(bool on);      // 周期输出开关
