#include "motor_xy.h"
#include "imu_mpu6050.h"
#include "console.h"

void setup(){
  Serial.begin(115200);
  delay(300);

  motorXYBegin();
  imuBegin();
  consoleBegin();
}

void loop(){
  imuUpdate();
  motorXYUpdate();
  consoleUpdate();
}
