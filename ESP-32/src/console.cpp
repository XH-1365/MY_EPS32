#include "console.h"
#include <Arduino.h>
#include "motor_xy.h"
#include "imu_mpu6050.h"

static String line;

void consoleBegin(){
  Serial.println("=== Console ===");
  Serial.println("cmd: home | pos | mx <mm> | my <mm> | imu | imuon | imuoff | stop");
}

void consoleUpdate(){
  while(Serial.available()){
    char c = (char)Serial.read();
    if(c=='\n' || c=='\r'){
      line.trim();
      if(line.length()){
        Serial.print(">> "); Serial.println(line);

        if(line=="stop"){
          motorXYStopBrake();
          Serial.println("OK stop");
        } else if(line=="home"){
          motorXYHome();
          imuZero();
          Serial.println("OK home (enc=0, ang=0)");
        } else if(line=="pos"){
          motorPrintPos();
        } else if(line=="imu"){
          imuPrintOnce();
        } else if(line=="imuon"){
          imuSetStream(true);
          Serial.println("IMU angle stream ON (200ms)");
        } else if(line=="imuoff"){
          imuSetStream(false);
          Serial.println("IMU angle stream OFF");
        } else if(line.startsWith("mx ")){
          motorMoveX(line.substring(3).toFloat());
        } else if(line.startsWith("my ")){
          motorMoveY(line.substring(3).toFloat());
        } else {
          Serial.println("cmd: home | pos | mx <mm> | my <mm> | imu | imuon | imuoff | stop");
        }
      }
      line = "";
    } else {
      line += c;
      if(line.length()>80) line="";
    }
  }
}
