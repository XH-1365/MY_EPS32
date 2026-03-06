#include "ServoPunch.h"
#include "Config.h"

static inline uint32_t angleToDuty(int angle){
  float us = 500.0f + (2000.0f * angle / 180.0f);
  float duty = us / 20000.0f;
  uint32_t maxDuty = (1u << SERVO_RES) - 1u;
  return (uint32_t)(duty * maxDuty);
}

void servoInit(){
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
  ledcWrite(SERVO_CH, angleToDuty(SERVO_UP_ANG));
}
void servoUp(){
  ledcWrite(SERVO_CH, angleToDuty(SERVO_UP_ANG));
}
void servoPunchOnce(){
  ledcWrite(SERVO_CH, angleToDuty(SERVO_DOWN_ANG));
  delay(SERVO_DOWN_MS);
  ledcWrite(SERVO_CH, angleToDuty(SERVO_UP_ANG));
}