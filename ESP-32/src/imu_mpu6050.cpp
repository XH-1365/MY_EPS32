#include "imu_mpu6050.h"
#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

static inline void mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}
static inline int16_t mpuRead16(uint8_t reg){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
  if (Wire.available() < 2) return 0;
  return (int16_t)((Wire.read() << 8) | Wire.read());
}
static void readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz,int16_t &tmp){
  ax = mpuRead16(0x3B);
  ay = mpuRead16(0x3D);
  az = mpuRead16(0x3F);
  tmp= mpuRead16(0x41);
  gx = mpuRead16(0x43);
  gy = mpuRead16(0x45);
  gz = mpuRead16(0x47);
}

static float gBiasX=0, gBiasY=0, gBiasZ=0;
static float angX=0, angY=0, angZ=0;
static float offX=0, offY=0, offZ=0;
static bool streamOn=false;
static unsigned long lastUs=0;
static unsigned long lastPrintMs=0;

static inline void wrap180(float &a){
  while(a > 180) a -= 360;
  while(a < -180) a += 360;
}

static void calibrateGyro(int samples=600){
  long sx=0, sy=0, sz=0;
  int16_t ax,ay,az,gx,gy,gz,tmp;
  for(int i=0;i<samples;i++){
    readMPU(ax,ay,az,gx,gy,gz,tmp);
    sx += gx; sy += gy; sz += gz;
    delay(3);
  }
  gBiasX = (float)sx / samples;
  gBiasY = (float)sy / samples;
  gBiasZ = (float)sz / samples;
}

bool imuBegin(){
  Wire.begin(21, 22);
  Wire.setClock(400000);

  mpuWrite(0x6B, 0x00);
  delay(80);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
  delay(10);

  Serial.println("Keep MPU still, calibrating gyro...");
  calibrateGyro(600);
  Serial.printf("Gyro bias: %.1f %.1f %.1f (LSB)\n", gBiasX, gBiasY, gBiasZ);

  imuZero();
  return true;
}

void imuZero(){
  angX = angY = angZ = 0;
  offX = offY = offZ = 0;
  lastUs = micros();
}

void imuUpdate(){
  unsigned long nowUs = micros();
  if (lastUs == 0) lastUs = nowUs;
  float dt = (nowUs - lastUs) / 1000000.0f;
  if (dt < 0.005f) return;
  lastUs = nowUs;

  int16_t ax,ay,az,gx,gy,gz,tmp;
  readMPU(ax,ay,az,gx,gy,gz,tmp);

  float gxd = ((float)gx - gBiasX) / 131.0f;
  float gyd = ((float)gy - gBiasY) / 131.0f;
  float gzd = ((float)gz - gBiasZ) / 131.0f;

  angX += gxd * dt;
  angY += gyd * dt;
  angZ += gzd * dt;
  wrap180(angX); wrap180(angY); wrap180(angZ);

  if(streamOn && (millis() - lastPrintMs >= 200)){
    lastPrintMs = millis();
    imuPrintOnce();
  }
}

ImuAngles imuGetAngles(){
  ImuAngles a;
  a.x_deg = angX - offX;
  a.y_deg = angY - offY;
  a.z_deg = angZ - offZ;
  return a;
}

void imuPrintOnce(){
  ImuAngles a = imuGetAngles();
  Serial.printf("[IMU] ang(deg): X=% .2f Y=% .2f Z=% .2f\n", a.x_deg, a.y_deg, a.z_deg);
}

void imuSetStream(bool on){
  streamOn = on;
}
