#include <Arduino.h>
#include <Wire.h>
#include <math.h>

/* =========================
   MPU6050（直接读寄存器，不做WHO_AM_I判断）
   角度 = 陀螺积分（deg），home可清零
   ========================= */
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

void readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz,int16_t &tmp){
  ax = mpuRead16(0x3B);
  ay = mpuRead16(0x3D);
  az = mpuRead16(0x3F);
  tmp= mpuRead16(0x41);
  gx = mpuRead16(0x43);
  gy = mpuRead16(0x45);
  gz = mpuRead16(0x47);
}

// 陀螺零偏（LSB）
float gBiasX=0, gBiasY=0, gBiasZ=0;

// “能用就行”的初始化：只要唤醒成功、后续能读到数据，就算OK
bool imuBegin(){
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // 唤醒
  mpuWrite(0x6B, 0x00);
  delay(80);

  // Gyro ±250 dps, Accel ±2g
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
  delay(10);

  // 不检查WHO_AM_I，直接返回 true
  return true;
}

void calibrateGyro(int samples=600){
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

static inline void wrap180(float &a){
  while(a > 180) a -= 360;
  while(a < -180) a += 360;
}

/* ========= 角度（deg） ========= */
float angX=0, angY=0, angZ=0;
float offX=0, offY=0, offZ=0;         // 零点偏移：home时把当前角度当作0
bool imuStream=false;
unsigned long lastUs = 0;
unsigned long lastPrintMs = 0;

void imuZero(){ // ✅ 你要的“清零”
  // 方式A：积分清零（更直观）
  angX = angY = angZ = 0;
  // 方式B：也可以用offset（防漂一点点），这里一起用：把当前当0
  offX = offY = offZ = 0;
  lastUs = micros();
}

void imuUpdate(){
  unsigned long nowUs = micros();
  if (lastUs == 0) lastUs = nowUs;
  float dt = (nowUs - lastUs) / 1000000.0f;
  if (dt < 0.005f) return; // ~200Hz
  lastUs = nowUs;

  int16_t ax,ay,az,gx,gy,gz,tmp;
  readMPU(ax,ay,az,gx,gy,gz,tmp);

  // dps
  float gxd = ((float)gx - gBiasX) / 131.0f;
  float gyd = ((float)gy - gBiasY) / 131.0f;
  float gzd = ((float)gz - gBiasZ) / 131.0f;

  // 积分 -> deg
  angX += gxd * dt;
  angY += gyd * dt;
  angZ += gzd * dt;
  wrap180(angX); wrap180(angY); wrap180(angZ);

  if(imuStream && (millis() - lastPrintMs >= 200)){
    lastPrintMs = millis();
    Serial.printf("[IMU] ang(deg): X=% .2f Y=% .2f Z=% .2f\n",
                  angX - offX, angY - offY, angZ - offZ);
  }
}

void imuPrintOnce(){
  Serial.printf("[IMU] ang(deg): X=% .2f Y=% .2f Z=% .2f\n",
                angX - offX, angY - offY, angZ - offZ);
}

/* =========================
   XY 精准停下（保持你那套逻辑）
   X：单霍尔
   Y：AB
   ========================= */

/* ========= X电机 ========= */
#define X_AIN1 25
#define X_AIN2 26
#define X_PWM  27

/* ========= Y电机 ========= */
#define Y_AIN1 5
#define Y_AIN2 18
#define Y_PWM  19

/* ========= 编码器/霍尔 ========= */
#define X_HALL 34
#define Y_ENCA 33
#define Y_ENCB 32

/* ========= PWM ========= */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH_X 0
#define PWM_CH_Y 1

/* ========= 标定 ========= */
#define PULSES_PER_MM_X 273.0f
#define PULSES_PER_MM_Y 6386.86f

/* ========= 速度/刹车 ========= */
#define PWM_FAST 255
#define PWM_SLOW_X 140
#define PWM_SLOW_Y 120
#define SLOW_ZONE_MM_X 6.0f
#define SLOW_ZONE_MM_Y 12.0f
#define BRAKE_MS_X 180
#define BRAKE_MS_Y 250

/* ========= 到位死区 ========= */
#define DONE_MM_X 0.20f
#define DONE_MM_Y 0.20f
#define DONE_COUNTS_X ((long)(DONE_MM_X * PULSES_PER_MM_X))
#define DONE_COUNTS_Y ((long)(DONE_MM_Y * PULSES_PER_MM_Y))

volatile long encX = 0; // 单霍尔只++
volatile long encY = 0; // AB可正可负

// X hall 防抖
volatile uint32_t lastHallUs = 0;
void IRAM_ATTR isrXHall() {
  uint32_t now = micros();
  if(now - lastHallUs < 2000) return;
  lastHallUs = now;
  encX++;
}

volatile int lastYA = 0;
void IRAM_ATTR isrYAB() {
  int A = digitalRead(Y_ENCA);
  int B = digitalRead(Y_ENCB);
  if (A != lastYA) {
    encY += (B != A) ? 1 : -1;
    lastYA = A;
  }
}

void motorRun(int in1, int in2, int ch, int pwmSigned) {
  if (pwmSigned > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(ch, (uint8_t)constrain(pwmSigned, 0, 255));
  } else if (pwmSigned < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(ch, (uint8_t)constrain(-pwmSigned, 0, 255));
  } else {
    ledcWrite(ch, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void motorBrake(int in1, int in2, int ch, bool on) {
  if (on) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    ledcWrite(ch, 255);
  } else {
    ledcWrite(ch, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void stopAllBrake() {
  motorBrake(X_AIN1, X_AIN2, PWM_CH_X, true);
  motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, true);
  delay(150);
  motorBrake(X_AIN1, X_AIN2, PWM_CH_X, false);
  motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, false);
}

struct MoveState {
  bool running=false;
  long start=0;
  long targetDelta=0; // X:正数；Y:正负
  int dirSign=1;
};
MoveState mx, my;

bool reachedHall(long cur, long start, long targetPulses){
  return (cur - start) >= targetPulses;
}
bool reachedAB(long cur, long start, long delta){
  long moved = cur - start;
  return (delta >= 0) ? (moved >= delta) : (moved <= delta);
}
long remainAbsAB(long cur, long start, long delta){
  long moved = cur - start;
  return labs(delta - moved);
}

void startMoveX(float mm) {
  long pulses = (long)lroundf(fabsf(mm) * PULSES_PER_MM_X);
  if (pulses <= 0) return;

  noInterrupts();
  mx.start = encX;
  mx.targetDelta = pulses;
  mx.running = true;
  interrupts();

  mx.dirSign = (mm >= 0) ? 1 : -1;
  motorRun(X_AIN1, X_AIN2, PWM_CH_X, mx.dirSign * PWM_FAST);
  Serial.printf("[X] move %.3fmm pulses=%ld\n", mm, pulses);
}

void startMoveY(float mm) {
  long pulses = (long)lroundf(fabsf(mm) * PULSES_PER_MM_Y);
  if (pulses <= 0) return;

  noInterrupts();
  my.start = encY;
  my.targetDelta = (mm >= 0) ? pulses : -pulses;
  my.running = true;
  interrupts();

  int dir = (mm >= 0) ? 1 : -1;
  motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, dir * PWM_FAST);
  Serial.printf("[Y] move %.3fmm pulses=%ld\n", mm, pulses);
}

void updateX() {
  if(!mx.running) return;

  long cur, st, target;
  noInterrupts(); cur = encX; st = mx.start; target = mx.targetDelta; interrupts();

  long moved = cur - st;
  float remainMm = (float)labs(target - moved) / PULSES_PER_MM_X;

  if(remainMm < SLOW_ZONE_MM_X){
    motorRun(X_AIN1, X_AIN2, PWM_CH_X, mx.dirSign * PWM_SLOW_X);
  }

  if(reachedHall(cur, st, target) || labs(target - moved) <= DONE_COUNTS_X){
    mx.running = false;
    motorBrake(X_AIN1, X_AIN2, PWM_CH_X, true);
    delay(BRAKE_MS_X);
    motorBrake(X_AIN1, X_AIN2, PWM_CH_X, false);
    motorRun(X_AIN1, X_AIN2, PWM_CH_X, 0);
    Serial.println("[X] reached & braked");
  }
}

void updateY() {
  if(!my.running) return;

  long cur, st, d;
  noInterrupts(); cur = encY; st = my.start; d = my.targetDelta; interrupts();

  float remainMm = (float)remainAbsAB(cur, st, d) / PULSES_PER_MM_Y;

  if(remainMm < SLOW_ZONE_MM_Y){
    int dir = (d >= 0) ? 1 : -1;
    motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, dir * PWM_SLOW_Y);
  }

  if(reachedAB(cur, st, d) || remainAbsAB(cur, st, d) <= DONE_COUNTS_Y){
    my.running = false;
    motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, true);
    delay(BRAKE_MS_Y);
    motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, false);
    motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, 0);
    Serial.println("[Y] reached & braked");
  }
}

/* ========= 串口 ========= */
String line;

void printPos(){
  long x,y;
  noInterrupts(); x=encX; y=encY; interrupts();
  Serial.printf("encX=%ld Xmm=%.3f | encY=%ld Ymm=%.3f\n",
    x, (float)x / PULSES_PER_MM_X,
    y, (float)y / PULSES_PER_MM_Y
  );
}

void setup(){
  Serial.begin(115200);
  delay(300);

  // 电机
  pinMode(X_AIN1, OUTPUT); pinMode(X_AIN2, OUTPUT);
  pinMode(Y_AIN1, OUTPUT); pinMode(Y_AIN2, OUTPUT);

  ledcSetup(PWM_CH_X, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_Y, PWM_FREQ, PWM_RES);
  ledcAttachPin(X_PWM, PWM_CH_X);
  ledcAttachPin(Y_PWM, PWM_CH_Y);

  motorRun(X_AIN1, X_AIN2, PWM_CH_X, 0);
  motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, 0);

  // IMU：不检查whoami，直接用能读的方式
  Serial.println("IMU begin...");
  imuBegin();
  Serial.println("Keep MPU still, calibrating gyro...");
  calibrateGyro(600);
  Serial.printf("Gyro bias: %.1f %.1f %.1f (LSB)\n", gBiasX, gBiasY, gBiasZ);
  imuZero();

  // 编码器/霍尔
  pinMode(X_HALL, INPUT);          // GPIO34无上拉，建议外部上拉
  pinMode(Y_ENCA, INPUT_PULLUP);
  pinMode(Y_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(X_HALL), isrXHall, RISING);
  attachInterrupt(digitalPinToInterrupt(Y_ENCA), isrYAB, CHANGE);

  Serial.println("=== Precise Stop XY + IMU(angles) ===");
  Serial.println("cmd: home | pos | mx <mm> | my <mm> | imu | imuon | imuoff | stop");
  Serial.println("ex: mx 2  / my -2 / imu");
}

void loop(){
  imuUpdate();
  updateX();
  updateY();

  while(Serial.available()){
    char c = (char)Serial.read();
    if(c=='\n' || c=='\r'){
      line.trim();
      if(line.length()){
        Serial.print(">> "); Serial.println(line);

        if(line=="stop"){
          mx.running = my.running = false;
          stopAllBrake();
          Serial.println("OK stop");
        } else if(line=="home"){
          noInterrupts(); encX=0; encY=0; interrupts();
          imuZero();  // ✅ home 同时角度清零
          Serial.println("OK home (encX=encY=0, ang=0)");
        } else if(line=="pos"){
          printPos();
        } else if(line=="imu"){
          imuPrintOnce();  // ✅ imu直接显示角度
        } else if(line=="imuon"){
          imuStream = true;
          Serial.println("IMU angle stream ON (200ms)");
        } else if(line=="imuoff"){
          imuStream = false;
          Serial.println("IMU angle stream OFF");
        } else if(line.startsWith("mx ")){
          float mm = line.substring(3).toFloat();
          startMoveX(mm);
        } else if(line.startsWith("my ")){
          float mm = line.substring(3).toFloat();
          startMoveY(mm);
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
