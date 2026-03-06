#include "Motion.h"
#include "Config.h"

/* ===== encoder counts ===== */
volatile long encX = 0;
volatile long encY = 0;
volatile int  lastYA = 0;
volatile int  xDirSignISR = +1;

long curEncX(){ long v; noInterrupts(); v=encX; interrupts(); return v; }
long curEncY(){ long v; noInterrupts(); v=encY; interrupts(); return v; }

float curXmm(){ return (float)curEncX() / PULSES_PER_MM_X; }
float curYmm(){ return (float)curEncY() / PULSES_PER_MM_Y; }

void IRAM_ATTR isrXHall(){ encX += xDirSignISR; }
void IRAM_ATTR isrYAB(){
  int A = digitalRead(Y_ENCA);
  int B = digitalRead(Y_ENCB);
  if (A != lastYA) {
    encY += (B != A) ? 1 : -1;
    lastYA = A;
  }
}

/* ===== motor control ===== */
static inline void motorRun(int in1,int in2,int ch,int pwmSigned){
  if(pwmSigned>0){ digitalWrite(in1,HIGH); digitalWrite(in2,LOW); ledcWrite(ch, (uint8_t)constrain(pwmSigned,0,255)); }
  else if(pwmSigned<0){ digitalWrite(in1,LOW); digitalWrite(in2,HIGH); ledcWrite(ch, (uint8_t)constrain(-pwmSigned,0,255)); }
  else { ledcWrite(ch,0); digitalWrite(in1,LOW); digitalWrite(in2,LOW); }
}
static inline void motorBrake(int in1,int in2,int ch,bool on){
  if(on){ digitalWrite(in1,HIGH); digitalWrite(in2,HIGH); ledcWrite(ch,255); }
  else { ledcWrite(ch,0); digitalWrite(in1,LOW); digitalWrite(in2,LOW); }
}

void stopAllBrake(){
  motorBrake(X_AIN1,X_AIN2,PWM_CH_X,true);
  motorBrake(Y_AIN1,Y_AIN2,PWM_CH_Y,true);
  delay(120);
  motorBrake(X_AIN1,X_AIN2,PWM_CH_X,false);
  motorBrake(Y_AIN1,Y_AIN2,PWM_CH_Y,false);
}

/* ===== move state ===== */
struct MoveState{ bool running=false; long start=0; long targetDelta=0; };
static MoveState mx,my;

bool isMovingX(){ return mx.running; }
bool isMovingY(){ return my.running; }

static inline bool reachedAB(long cur,long start,long delta){
  long moved = cur - start;
  return (delta>=0) ? (moved>=delta) : (moved<=delta);
}
static inline long remainAbsAB(long cur,long start,long delta){
  long moved = cur - start;
  long remain = delta - moved;
  return labs(remain);
}
static inline long doneCountsX(){ return (long)lroundf(DONE_MM_X * PULSES_PER_MM_X); }
static inline long doneCountsY(){ return (long)lroundf(DONE_MM_Y * PULSES_PER_MM_Y); }

void startMoveX(float mm){
  long pulses = (long)lroundf(fabsf(mm) * PULSES_PER_MM_X);
  if(pulses<=0) return;
  int dir = (mm>=0)?1:-1;

  noInterrupts();
  mx.start = encX;
  mx.targetDelta = dir * pulses;
  mx.running = true;
  xDirSignISR = dir;
  interrupts();

  motorRun(X_AIN1,X_AIN2,PWM_CH_X, dir*PWM_FAST);
}

void startMoveY(float mm){
  long pulses = (long)lroundf(fabsf(mm) * PULSES_PER_MM_Y);
  if(pulses<=0) return;
  int dir = (mm>=0)?1:-1;
#if Y_INVERT
  dir = -dir;
#endif
  long delta = (mm>=0)?pulses:-pulses;
#if Y_INVERT
  delta = -delta;
#endif

  noInterrupts();
  my.start = encY;
  my.targetDelta = delta;
  my.running = true;
  interrupts();

  motorRun(Y_AIN1,Y_AIN2,PWM_CH_Y, dir*PWM_FAST);
}

static void updateX(){
  if(!mx.running) return;
  long cur=curEncX(), st=mx.start, d=mx.targetDelta;

  float remainMm = (float)remainAbsAB(cur,st,d) / PULSES_PER_MM_X;
  if(remainMm < SLOW_ZONE_MM_X){
    int dir = (d>=0)?1:-1;
    motorRun(X_AIN1,X_AIN2,PWM_CH_X, dir*PWM_SLOW_X);
  }

  if(reachedAB(cur,st,d) || remainAbsAB(cur,st,d)<=doneCountsX()){
    mx.running=false;
    motorBrake(X_AIN1,X_AIN2,PWM_CH_X,true);
    delay(BRAKE_MS_X);
    motorBrake(X_AIN1,X_AIN2,PWM_CH_X,false);
    motorRun(X_AIN1,X_AIN2,PWM_CH_X,0);
  }
}
static void updateY(){
  if(!my.running) return;
  long cur=curEncY(), st=my.start, d=my.targetDelta;

  float remainMm = (float)remainAbsAB(cur,st,d) / PULSES_PER_MM_Y;
  if(remainMm < SLOW_ZONE_MM_Y){
    int dir = (d>=0)?1:-1;
    motorRun(Y_AIN1,Y_AIN2,PWM_CH_Y, dir*PWM_SLOW_Y);
  }

  if(reachedAB(cur,st,d) || remainAbsAB(cur,st,d)<=doneCountsY()){
    my.running=false;
    motorBrake(Y_AIN1,Y_AIN2,PWM_CH_Y,true);
    delay(BRAKE_MS_Y);
    motorBrake(Y_AIN1,Y_AIN2,PWM_CH_Y,false);
    motorRun(Y_AIN1,Y_AIN2,PWM_CH_Y,0);
  }
}

void motionUpdate(){
  updateX();
  updateY();
}

void waitMoveDone(){
  while(mx.running || my.running){
    motionUpdate();
    delay(1);
  }
}

void moveToAbs(float x_mm,float y_mm){
  float dx = x_mm - curXmm();
  float dy = y_mm - curYmm();
  if(fabsf(dx)>0.0001f) startMoveX(dx);
  if(fabsf(dy)>0.0001f) startMoveY(dy);
  waitMoveDone();
}

void zeroEncoders(){
  noInterrupts(); encX=0; encY=0; interrupts();
  mx.running=false; my.running=false;
}

void motionInit(){
  pinMode(X_AIN1,OUTPUT); pinMode(X_AIN2,OUTPUT);
  pinMode(Y_AIN1,OUTPUT); pinMode(Y_AIN2,OUTPUT);

  pinMode(X_HALL,INPUT);
  pinMode(Y_ENCA,INPUT_PULLUP);
  pinMode(Y_ENCB,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(X_HALL), isrXHall, RISING);
  attachInterrupt(digitalPinToInterrupt(Y_ENCA), isrYAB, CHANGE);

  ledcSetup(PWM_CH_X, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_Y, PWM_FREQ, PWM_RES);
  ledcAttachPin(X_PWM, PWM_CH_X);
  ledcAttachPin(Y_PWM, PWM_CH_Y);

  motorRun(X_AIN1,X_AIN2,PWM_CH_X,0);
  motorRun(Y_AIN1,Y_AIN2,PWM_CH_Y,0);
}