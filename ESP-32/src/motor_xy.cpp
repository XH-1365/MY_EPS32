#include "motor_xy.h"
#include <math.h>

/* ========= X电机 ========= */
#define X_AIN1 25
#define X_AIN2 26
#define X_PWM  27

/* ========= Y电机 ========= */
#define Y_AIN1 5
#define Y_AIN2 18
#define Y_PWM  19

/* ========= 编码器/霍尔 ========= */
#define X_HALL 34     // 单霍尔（34无内部上拉）
#define Y_ENCA 33     // AB
#define Y_ENCB 32

/* ========= PWM ========= */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH_X 0
#define PWM_CH_Y 1

/* ========= 标定 ========= */
static constexpr float PULSES_PER_MM_X = 273.0f;
static constexpr float PULSES_PER_MM_Y = 6386.86f;

/* ========= 速度/刹车 ========= */
static constexpr int PWM_FAST   = 255;   // 全速
static constexpr int PWM_SLOW_X = 140;   // X 慢速区
static constexpr int PWM_SLOW_Y = 120;   // Y 慢速区

static constexpr float SLOW_ZONE_MM_X = 6.0f;
static constexpr float SLOW_ZONE_MM_Y = 12.0f;

static constexpr int BRAKE_MS_X = 180;
static constexpr int BRAKE_MS_Y = 250;

/* ========= 到位死区 ========= */
static constexpr float DONE_MM_X = 0.20f;
static constexpr float DONE_MM_Y = 0.20f;

static constexpr long DONE_COUNTS_X = (long)(DONE_MM_X * PULSES_PER_MM_X);
static constexpr long DONE_COUNTS_Y = (long)(DONE_MM_Y * PULSES_PER_MM_Y);

/* ========= 计数 ========= */
volatile long encX = 0; // X 单霍尔只++
volatile long encY = 0; // Y AB可正可负

/* ========= X Hall 防抖 ========= */
volatile uint32_t lastHallUs = 0;
void IRAM_ATTR isrXHall() {
  uint32_t now = micros();
  if (now - lastHallUs < 2000) return; // 2ms 去抖
  lastHallUs = now;
  encX++;
}

/* ========= Y AB 解码 ========= */
volatile int lastYA = 0;
void IRAM_ATTR isrYAB() {
  int A = digitalRead(Y_ENCA);
  int B = digitalRead(Y_ENCB);
  if (A != lastYA) {
    encY += (B != A) ? 1 : -1;
    lastYA = A;
  }
}

/* ========= 电机控制 ========= */
static inline void motorRun(int in1, int in2, int ch, int pwmSigned) {
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

static inline void motorBrake(int in1, int in2, int ch, bool on) {
  if (on) {
    // TB6612 的短刹车：IN1=IN2=HIGH
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
    ledcWrite(ch, 255);
  } else {
    ledcWrite(ch, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void motorXYStopBrake() {
  motorBrake(X_AIN1, X_AIN2, PWM_CH_X, true);
  motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, true);
  delay(150);
  motorBrake(X_AIN1, X_AIN2, PWM_CH_X, false);
  motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, false);
}

/* ========= 运动状态 ========= */
struct MoveState {
  bool running = false;
  long start = 0;
  long targetDelta = 0;   // X：正数脉冲; Y：正负
  int dirSign = 1;        // X 单霍尔方向
};

static MoveState mx, my;

static inline bool reachedHall(long cur, long start, long targetPulses) {
  return (cur - start) >= targetPulses;
}

static inline bool reachedAB(long cur, long start, long delta) {
  long moved = cur - start;
  return (delta >= 0) ? (moved >= delta) : (moved <= delta);
}

static inline long remainAbsAB(long cur, long start, long delta) {
  long moved = cur - start;
  return labs(delta - moved);
}

/* ========= 对外：home ========= */
void motorXYHome() {
  noInterrupts();
  encX = 0;
  encY = 0;
  interrupts();
}

/* ========= 对外：启动移动 ========= */
void motorMoveX(float mm) {
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

void motorMoveY(float mm) {
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

/* ========= 内部：更新刹车逻辑 ========= */
static void updateX() {
  if (!mx.running) return;

  long cur, st, target;
  noInterrupts();
  cur = encX; st = mx.start; target = mx.targetDelta;
  interrupts();

  long moved = cur - st;
  float remainMm = (float)labs(target - moved) / PULSES_PER_MM_X;

  // 慢速区
  if (remainMm < SLOW_ZONE_MM_X) {
    motorRun(X_AIN1, X_AIN2, PWM_CH_X, mx.dirSign * PWM_SLOW_X);
  }

  // 到位判定：到了目标 or 进入 deadband
  if (reachedHall(cur, st, target) || labs(target - moved) <= DONE_COUNTS_X) {
    mx.running = false;

    motorBrake(X_AIN1, X_AIN2, PWM_CH_X, true);
    delay(BRAKE_MS_X);
    motorBrake(X_AIN1, X_AIN2, PWM_CH_X, false);

    motorRun(X_AIN1, X_AIN2, PWM_CH_X, 0);
    Serial.println("[X] reached & braked");
  }
}

static void updateY() {
  if (!my.running) return;

  long cur, st, d;
  noInterrupts();
  cur = encY; st = my.start; d = my.targetDelta;
  interrupts();

  float remainMm = (float)remainAbsAB(cur, st, d) / PULSES_PER_MM_Y;

  // 慢速区
  if (remainMm < SLOW_ZONE_MM_Y) {
    int dir = (d >= 0) ? 1 : -1;
    motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, dir * PWM_SLOW_Y);
  }

  // 到位判定：到了目标 or 进入 deadband
  if (reachedAB(cur, st, d) || remainAbsAB(cur, st, d) <= DONE_COUNTS_Y) {
    my.running = false;

    motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, true);
    delay(BRAKE_MS_Y);
    motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, false);

    motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, 0);
    Serial.println("[Y] reached & braked");
  }
}

/* ========= 对外：周期更新 ========= */
void motorXYUpdate() {
  updateX();
  updateY();
}

/* ========= 对外：打印位置 ========= */
void motorPrintPos() {
  long x, y;
  noInterrupts(); x = encX; y = encY; interrupts();
  Serial.printf("encX=%ld Xmm=%.3f | encY=%ld Ymm=%.3f\n",
                x, (float)x / PULSES_PER_MM_X,
                y, (float)y / PULSES_PER_MM_Y);
}

/* ========= 初始化 ========= */
void motorXYBegin() {
  // 电机引脚
  pinMode(X_AIN1, OUTPUT);
  pinMode(X_AIN2, OUTPUT);
  pinMode(Y_AIN1, OUTPUT);
  pinMode(Y_AIN2, OUTPUT);

  // PWM
  ledcSetup(PWM_CH_X, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_Y, PWM_FREQ, PWM_RES);
  ledcAttachPin(X_PWM, PWM_CH_X);
  ledcAttachPin(Y_PWM, PWM_CH_Y);

  // 编码器/霍尔
  pinMode(X_HALL, INPUT);          // 34无上拉（必要时外接上拉）
  pinMode(Y_ENCA, INPUT_PULLUP);
  pinMode(Y_ENCB, INPUT_PULLUP);

  // 预读一次，避免 lastYA 初值怪
  lastYA = digitalRead(Y_ENCA);

  attachInterrupt(digitalPinToInterrupt(X_HALL), isrXHall, RISING);
  attachInterrupt(digitalPinToInterrupt(Y_ENCA), isrYAB, CHANGE);

  // 初始停
  motorRun(X_AIN1, X_AIN2, PWM_CH_X, 0);
  motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, 0);

  motorXYHome();
  Serial.println("[motorXY] init OK");
}
