#include <Arduino.h>
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
// X：单霍尔（你以前那套）
#define X_HALL 34

// Y：AB 相
#define Y_ENCA 33
#define Y_ENCB 32

/* ========= PWM ========= */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH_X 0
#define PWM_CH_Y 1

/* ========= 你的标定 ========= */
#define PULSES_PER_MM_X 273.0f     // 你之前用的（可调）
#define PULSES_PER_MM_Y 6386.86f   // 15073 / 2.36mm

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

/* ========= 计数 ========= */
volatile long encX = 0; // X 单霍尔：只++（方向靠电机方向）
volatile long encY = 0; // Y AB：可正可负

// X 单霍尔中断
void IRAM_ATTR isrXHall() { encX++; }

// Y AB中断（A相触发，读AB判向）
volatile int lastYA = 0;
void IRAM_ATTR isrYAB() {
  int A = digitalRead(Y_ENCA);
  int B = digitalRead(Y_ENCB);
  if (A != lastYA) {
    encY += (B != A) ? 1 : -1;
    lastYA = A;
  }
}

/* ========= 电机控制：真刹车 ========= */
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

// TB6612类真短刹：IN1=IN2=HIGH 且 PWM=255
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

/* ========= 运动状态结构 ========= */
struct MoveState {
  bool running = false;
  long start = 0;
  long targetDelta = 0;   // 本次要走的脉冲（正负）
  float pulsesPerMm = 1;
  float slowZoneMm = 5;
  int pwmSlow = 120;
  int pwmFast = 255;
  long doneCounts = 100;
  int brakeMs = 200;
  int in1, in2, ch;
  volatile long* enc;
  // 对于单霍尔轴：需要记住“走动方向”（因为 enc 只++）
  int dirSign = 1;
};

MoveState mx, my;

bool reachedAB(long cur, long start, long delta) {
  long moved = cur - start;
  return (delta >= 0) ? (moved >= delta) : (moved <= delta);
}
long remainAbsAB(long cur, long start, long delta) {
  long moved = cur - start;
  long remain = delta - moved;
  return labs(remain);
}

// 单霍尔：enc 只++，所以 moved = enc - start（永远正），必须用 dirSign 判断
bool reachedHall(long cur, long start, long targetPulses) {
  long moved = cur - start; // >=0
  return moved >= targetPulses;
}

void startMoveX(float mm) {
  long pulses = (long)lroundf(fabsf(mm) * PULSES_PER_MM_X);
  if (pulses <= 0) return;

  noInterrupts();
  mx.start = encX;
  mx.targetDelta = pulses; // 单霍尔用正数表示“需要走的脉冲数”
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
  if (!mx.running) return;

  long cur, st, target;
  noInterrupts();
  cur = encX; st = mx.start; target = mx.targetDelta;
  interrupts();

  long moved = cur - st; // 单霍尔 always >=0
  float remainMm = (float)labs(target - moved) / PULSES_PER_MM_X;

  if (remainMm < SLOW_ZONE_MM_X) {
    motorRun(X_AIN1, X_AIN2, PWM_CH_X, mx.dirSign * PWM_SLOW_X);
  }

  if (reachedHall(cur, st, target) || labs(target - moved) <= DONE_COUNTS_X) {
    mx.running = false;
    motorBrake(X_AIN1, X_AIN2, PWM_CH_X, true);
    delay(BRAKE_MS_X);
    motorBrake(X_AIN1, X_AIN2, PWM_CH_X, false);
    motorRun(X_AIN1, X_AIN2, PWM_CH_X, 0);
    Serial.println("[X] reached & braked");
  }
}

void updateY() {
  if (!my.running) return;

  long cur, st, d;
  noInterrupts();
  cur = encY; st = my.start; d = my.targetDelta;
  interrupts();

  float remainMm = (float)remainAbsAB(cur, st, d) / PULSES_PER_MM_Y;

  if (remainMm < SLOW_ZONE_MM_Y) {
    int dir = (d >= 0) ? 1 : -1;
    motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, dir * PWM_SLOW_Y);
  }

  if (reachedAB(cur, st, d) || remainAbsAB(cur, st, d) <= DONE_COUNTS_Y) {
    my.running = false;
    motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, true);
    delay(BRAKE_MS_Y);
    motorBrake(Y_AIN1, Y_AIN2, PWM_CH_Y, false);
    motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, 0);
    Serial.println("[Y] reached & braked");
  }
}

/* ========= 串口命令 ========= */
String line;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(X_AIN1, OUTPUT); pinMode(X_AIN2, OUTPUT);
  pinMode(Y_AIN1, OUTPUT); pinMode(Y_AIN2, OUTPUT);

  // X_HALL=34 没内部上拉，如果你的霍尔模块没自带上拉，必须外接10k上拉到3.3V
  pinMode(X_HALL, INPUT);
  pinMode(Y_ENCA, INPUT_PULLUP);
  pinMode(Y_ENCB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(X_HALL), isrXHall, RISING);
  attachInterrupt(digitalPinToInterrupt(Y_ENCA), isrYAB, CHANGE);

  ledcSetup(PWM_CH_X, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_Y, PWM_FREQ, PWM_RES);
  ledcAttachPin(X_PWM, PWM_CH_X);
  ledcAttachPin(Y_PWM, PWM_CH_Y);

  motorRun(X_AIN1, X_AIN2, PWM_CH_X, 0);
  motorRun(Y_AIN1, Y_AIN2, PWM_CH_Y, 0);

  Serial.println("=== Precise Stop XY Console ===");
  Serial.println("cmd: home | pos | mx <mm> | my <mm> | stop");
  Serial.println("ex: mx 2  / my -2");
}

void loop() {
  updateX();
  updateY();

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      if (line.length()) {
        Serial.print(">> "); Serial.println(line);

        if (line == "stop") {
          mx.running = my.running = false;
          stopAllBrake();
          Serial.println("OK stop");
        } else if (line == "home") {
          noInterrupts(); encX = 0; encY = 0; interrupts();
          Serial.println("OK home (encX=encY=0)");
        } else if (line == "pos") {
          long x, y;
          noInterrupts(); x = encX; y = encY; interrupts();
          Serial.printf("encX=%ld Xmm=%.3f | encY=%ld Ymm=%.3f\n",
                        x, (float)x / PULSES_PER_MM_X,
                        y, (float)y / PULSES_PER_MM_Y);
        } else if (line.startsWith("mx ")) {
          float mm = line.substring(3).toFloat();
          startMoveX(mm);
        } else if (line.startsWith("my ")) {
          float mm = line.substring(3).toFloat();
          startMoveY(mm);
        } else {
          Serial.println("cmd: home | pos | mx <mm> | my <mm> | stop");
        }
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 80) line = "";
    }
  }
}
