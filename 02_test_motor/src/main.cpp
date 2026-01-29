#include <Arduino.h>

/* ===== 电机引脚 ===== */
#define AIN1 3
#define AIN2 4
#define PWMA 5

/* ===== 编码器 ===== */
#define HALL_A 7
#define HALL_B 6

/* ===== PWM ===== */
#define PWM_CH   0
#define PWM_FREQ 20000
#define PWM_RES  8

/* ===== 机械参数 ===== */
#define ENCODER_PPR   520.0
#define GEAR_RATIO   1030.0
#define COUNT_PER_DEG (ENCODER_PPR * GEAR_RATIO / 360.0)

/* ===== PID 参数（已为 1:1030 缩放） ===== */
#define KP  3.0
#define KI  0.01
#define KD  15.0

/* ===== 控制阈值（关键） ===== */
#define STOP_ERR_DEG   0.15     // 到位死区
#define SLOW_ERR_DEG   3.0      // 进入慢速区
#define MAX_PWM        160
#define MIN_PWM        40

volatile long encoderCount = 0;
volatile int lastA = 0;

/* ===== PID 状态 ===== */
float integral = 0;
float lastErr = 0;

/* ===== 编码器中断 ===== */
void IRAM_ATTR hallISR() {
  int A = digitalRead(HALL_A);
  int B = digitalRead(HALL_B);

  if (A != lastA) {
    encoderCount += (B != A) ? 1 : -1;
    lastA = A;
  }
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(HALL_A, INPUT_PULLUP);
  pinMode(HALL_B, INPUT_PULLUP);

  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH);
  ledcWrite(PWM_CH, 0);

  attachInterrupt(digitalPinToInterrupt(HALL_A), hallISR, CHANGE);

  Serial.begin(115200);
  encoderCount = 0;   // ⚠️ 临时零点
}

/* ===== 角度归一化（最近路径） ===== */
float normalizeAngle(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void loop() {
  float targetAngle = 5.0;   // 测试用
  float currentAngle = encoderCount / COUNT_PER_DEG;

  float err = normalizeAngle(targetAngle - currentAngle);

  /* ===== 到位：直接断 PWM ===== */
  if (abs(err) < STOP_ERR_DEG) {
    ledcWrite(PWM_CH, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    integral = 0;   // 防止 I 抖
    lastErr = err;
    return;
  }

  /* ===== PID ===== */
  integral += err;
  integral = constrain(integral, -50, 50);   // 抗积分饱和

  float derivative = err - lastErr;
  lastErr = err;

  float output = KP * err + KI * integral + KD * derivative;

  /* ===== 方向 ===== */
  if (output > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    output = -output;
  }

  /* ===== PWM 限制 ===== */
  int pwm = constrain(output, MIN_PWM, MAX_PWM);

  // 慢速区进一步压小
  if (abs(err) < SLOW_ERR_DEG) {
    pwm = MIN_PWM;
  }

  ledcWrite(PWM_CH, pwm);

  Serial.printf("Cur=%.3f  Err=%.3f  PWM=%d\n",
                currentAngle, err, pwm);

  delay(10);
}
