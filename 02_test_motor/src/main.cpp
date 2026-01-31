// 恢复最小移动/打点实现并增加打印队列（由主循环执行，HTTP 仅下发任务）
#include <stddef.h>

// 声明 schedule 函数供 wifi_server 调用
void scheduleBraillePrint(const char* utf8text);

// 前向声明：移动/打点函数（实现下方）
long mmToCounts(float mm);
void moveAxisToAbsoluteMM(bool isLeftAxis, float target_mm, int pwm, int minpwm);
void moveToXYBlocking(float x_mm, float y_mm, int pwm);
void triggerCut();
void printDot(float x_mm, float y_mm, int pwm);
#include <Arduino.h>

// 来自 initialization.CPP 的初始化函数
void runInitialization();
// WiFi server functions (implemented in wifi_server.cpp)
void setupWiFi();
void handleServer();
// 供 web server 切换右轴反向使用
void toggleInvertRight();
bool getInvertRight();
// 对外接口：重置 home（将当前编码器计数与位姿归零）
void resetHome();
// 对外接口：设置角度（度数）
void setThetaDeg(float deg);
// 对外接口：调度单轴移动（由 web 线程设置，主循环执行以避免阻塞）
void scheduleAxisMove(bool isLeftAxis, float pos_mm, int pwm);
// 对外接口：紧急停止所有电机
void stopAll();

/* ========= 左电机 ========= */
#define AIN1_L 3
#define AIN2_L 4
#define PWM_L  5
#define HALL_A_L 7
#define HALL_B_L 6

/* ========= 右电机 ========= */
#define AIN1_R 2
#define AIN2_R 1
#define PWM_R  10
#define HALL_A_R 9
#define HALL_B_R 8 

/* ========= PWM ========= */
#define PWM_FREQ 20000
#define PWM_RES  8
#define PWM_CH_L 0
#define PWM_CH_R 1

/* ========= 机械参数 ========= */
#define ENCODER_PPR   520.0
#define GEAR_RATIO    1030.0
#define COUNT_PER_DEG (ENCODER_PPR * GEAR_RATIO / 360.0)

#define WHEEL_BASE  150.0   // 左右轮间距 mm
#define WHEEL_RADIUS 20.0   // 电机轮半径 mm

/* ========= PID 参数（稳） ========= */
#define KP_POS   2.0
#define KD_POS   15.0
#define KP_SYNC  1.2
#define KD_SYNC  6.0

/* ========= 控制阈值 ========= */
#define STOP_ERR_DEG  0.15
#define MAX_PWM       150
#define MIN_PWM_L     45
#define MIN_PWM_R     45

volatile long encL = 0;
volatile long encR = 0;
volatile int lastAL = 0;
volatile int lastAR = 0;

// 右轴（Y）方向反转开关——用于软件翻转方向测试
volatile bool invertRightAxis = false;

/* ========= 中断 ========= */
void IRAM_ATTR hallISR_L() {
  int A = digitalRead(HALL_A_L);
  int B = digitalRead(HALL_B_L);
  if (A != lastAL) {
    encL += (B != A) ? 1 : -1;
    lastAL = A;
  }
}

void IRAM_ATTR hallISR_R() {
  int A = digitalRead(HALL_A_R);
  int B = digitalRead(HALL_B_R);
  if (A != lastAR) {
    encR += (B != A) ? 1 : -1;
    lastAR = A;
  }
}

/* ========= 状态 ========= */
float lastPosErr = 0;
float lastSyncErr = 0;
bool moving = false;

/* ========= XY 控制目标 ========= */
float targetX = 200;  // mm
float targetY = 50;   // mm

/* ========= 机器人位姿（x,y,theta） ========= */
float currentX = 0;
float currentY = 0;
float currentTheta = 0; // rad

long prevEncL = 0;
long prevEncR = 0;

// ---- 单轴调度标志（由网页设置，主循环读取） ----
volatile bool axisMovePending = false;
volatile bool axisMoveIsLeft = false;
volatile float axisMovePos = 0.0f;
volatile int axisMovePwm = 0;

// 请求回到 home（由网页触发，主循环执行）
volatile bool goHomePending = false;

// web 可调用的接口
void requestGoHome();

// 校准（半自动）：在网页上开始移动，用户观察移动到物理刻度后按 Stop
volatile bool calibRunning = false;
volatile bool calibIsLeft = false;
volatile long calibStartEnc = 0;
float calibTargetMM = 100.0f;
int calibPwm = 120;

// 测量结果（运行时），将在 setup() 中用宏初始化
float countsPerMmL;
float countsPerMmR;

// 控制参数
#define KP_LINEAR  1.0   // 线速度比例
#define KP_ANGULAR 4.0   // 角速度比例
#define POS_TOL_MM 2.0   // 到位距离阈值 mm

// 物理打印板尺寸（mm）
#define PLATE_LEN_MM 76.0
#define PLATE_WID_MM 34.0

// 编码器计数与线性位移关系
#define COUNTS_PER_MM (COUNT_PER_DEG * 360.0 / (2.0 * PI * WHEEL_RADIUS))

// 切削/打点触发引脚
#define CUT_PIN 11

// 演示开关：1=上电运行 demo 打点序列，0=不自动运行
#define DEMO_PRINT 0

// 如果你的左右电机与X/Y轴映射不符，设为 1 将交换 X/Y 映射
#define SWAP_XY 0

// 运行第一排打点（上电自动运行一次）
#define RUN_PRINT_FIRST_ROW 0

// 第一排参数（可按需调整）
#define FIRST_ROW_COUNT 10
#define FIRST_ROW_START_X 5.0
#define FIRST_ROW_SPACING 7.0
#define FIRST_ROW_Y 5.0
#define FIRST_ROW_PWM 120

// 打印网格（用于图中小区域）
#define RUN_PRINT_GRID 0
#define GRID_COLS 10
#define GRID_ROWS 5
#define GRID_START_X 8.0
#define GRID_START_Y 6.0
#define GRID_SPACING_X 1.6
#define GRID_SPACING_Y 2.5
#define GRID_PWM 120

/* ========= 使用编码器增量更新位姿 ========= */
void updatePose() {
  long curL = encL;
  long curR = encR;

  long dCountL = curL - prevEncL;
  long dCountR = curR - prevEncR;

  prevEncL = curL;
  prevEncR = curR;

  float angleL_deg = dCountL / COUNT_PER_DEG;  // deg
  float angleR_deg = dCountR / COUNT_PER_DEG;  // deg

  float distL = angleL_deg * 2.0 * PI * WHEEL_RADIUS / 360.0; // mm
  float distR = angleR_deg * 2.0 * PI * WHEEL_RADIUS / 360.0; // mm

  float ds = (distL + distR) / 2.0;
  float dtheta = (distR - distL) / WHEEL_BASE; // rad (approx)

  float midTheta = currentTheta + dtheta / 2.0;
  currentX += ds * cos(midTheta);
  currentY += ds * sin(midTheta);
  currentTheta += dtheta;

  // 规范化角度到 [-PI, PI]
  while (currentTheta > PI) currentTheta -= 2 * PI;
  while (currentTheta < -PI) currentTheta += 2 * PI;
}



// 打印任务结构（最多支持 2000 点）
#define MAX_JOB_POINTS 2000
struct DotPoint { float x; float y; };
volatile bool jobPending = false;
DotPoint jobPoints[MAX_JOB_POINTS];
int jobCount = 0;
int jobIndex = 0;

// 将 UTF-8 文本转换为盲文点并排入 jobPoints（非常简单的 a-z 映射）
void scheduleBraillePrint(const char* utf8text) {
  // 简单映射：只处理 a-z 和空格
  static const unsigned char braille_map[27] = {
    // a b c d e f g h i j k l m n o p q r s t u v w x y z (space->0)
    0b000001,0b000011,0b000101,0b001101,0b001001,0b000111,0b001111,0b001011,0b000110,0b001110,
    0b010001,0b010011,0b010101,0b011101,0b011001,0b010111,0b011111,0b011011,0b010110,0b011110,
    0b100001,0b100011,0b101110,0b100101,0b101101,0b101001,0
  };

  const float startX = 5.0f;
  const float startY = 5.0f;
  const float cellAdvance = 6.0f; // mm between cells
  const float dotX[2] = {0.0f, 1.6f};
  const float dotY[3] = {0.0f, 2.5f, 5.0f};

  // clear previous job
  jobCount = 0; jobIndex = 0;

  int col = 0;
  for (const char* p = utf8text; *p && jobCount < MAX_JOB_POINTS; ++p) {
    char ch = *p;
    if (ch >= 'A' && ch <= 'Z') ch = ch - 'A' + 'a';
    unsigned char pat = 0;
    if (ch >= 'a' && ch <= 'z') pat = braille_map[ch - 'a'];
    else if (ch == ' ') pat = 0;

    float cellX = startX + col * cellAdvance;
    float cellY = startY;
    // mapping bits to dots: bit0->dot1 (top-left), bit1->dot2 (mid-left), bit2->dot3 (bot-left)
    // bit3->dot4 (top-right), bit4->dot5 (mid-right), bit5->dot6 (bot-right)
    for (int d = 0; d < 6 && jobCount < MAX_JOB_POINTS; ++d) {
      if (pat & (1 << d)) {
        int colIdx = (d >= 3) ? 1 : 0;
        int rowIdx = d % 3;
        jobPoints[jobCount].x = cellX + dotX[colIdx];
        jobPoints[jobCount].y = cellY + dotY[rowIdx];
        jobCount++;
      }
    }
    col++;
  }
  if (jobCount > 0) {
    jobPending = true;
    Serial.printf("scheduleBraillePrint: queued %d points for text '%s'\n", jobCount, utf8text);
    // 打印前 10 个点用于调试
    for (int i = 0; i < jobCount && i < 10; ++i) {
      Serial.printf("  point[%d]=(%.2f,%.2f)\n", i, jobPoints[i].x, jobPoints[i].y);
    }
  }
}

// 将线性 mm 转换为编码器计数
long mmToCounts(float mm) {
  return (long)(mm * COUNTS_PER_MM);
}

// 阻塞式移动单轴（左电机为 X，右电机为 Y）到指定绝对地址（mm，相对于上电时的初始位置）
// pwm 为输出 PWM（0-255 范围内使用 ledc），minpwm 用于克服静摩擦
void moveAxisToAbsoluteMM(bool isLeftAxis, float target_mm, int pwm, int minpwm) {
  volatile long &enc = isLeftAxis ? encL : encR;
  int in1 = isLeftAxis ? AIN1_L : AIN1_R;
  int in2 = isLeftAxis ? AIN2_L : AIN2_R;
  int ch = isLeftAxis ? PWM_CH_L : PWM_CH_R;

  // 使用每轴运行时 countsPerMm（允许运行时校准）
  long targetCounts = (long)(target_mm * (isLeftAxis ? countsPerMmL : countsPerMmR));

  long cur;
  noInterrupts();
  cur = enc;
  interrupts();

  long diff = targetCounts - cur;
  if (diff == 0) {
    Serial.printf("moveAxisToAbsoluteMM: axis=%c target=%.2f counts=%ld cur=%ld diff=0\n", isLeftAxis? 'L':'R', target_mm, targetCounts, cur);
    return;
  }

  bool forward = diff > 0;
  // 如果是右轴并设置了软件反转，则在决定 forward 时取反
  bool appliedInvert = false;
  if (!isLeftAxis && invertRightAxis) {
    forward = !forward;
    appliedInvert = true;
  }
  Serial.printf("moveAxisToAbsoluteMM START axis=%c target=%.2f targetCounts=%ld cur=%ld diff=%ld forward=%d invertApplied=%d\n", isLeftAxis? 'L':'R', target_mm, targetCounts, cur, diff, forward?1:0, appliedInvert?1:0);
  if (forward) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  int outpwm = constrain(pwm, 0, MAX_PWM);
  if (outpwm > 0) outpwm = max(outpwm, minpwm);
  ledcWrite(ch, outpwm);

  // 等待达到目标（带超时保护）
  unsigned long start = millis();
  const unsigned long TIMEOUT_MS = 8000; // 8 秒超时
  const unsigned long STALL_MS = 400; // 失速检测：若编码器长时间不变则判为失速
  long lastEnc = cur;
  unsigned long lastMoveTime = start;
  while (true) {
    noInterrupts();
    long now = enc;
    interrupts();
    if (forward) {
      if (now >= targetCounts) break;
    } else {
      if (now <= targetCounts) break;
    }
    // 失速检测：编码器数值未变化
    if (now != lastEnc) {
      lastEnc = now;
      lastMoveTime = millis();
    }
    if (millis() - lastMoveTime > STALL_MS) {
      ledcWrite(ch, 0);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
      Serial.printf("Stall detected on axis %c target=%ld cur=%ld\n", isLeftAxis? 'L':'R', targetCounts, now);
      return;
    }
    if (millis() - start > TIMEOUT_MS) {
      // 超时：停止电机并打印调试信息
      ledcWrite(ch, 0);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, HIGH);
      Serial.printf("Timeout on axis %c target=%ld cur=%ld\n", isLeftAxis? 'L':'R', targetCounts, now);
      return;
    }
    delay(2);
  }

  // 停止并上锁
  ledcWrite(ch, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
}

// 阻塞式 XY 移动（先 X 后 Y），相对上电初始位置
void moveToXYBlocking(float x_mm, float y_mm, int pwm) {
#if SWAP_XY
  // 交换映射：右电机作为 X，左电机作为 Y
  moveAxisToAbsoluteMM(false, x_mm, pwm, MIN_PWM_R);
  moveAxisToAbsoluteMM(true, y_mm, pwm, MIN_PWM_L);
#else
  // 常规映射：左电机 -> X，右电机 -> Y
  moveAxisToAbsoluteMM(true, x_mm, pwm, MIN_PWM_L);
  moveAxisToAbsoluteMM(false, y_mm, pwm, MIN_PWM_R);
#endif
}

// 触发切削/打点（短脉冲）
void triggerCut() {
  digitalWrite(CUT_PIN, HIGH);
  delay(120);
  digitalWrite(CUT_PIN, LOW);
}

// 打印一个点：移动到 (x,y)、触发、返回 home (0,0)
void printDot(float x_mm, float y_mm, int pwm) {
  Serial.printf("printDot: target=(%.2f,%.2f) pwm=%d\n", x_mm, y_mm, pwm);
  // 记录移动前编码器与位姿
  noInterrupts();
  long beforeL = encL; long beforeR = encR;
  float beforeX = currentX; float beforeY = currentY; float beforeT = currentTheta;
  interrupts();
  Serial.printf("  before encL=%ld encR=%ld pose=(%.2f,%.2f,%.3f)\n", beforeL, beforeR, beforeX, beforeY, beforeT);

  moveToXYBlocking(x_mm, y_mm, pwm);

  noInterrupts();
  long atL = encL; long atR = encR;
  float atX = currentX; float atY = currentY; float atT = currentTheta;
  interrupts();
  Serial.printf("  after move encL=%ld encR=%ld pose=(%.2f,%.2f,%.3f)\n", atL, atR, atX, atY, atT);

  triggerCut();

  // 返回 home 并记录
  moveToXYBlocking(0.0, 0.0, pwm);
  noInterrupts();
  long retL = encL; long retR = encR;
  float retX = currentX; float retY = currentY; float retT = currentTheta;
  interrupts();
  Serial.printf("  returned encL=%ld encR=%ld pose=(%.2f,%.2f,%.3f)\n", retL, retR, retX, retY, retT);
}

// 打印第一排：从 startX 开始，沿 X 方向按 spacing 打 count 个点，完成后返回 home
void printFirstRow(int count, float startX, float spacing, float y, int pwm) {
  for (int i = 0; i < count; ++i) {
    float x = startX + i * spacing;
    // 限制在板子范围内
    if (x < 0) x = 0;
    if (x > PLATE_LEN_MM) x = PLATE_LEN_MM;
    if (y < 0) y = 0;
    if (y > PLATE_WID_MM) y = PLATE_WID_MM;
    moveToXYBlocking(x, y, pwm);
    triggerCut();
    delay(200); // 等待短暂时间保证打点完成
  }
  // 完成后回到初始位置（home）
  moveToXYBlocking(0.0, 0.0, pwm);
}

// 打印一个矩形网格：cols x rows，起点为 startX,startY（mm），间距 spacingX,spacingY（mm）
void printGrid(int cols, int rows, float startX, float startY, float spacingX, float spacingY, int pwm) {
  for (int r = 0; r < rows; ++r) {
    float y = startY + r * spacingY;
    if (y < 0) y = 0;
    if (y > PLATE_WID_MM) y = PLATE_WID_MM;
    for (int c = 0; c < cols; ++c) {
      float x = startX + c * spacingX;
      if (x < 0) x = 0;
      if (x > PLATE_LEN_MM) x = PLATE_LEN_MM;
      moveToXYBlocking(x, y, pwm);
      triggerCut();
      delay(120);
    }
    delay(200);
  }
  // 返回 home
  moveToXYBlocking(0.0, 0.0, pwm);
}

// 切换 / 读取 右轴方向反转开关（供 web server 调用）
void toggleInvertRight() {
  invertRightAxis = !invertRightAxis;
  Serial.printf("invertRightAxis=%d\n", invertRightAxis ? 1 : 0);
}

bool getInvertRight() {
  return invertRightAxis;
}

void setup() {
  // 调用外部初始化程序（来自 initialization.CPP）
  //runInitialization();//初始化程序
  pinMode(AIN1_L, OUTPUT);
  pinMode(AIN2_L, OUTPUT);
  pinMode(AIN1_R, OUTPUT);
  pinMode(AIN2_R, OUTPUT);

  pinMode(HALL_A_L, INPUT_PULLUP);
  pinMode(HALL_B_L, INPUT_PULLUP);
  pinMode(HALL_A_R, INPUT_PULLUP);
  pinMode(HALL_B_R, INPUT_PULLUP);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_L, PWM_CH_L);
  ledcAttachPin(PWM_R, PWM_CH_R);

  attachInterrupt(digitalPinToInterrupt(HALL_A_L), hallISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_A_R), hallISR_R, CHANGE);

  Serial.begin(115200);

  delay(200);
  // 初始化运行时每轴 counts/mm（可由校准修改）
  countsPerMmL = (float)(COUNTS_PER_MM);
  countsPerMmR = (float)(COUNTS_PER_MM);
  // 切削引脚配置
  pinMode(CUT_PIN, OUTPUT);
  digitalWrite(CUT_PIN, LOW);

  // 将当前上电位置视为 home（编码器计数归零）
  noInterrupts();
  encL = 0; encR = 0; prevEncL = 0; prevEncR = 0;
  interrupts();

  Serial.println("XY control start");
  // 启动 WiFi web server（在手机上用 WiFi 连接 SSID: Braille_ESP 密码:12345678）
  setupWiFi();
}
void loop() {
  // 运行第一排打点（可通过宏控制），仅执行一次
#if RUN_PRINT_FIRST_ROW
  static bool firstRowDone = false;
  if (!firstRowDone) {
    Serial.println("Printing first row...");
    printFirstRow(FIRST_ROW_COUNT, FIRST_ROW_START_X, FIRST_ROW_SPACING, FIRST_ROW_Y, FIRST_ROW_PWM);
    Serial.println("First row complete, returned to home.");
    firstRowDone = true;
  }
#endif

#if RUN_PRINT_GRID
  static bool gridDone = false;
  if (!gridDone) {
    Serial.println("Printing grid area...");
    printGrid(GRID_COLS, GRID_ROWS, GRID_START_X, GRID_START_Y, GRID_SPACING_X, GRID_SPACING_Y, GRID_PWM);
    Serial.println("Grid complete, returned to home.");
    gridDone = true;
  }
#endif

  // 处理 Web 请求
  handleServer();
  // 处理回 home 请求（网页按 Home）
  if (goHomePending) {
    noInterrupts();
    goHomePending = false;
    interrupts();
    Serial.println("Executing Go Home (0,0)");
    moveToXYBlocking(0.0, 0.0, 120);
    Serial.println("Returned to home");
  }
  // 处理单轴移动调度（由 web 请求设置）
  if (axisMovePending) {
    noInterrupts();
    bool isLeft = axisMoveIsLeft;
    float pos = axisMovePos;
    int pwm = axisMovePwm;
    axisMovePending = false;
    interrupts();
    Serial.printf("Executing scheduled axis %c -> %.2f mm\n", isLeft? 'L':'R', pos);
    moveAxisToAbsoluteMM(isLeft, pos, pwm, isLeft? MIN_PWM_L : MIN_PWM_R);
    Serial.println("Axis move complete");
  }
  // 处理排队的打印任务（每次循环处理一个点，HTTP 仅负责下发任务）
  if (jobPending) {
    if (jobIndex < jobCount) {
      Serial.printf("Printing job point %d/%d: x=%.2f y=%.2f\n", jobIndex+1, jobCount, jobPoints[jobIndex].x, jobPoints[jobIndex].y);
      // 使用网格打印的 PWM 强度作为默认打点功率
      printDot(jobPoints[jobIndex].x, jobPoints[jobIndex].y, GRID_PWM);
      jobIndex++;
    } else {
      Serial.println("Braille job complete, returned to home.");
      jobPending = false;
      jobCount = 0;
      jobIndex = 0;
    }
  }

  // 空循环，设备待命
  delay(100);
}

// 将当前编码器计数与位姿归零，把当前位置当作 home
void resetHome() {
  noInterrupts();
  encL = 0; encR = 0;
  prevEncL = 0; prevEncR = 0;
  currentX = 0.0f; currentY = 0.0f; currentTheta = 0.0f;
  interrupts();
  Serial.println("Home reset: encoders and pose set to zero");
}

// 设置当前角度（以度为单位），不会改变编码器计数
void setThetaDeg(float deg) {
  float rad = deg * PI / 180.0f;
  noInterrupts();
  currentTheta = rad;
  // 规范化
  while (currentTheta > PI) currentTheta -= 2 * PI;
  while (currentTheta < -PI) currentTheta += 2 * PI;
  interrupts();
  Serial.printf("Theta set to %.2f deg (%.3f rad)\n", deg, currentTheta);
}

void scheduleAxisMove(bool isLeftAxis, float pos_mm, int pwm) {
  noInterrupts();
  axisMoveIsLeft = isLeftAxis;
  axisMovePos = pos_mm;
  axisMovePwm = pwm;
  axisMovePending = true;
  interrupts();
  Serial.printf("Scheduled axis move %c -> %.2f mm pwm=%d\n", isLeftAxis? 'L':'R', pos_mm, pwm);
}

// 立即停止所有电机并上锁
void stopAll() {
  ledcWrite(PWM_CH_L, 0);
  ledcWrite(PWM_CH_R, 0);
  digitalWrite(AIN1_L, HIGH);
  digitalWrite(AIN2_L, HIGH);
  digitalWrite(AIN1_R, HIGH);
  digitalWrite(AIN2_R, HIGH);
  Serial.println("All motors stopped (stopAll)");
}

// 请求回 home（由 web server 调用）
void requestGoHome() {
  noInterrupts();
  goHomePending = true;
  interrupts();
  Serial.println("GoHome requested");
}

// 启动半自动校准：重置 home，记录起始编码器并以指定 PWM 正向驱动轴
void startCalibration(bool isLeft, float mm, int pwm) {
  // 以当前轴为 home（简单起见）
  resetHome();
  noInterrupts();
  calibStartEnc = isLeft ? encL : encR;
  interrupts();
  calibIsLeft = isLeft;
  calibTargetMM = mm;
  calibPwm = pwm;

  int in1 = isLeft ? AIN1_L : AIN1_R;
  int in2 = isLeft ? AIN2_L : AIN2_R;
  int ch = isLeft ? PWM_CH_L : PWM_CH_R;
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  ledcWrite(ch, constrain(pwm, 0, MAX_PWM));
  calibRunning = true;
  Serial.printf("Calibration started axis=%c mm=%.2f pwm=%d\n", isLeft? 'L':'R', mm, pwm);
}

// 停止校准并计算 counts/mm
float stopCalibration() {
  if (!calibRunning) return -1.0f;
  int in1 = calibIsLeft ? AIN1_L : AIN1_R;
  int in2 = calibIsLeft ? AIN2_L : AIN2_R;
  int ch = calibIsLeft ? PWM_CH_L : PWM_CH_R;
  // 停止电机
  ledcWrite(ch, 0);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);

  noInterrupts();
  long nowEnc = calibIsLeft ? encL : encR;
  interrupts();

  long delta = labs(nowEnc - calibStartEnc);
  float measured = 0.0f;
  if (calibTargetMM > 0.0f) measured = (float)delta / calibTargetMM;
  if (calibIsLeft) countsPerMmL = measured; else countsPerMmR = measured;
  calibRunning = false;
  Serial.printf("Calibration stopped axis=%c delta_counts=%ld mm=%.2f counts_per_mm=%.3f\n", calibIsLeft? 'L':'R', delta, calibTargetMM, measured);
  return measured;
}
