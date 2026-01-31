#include <Arduino.h>
#include <ESP32Encoder.h>
#include <QuickPID.h>
#include <ESP32Servo.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include <HUSKYLENS.h>
#pragma GCC diagnostic pop
#include <esp32-hal-ledc.h>

// ===== 編碼器物件 =====
ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

// ===== 伺服馬達物件 =====
Servo arm;  // 手臂伺服馬達
Servo claw; // 爪子伺服馬達

// ===== 伺服馬達腳位定義 =====
#define ARM_PIN 14  // 手臂伺服馬達腳位
#define CLAW_PIN 15 // 爪子伺服馬達腳位

// ===== 伺服馬達角度設定 =====
#define ARM_UP 90       // 手臂升起角度
#define ARM_DOWN 40     // 手臂下降角度
#define CLAW_OPEN 150   // 爪子開啟角度
#define CLAW_CLOSE 45   // 爪子關閉角度
#define CAMERA_FRONT 90 // 攝影機正前方角度
#define CAMERA_LEFT 180 // 攝影機左側角度
#define CAMERA_RIGHT 0  // 攝影機右側角度

// ===== 編碼器腳位定義 =====
#define LEFT_ENCODER_A 18  // 左編碼器 A 相
#define LEFT_ENCODER_B 19  // 左編碼器 B 相
#define RIGHT_ENCODER_A 23 // 右編碼器 A 相
#define RIGHT_ENCODER_B 5  // 右編碼器 B 相

// ===== 腳位定義 =====
// 紅外線感測器腳位
#define IR_LL_PIN 39 // 最左側紅外線
#define IR_L_PIN 32  // 左側紅外線
#define IR_M_PIN 33  // 中間紅外線
#define IR_R_PIN 34  // 右側紅外線
#define IR_RR_PIN 35 // 最右側紅外線

// 馬達控制腳位
#define MOTOR_L_FWD 27 // 左馬達正轉
#define MOTOR_L_BWD 13 // 左馬達反轉
#define MOTOR_R_FWD 2  // 右馬達正轉
#define MOTOR_R_BWD 4  // 右馬達反轉

// PWM 通道 (使用 Timer 2 的通道 8-11，Timer 0 預留給伺服馬達)
#define CH_L_FWD 8  // 左馬達正轉通道 (Timer 2)
#define CH_L_BWD 9  // 左馬達反轉通道 (Timer 2)
#define CH_R_FWD 10 // 右馬達正轉通道 (Timer 2)
#define CH_R_BWD 11 // 右馬達反轉通道 (Timer 2)

// ===== 參數設定 =====
#define IR_THRESHOLD 2000 // 紅外線感測器閾值
#define PWM_FREQ 75000    // PWM 頻率
#define PWM_RES 8         // PWM 解析度 (8-bit = 0~255)

// ===== 函式前向宣告 =====
// 提示：函式宣告格式為 回傳型別 函式名稱(參數);
//       例如：void forward(); 或 int IR_M_read();

// --- 紅外線感測器 ---
int IR_LL_read(); // 讀取最左側紅外線感測器
int IR_L_read();  // 讀取左側紅外線感測器
int IR_M_read();  // 讀取中間紅外線感測器
int IR_R_read();  // 讀取右側紅外線感測器
int IR_RR_read(); // 讀取最右側紅外線感測器

// --- 馬達控制 ---
void motor(int L, int R); // 馬達控制 (L:左輪速度, R:右輪速度, 正值前進/負值後退)
void forward();           // 前進
void backward();          // 後退
void m_Left();            // 左轉 (左輪停止)
void m_Right();           // 右轉 (右輪停止)
void b_Left();            // 急左轉 (左輪反轉)
void b_Right();           // 急右轉 (右輪反轉)
void stop();              // 停止

// --- 伺服馬達控制 ---
void arm_up();       // 手臂升起
void arm_down();     // 手臂下降
void claw_open();    // 爪子開啟
void claw_close();   // 爪子關閉
void camera_front(); // 攝影機正前方
void camera_left();  // 攝影機左側
void camera_right(); // 攝影機右側

void pick_up();
void put_down();

// --- 測試指令 ---
void test_encoder(); // 編碼馬達測試 (顯示編碼器計數值)
void test_servo();   // 伺服馬達測試 (手臂和爪子動作)
void test_motor();   // 馬達測試 (前進、後退、左轉、右轉)
void test_ir();      // 紅外線感測器測試 (顯示感測器狀態)

// --- 循跡功能 ---
void trail(); // 循跡

// Padilla 循跡功能
int Padilla_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms, int lastError);
void Padilla_right(int baseSpeed, int turnSpeedL, int turnSpeedR, float Kp, float Kd, bool useStop);
void Padilla_left(int baseSpeed, int turnSpeedL, int turnSpeedR, float Kp, float Kd, bool useStop);

// ===== 自訂函式區 =====
// TODO: 請在此區塊建立你的自訂函式
//
// 【函式建立格式】
//   回傳型別 函式名稱(參數列表)
//   {
//       函式內容
//   }
//
// 【建議建立的函式】
//
// --- 紅外線感測器 ---

int IR_LL_read()
{
  int sensorValue = analogRead(IR_LL_PIN);
  return (sensorValue > IR_THRESHOLD) ? 1 : 0;
}
int IR_L_read()
{
  int sensorValue = analogRead(IR_L_PIN);
  return (sensorValue > IR_THRESHOLD) ? 1 : 0;
}
int IR_M_read()
{
  int sensorValue = analogRead(IR_M_PIN);
  return (sensorValue > IR_THRESHOLD) ? 1 : 0;
}
int IR_R_read()
{
  int sensorValue = analogRead(IR_R_PIN);
  return (sensorValue > IR_THRESHOLD) ? 1 : 0;
}
int IR_RR_read()
{
  int sensorValue = analogRead(IR_RR_PIN);
  return (sensorValue > IR_THRESHOLD) ? 1 : 0;
}

// --- 馬達控制 ---
//!---------------------左側馬達力量較小(35左右),待調-----------------------

void forward()
{
  motor(250, 215);
}
void backward()
{
  motor(-150, -200);
}
void m_Left()
{
  motor(0, 200);
}
void m_Right()
{
  motor(200, 0);
}
void b_Left()
{
  motor(-110, 50);
}
void b_Right()
{
  motor(60, -90);
}
void big_stop()
{
  motor(-255, -200);
  delay(10);
  motor(0, 0);
  leftEncoder.clearCount();
  rightEncoder.clearCount();
}
void stop()
{
  motor(0, 0);
  leftEncoder.clearCount();
  rightEncoder.clearCount();
}
void motor(int L, int R)
{
  if (L > 0)
  {
    ledcWrite(CH_L_FWD, constrain(abs(L), 0, 255));
    ledcWrite(CH_L_BWD, 0);
  }
  else
  {
    ledcWrite(CH_L_FWD, 0);
    ledcWrite(CH_L_BWD, constrain(abs(L), 0, 255));
  }
  if (R > 0)
  {
    ledcWrite(CH_R_FWD, constrain(abs(R), 0, 255));
    ledcWrite(CH_R_BWD, 0);
  }
  else
  {
    ledcWrite(CH_R_FWD, 0);
    ledcWrite(CH_R_BWD, constrain(abs(R), 0, 255));
  }
}
// --- 動作函式 ---
// 功能：前進、後退、左轉、右轉、停止等
// 提示：呼叫馬達控制函式，帶入適當的左右輪速度
//
// --- 伺服馬達控制 ---
// 功能：手臂升降、爪子開合
// 提示：使用 arm.write(角度) 和 claw.write(角度)
//
// --- 測試函式 ---
// 功能：測試各元件是否正常運作
// 提示：依序執行動作並用 Serial 輸出狀態
void test_motor()
{
  Serial.println("Motor Test Start");
  Serial.println("Forward");
  forward();
  delay(1000);
  Serial.println("stop");
  stop();
  delay(1000);
  Serial.println("Backward");
  backward();
  delay(1000);
  Serial.println("stop");
  stop();
  delay(1000);
  Serial.println("Left");
  m_Left();
  delay(1000);
  Serial.println("stop");
  stop();
  delay(1000);
  Serial.println("Right");
  m_Right();
  delay(1000);
  Serial.println("stop");
  stop();
  Serial.println("b_Left");
  b_Left();
  delay(1000);
  Serial.println("stop");
  stop();
  delay(1000);
  Serial.println("b_Right");
  b_Right();
  delay(1000);
  Serial.println("stop");
  stop();
  Serial.println("Motor Test End");
}

void test_ir()
{
  Serial.println("IR Sensor Test Start");
  while (true)
  {
    Serial.print("LL: ");
    Serial.print(IR_LL_read());
    Serial.print(" L: ");
    Serial.print(IR_L_read());
    Serial.print(" M: ");
    Serial.print(IR_M_read());
    Serial.print(" R: ");
    Serial.print(IR_R_read());
    Serial.print(" RR: ");
    Serial.println(IR_RR_read());
    delay(500);
  }
  Serial.println("IR Sensor Test End");
}

void test_encoder()
{
  long leftCount = leftEncoder.getCount();
  long rightCount = rightEncoder.getCount();

  Serial.print("Left Encoder Count: ");
  Serial.println(leftCount);
  Serial.print("Right Encoder Count: ");
  Serial.println(rightCount);

  delay(100);
}

// --- 循跡功能 ---
void trail()
{
  if (IR_M_read() == 1)
  {
    if (IR_L_read() == 1 && IR_R_read() == 0)
    {
      m_Left(); // 左轉
    }
    else if (IR_L_read() == 0 && IR_R_read() == 1)
    {
      m_Right(); // 右轉
    }
    else
    {
      forward(); // 前進
    }
  }
  else
  {
    if ((IR_L_read() == 1) && (IR_R_read() == 0))
    {
      m_Left(); // 左轉
    }
    else if ((IR_L_read() == 0) && (IR_R_read() == 1))
    {
      m_Right(); // 右轉
    }
  }
}
// todo: Padilla trail
//!----------from aerc2-------------
int Padilla_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms, int lastError)
{
  const int minimumSpeed = -255; // 最小速度
  const int maximumSpeed = 255;  // 最大速度
  int integral = 0;              // 積分項

  unsigned long start_time = millis();

  while (true)
  {
    if (ms > 0 && millis() - start_time >= ms)
    {
      break;
    }
    // 計算偏差值
    int error = 0;

    if (useFiveIR)
    {
      if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = 0;
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -0.4;
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 1 && IR_RR_read() == 0)
      {
        error = 0.4;
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 1 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -1.9;
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 1 && IR_RR_read() == 0)
      {
        error = 1.9;
      }
      else if (IR_LL_read() == 1 && IR_L_read() == 1 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -2.8;
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 1 && IR_RR_read() == 1)
      {
        error = 2.8;
      }
      else if (IR_LL_read() == 1 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -4.4;
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 1)
      {
        error = 4.4;
      }
      else
      {
        error = lastError;
      }
    }
    else
    {
      if (IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 0)
      {
        error = 0;
      }
      else if (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 0)
      {
        error = -0.4;
      }
      else if (IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 1)
      {
        error = 0.4;
      }
      else if (IR_L_read() == 1 && IR_M_read() == 0 && IR_R_read() == 0)
      {
        error = -1.9;
      }
      else if (IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 1)
      {
        error = 1.9;
      }
      else
      {
        error = lastError;
      }
    }

    // 計算積分項
    integral += error;

    // 計算微分項
    int derivative = error - lastError;

    // 計算調整值
    int adjustment = Kp * error + Ki * integral + Kd * derivative;

    // 計算新的馬達速度
    int speedL = baseSpeed + adjustment;
    int speedR = baseSpeed - adjustment;

    // 限制速度在最小和最大速度之間
    speedL = constrain(speedL, minimumSpeed, maximumSpeed);
    speedR = constrain(speedR, minimumSpeed, maximumSpeed);

    // 設置馬達速度
    motor(speedL, speedR);

    // 更新上一次的偏差值
    lastError = error;

    if (ms == 0 && exitCondition())
    {
      break;
    }
  }
  return lastError;
}
void Padilla_right(int baseSpeed, int turnSpeedL, int turnSpeedR, float Kp, float Kd, bool useStop)
{
  Padilla_trail(false, []()
                { return (IR_RR_read() == 1); }, Kp, Kd, 0, baseSpeed, 0, 0);
  while (IR_RR_read() == 1) // RR == 1 時直走（直到RR變0）
  {
    motor(baseSpeed, baseSpeed);
  }
  if (useStop)
  {
    stop();
  }
  while (IR_RR_read() == 0) // RR == 0 時轉彎（直到RR變1）
  {
    motor(turnSpeedL, turnSpeedR);
  }
  while (IR_RR_read() == 1) // RR == 1 時繼續轉（直到RR變0）
  {
    motor(turnSpeedL, turnSpeedR);
  }
}

void Padilla_left(int baseSpeed, int turnSpeedL, int turnSpeedR, float Kp, float Kd, bool useStop)
{
  Padilla_trail(false, []()
                { return (IR_LL_read() == 1); }, Kp, Kd, 0, baseSpeed, 0, 0);
  while (IR_LL_read() == 1) // LL == 1 時直走（直到LL變0）
  {
    motor(baseSpeed, baseSpeed);
  }
  if (useStop)
  {
    stop();
  }
  while (IR_LL_read() == 0) // LL == 0 時轉彎（直到LL變1）
  {
    motor(turnSpeedL, turnSpeedR);
  }
  while (IR_LL_read() == 1) // LL == 1 時繼續轉（直到LL變0）
  {
    motor(turnSpeedL, turnSpeedR);
  }
}
//!---------from aerc2-------------
// --- 伺服馬達控制 ---
void arm_up()
{
  arm.write(ARM_UP);
}

void arm_down()
{
  arm.write(ARM_DOWN);
}

void claw_open()
{
  claw.write(CLAW_OPEN);
}

void claw_close()
{
  claw.write(CLAW_CLOSE);
}

void pick_up()
{
  claw_close();
  delay(200);
  arm_up();
  delay(200);
}

void put_down()
{
  arm_down();
  delay(200);
  claw_open();
  delay(200);
}

// ===== 主程式 =====
void setup()
{
  Serial.begin(9600);

  // --- 伺服馬達定時器分配 (Timer 0 給伺服馬達) ---
  ESP32PWM::allocateTimer(0);

  // --- 伺服馬達初始化 ---
  arm.setPeriodHertz(50);           // 標準 50Hz 伺服馬達
  arm.attach(ARM_PIN, 500, 2400);   // SG90 脈寬範圍 500~2400us
  claw.setPeriodHertz(50);          // 標準 50Hz 伺服馬達
  claw.attach(CLAW_PIN, 500, 2400); // SG90 脈寬範圍 500~2400us

  // --- 編碼器初始化 ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  leftEncoder.attachHalfQuad(LEFT_ENCODER_A, LEFT_ENCODER_B);
  leftEncoder.clearCount(); //! 清除左輪計數值
  rightEncoder.attachHalfQuad(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
  rightEncoder.clearCount();

  // --- 紅外線感測器初始化 ---
  pinMode(IR_LL_PIN, INPUT);
  pinMode(IR_L_PIN, INPUT);
  pinMode(IR_M_PIN, INPUT);
  pinMode(IR_R_PIN, INPUT);
  pinMode(IR_RR_PIN, INPUT);

  // --- 馬達 PWM 初始化 (Timer 2，Timer 0 預留給伺服馬達) ---
  pinMode(MOTOR_L_BWD, OUTPUT);
  ledcSetup(CH_L_BWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_BWD, CH_L_BWD);

  pinMode(MOTOR_L_FWD, OUTPUT);
  ledcSetup(CH_L_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_FWD, CH_L_FWD);

  pinMode(MOTOR_R_BWD, OUTPUT);
  ledcSetup(CH_R_BWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_R_BWD, CH_R_BWD);

  pinMode(MOTOR_R_FWD, OUTPUT);
  ledcSetup(CH_R_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_R_FWD, CH_R_FWD);

  // TODO: 初始化完成後，可呼叫停止函式確保馬達不會亂轉

  //======================================================================程式開始==============================================================
  claw_open();
  delay(200);
  arm_down();
  delay(200);

  int error = 0;
  forward();
  delay(200);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1 && IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 100, 0, 250, 0, error);
  big_stop();
  delay(100);
  Padilla_trail(true, []()
                { return (IR_M_read() == 1 && IR_R_read() == 1 && IR_L_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 30, 0, 0, 50, 0, error);
  forward();
  delay(10);
  big_stop();
  delay(100);
  b_Right();
  delay(180);
  while (!(IR_RR_read() == 1))
  {
    b_Right();
  }
  while (!(IR_RR_read() == 0))
  {
    b_Right();
  }
  b_Left();
  delay(100);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_R_read() == 1); }, 30, 0, 0, 50, 0, 0);
  big_stop();
  pick_up();

  // //! 抵達右側
  b_Right();
  delay(200);
  while (!(IR_RR_read() == 1))
  {
    b_Right();
  }
  b_Left();
  delay(100);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1); }, 70, 100, 0, 100, 0, 0);
  forward();
  delay(50);
  b_Left();
  delay(200);
  while (!(IR_LL_read() == 1))
  {
    b_Left();
  }
  while (!(IR_LL_read() == 0))
  {
    b_Left();
  }
  b_Right();
  delay(100);
  error = Padilla_trail(false, []()
                        { return (IR_M_read() == 1 && IR_R_read() == 1 && IR_L_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 30, 0, 0, 50, 0, 0);
  error = Padilla_trail(false, []()
                        { return (false); }, 30, 0, 0, 250, 200, error);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 100, 0, 100, 0, error);
  big_stop();
  put_down();

  // //! ====== 中側程式 ======
  backward();
  delay(100);
  b_Left();
  delay(300);
    while (!(IR_LL_read() == 1))
  {
    b_Left();
  }
  
  b_Right();
  delay(200);
  stop();
  delay(50);
  error = 0;
    Padilla_trail(false, []()
                { return (IR_L_read() == 1 &&IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 30, 0, 0, 50, 0, 0);
  forward();
  delay(100);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 90, 0, 250, 0, error);

  forward();
  delay(50);
    Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 50, 20, 0, 50, 0, 0);
  big_stop();
  pick_up();
  b_Right();
  delay(200);
  while (!(IR_RR_read() == 1))
  {
    b_Right();
  }
  b_Left();
  delay(100);
  // //! 抵達中側
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 100, 0, 100, 0, error);
  forward();
  delay(50);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 100, 0, 250, 0, error);
  forward();
  delay(50);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 100, 0, 50, 0, error);
  backward();
  delay(100);
  big_stop();
  put_down();
  


  // //! ====== 左側程式 ======
  arm_up();
  b_Left();
  delay(300);
    while (!(IR_LL_read() == 1))
  {
    b_Left();
  }
  
  b_Right();
  delay(300);
  stop();
  delay(50);
  arm_down();
  error = 0;
    Padilla_trail(false, []()
                { return (IR_L_read() == 1 &&IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 80, 50, 0, 60, 0, 0);
  forward();
  delay(100);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1); }, 70, 100, 0, 100, 0, error);
  // forward();
  // delay(50);
  // Padilla_trail(false, []()
  //               { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1); }, 70, 100, 0, 100, 0, error);


  big_stop();
  delay(100);
  b_Left();
  delay(200);
  while (!(IR_LL_read() == 1))
  {
    b_Left();
  }
  while (!(IR_LL_read() == 0))
  {
    b_Left();
  }
  b_Right();
  delay(100);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_R_read() == 1); }, 30, 0, 0, 50, 0, 0);
  big_stop();
  pick_up();





  b_Right();
  delay(200);
  while (!(IR_RR_read() == 1))
  {
    b_Right();
  }
  b_Left();
  delay(100);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 70, 100, 0, 100, 0, 0);
  forward();
  delay(50);
  b_Right();
  delay(200);
  while (!(IR_RR_read() == 1))
  {
    b_Right();
  }
  while (!(IR_RR_read() == 0))
  {
    b_Right();
  }
  b_Left();
  delay(100);
  error = Padilla_trail(false, []()
                        { return (IR_M_read() == 1 && IR_R_read() == 1 && IR_L_read() == 1&& IR_RR_read() == 1 && IR_LL_read() == 1); }, 30, 0, 0, 50, 0, 0);
  error = Padilla_trail(false, []()
                        { return (false); }, 30, 0, 0, 50, 200, error);
  // Padilla_trail(false, []()
  //               { return (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 1); }, 70, 100, 0, 100, 0, error);
  big_stop();
  claw_open();
  //--------------------------------------------------------------
  stop();
}

void loop()
{
  // TODO: 在此撰寫主程式邏輯
}
