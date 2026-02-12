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

//===== 鏡頭斯服馬達物件 ======
Servo camera; // 攝影機伺服馬達

// ===== HUSKYLENS 物件 =====
HUSKYLENS huskylens;

// ===== 伺服馬達腳位定義 =====
#define ARM_PIN 14    // 手臂伺服馬達腳位
#define CLAW_PIN 15   // 爪子伺服馬達腳位
#define CAMERA_PIN 25 // 攝影機伺服馬達腳位

// ===== 伺服馬達角度設定 =====
#define ARM_UP 120         // 手臂升起角度
#define ARM_DOWN 40        // 手臂下降角度（取貨用）
#define ARM_DOWN_UNLOAD 50 // 手臂下降角度（卸貨用，可獨立調整）
#define CLAW_OPEN 150      // 爪子開啟角度
#define CLAW_CLOSE 45      // 爪子關閉角度
#define CAMERA_FRONT 90    // 攝影機正前方角度
#define CAMERA_LEFT 180    // 攝影機左側角度
#define CAMERA_RIGHT 0     // 攝影機右側角度

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
#define CH_L_FWD 4 // 左馬達正轉通道 (Timer 2)
#define CH_L_BWD 5 // 左馬達反轉通道 (Timer 2)
#define CH_R_FWD 6 // 右馬達正轉通道 (Timer 2)
#define CH_R_BWD 7 // 右馬達反轉通道 (Timer 2)

// ===== 參數設定 =====
#define IR_THRESHOLD 2000 // 紅外線感測器閾值
#define PWM_FREQ 75000    // PWM 頻率
#define PWM_RES 8         // PWM 解析度 (8-bit = 0~255)

// ===== 速度閉環控制參數 =====
// 速度控制週期（毫秒）：根據 test_max_speed() 測得的極限速度調整
// 建議：每週期計數變化 ≥ 5 較穩定
// 例如：極限 45 c/100ms → 用 20ms（約 9c）；極限 20 c/100ms → 用 50ms（約 10c）
#define SPEED_CONTROL_PERIOD 3 // 速度控制週期，單位 ms

//? 調參指引：SPEED_KP （速度閉環比例係數）
// - 作用：PWM 調整量 = (目標速度 - 實際速度) * Kp
// - 太大（例：5.0）→ 車子一頓一頓、抖動 → 往下調
// - 太小（例：0.01）→ 反應慢、達不到目標速度 → 往上調
// - 建議從 0.1 開始，逐步微調至平順
#define SPEED_KP 0.8 // 速度控制比例係數

// ===== 感測器與車體物理參數 (mm) =====
// 感測器中心到 IR_M 的距離 (左負右正)
#define SENSOR_LL_POS -43.0f // LL 距中心 (28+15)mm
#define SENSOR_L_POS -15.0f  // L 距中心 15mm
#define SENSOR_R_POS 15.0f   // R 距中心 15mm
#define SENSOR_RR_POS 43.0f  // RR 距中心 (15+28)mm
#define SENSOR_WIDTH 3.6f    // 感測器窗口寬度
#define LINE_WIDTH 18.0f     // 黑線寬度
// 車體參數
#define WHEEL_DIAMETER 64.5f   // 車輪直徑
#define WHEEL_TRACK 91.1f      // 車輪輪距 (左右輪中心距)
#define SENSOR_TO_AXLE 108.72f // IR_M 到輪軸中心距離

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
void motor(int L, int R);                                                               // 馬達控制 (L:左輪速度, R:右輪速度, 正值前進/負值後退)
void forward();                                                                         // 前進
void backward();                                                                        // 後退
void m_Left();                                                                          // 左轉 (左輪停止)
void m_Right();                                                                         // 右轉 (右輪停止)
void b_Left();                                                                          // 急左轉 (左輪反轉)
void b_Right();                                                                         // 急右轉 (右輪反轉)
void stop();                                                                            // 停止
void turn_turn(int direction = 1, int delayTime = 450, unsigned long confirmMs = 1500); // 迴轉 (direction: 0=左, 1=右)
void p_fw_v2(int distance);                                                             // 很正的前進（距離控制）
void speed_control(float L_target, float R_target);                                     // 調節速度
void p_right(int degree);                                                               // 右轉（控制度數）
void p_left(int degree);                                                                // 左轉（控制度數）

// --- 伺服馬達控制 ---
void arm_up();          // 手臂升起
void arm_down();        // 手臂下降（取貨用）
void arm_down_unload(); // 手臂下降（卸貨用）
void claw_open();       // 爪子開啟
void claw_close();      // 爪子關閉
void camera_front();    // 攝影機正前方
void camera_left();     // 攝影機左側
void camera_right();    // 攝影機右側

void pick_up();
void put_down();

// --- 測試指令 ---
void test_encoder(); // 編碼馬達測試 (顯示編碼器計數值)
void test_servo();   // 伺服馬達測試 (手臂和爪子動作)
void test_motor();   // 馬達測試 (前進、後退、左轉、右轉)
void test_ir();      // 紅外線感測器測試 (顯示感測器狀態)
void test_forward(); // 前進測試 (測距離與編碼器計數關係)

// --- 循跡功能 ---
void trail(); // 循跡

// Padilla 循跡功能
float Padilla_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms, float lastError);
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
// // !---------------------左側馬達力量較小(35左右),待調-----------------------
//?=-------------------------馬達特殊------------------------------
//!--統一註解代表右邊
// TODO:因特殊情況，左馬達統一放慢20

void forward()
{
  motor(230, 250); // 馬達速度215->250
}
void backward()
{
  motor(-130, -150); // 馬達速度-120->-150
}
void m_Left()
{
  motor(-20, 200);
}
void m_Right()
{
  motor(200, 20);
}
void b_Left()
{
  motor(-110, 60);
}
void b_Right()
{
  motor(60, -110); // 馬達速度-100->-110
}
void big_stop()
{
  motor(-255, -255); // 馬達速度-200->-255
  delay(10);
  motor(0, 0);
}
void stop()
{
  motor(0, 0);
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
void test_moving()
{
  p_fw_v2(10000);
  delay(500);
  p_right(90);
  delay(500);
  p_left(180);
  delay(500);
  p_right(90);
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
float Padilla_trail(bool useFiveIR, bool (*exitCondition)(), float Kp, float Kd, float Ki, int baseSpeed, unsigned long ms, float lastError)
{
  const int minimumSpeed = -255; // 最小速度
  const int maximumSpeed = 255;  // 最大速度
  float integral = 0.0f;         // 積分項

  unsigned long start_time = millis();

  while (true)
  {
    if (ms > 0 && millis() - start_time >= ms)
    {
      break;
    }

    // 計算偏差值
    float error = 0.0f;

    if (useFiveIR)
    {
      // 誤差值基於感測器物理間距等比計算 (LL:-43, L:-15, M:0, R:+15, RR:+43 mm)
      if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = 0; // 正中，偏移 0mm
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -0.75; // 略偏左，偏移約 7.5mm
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 1 && IR_RR_read() == 0)
      {
        error = 0.75; // 略偏右，偏移約 7.5mm
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 1 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -1.83; // 偏左，偏移約 18.3mm
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 1 && IR_RR_read() == 0)
      {
        error = 1.83; // 偏右，偏移約 18.3mm
      }
      else if (IR_LL_read() == 1 && IR_L_read() == 1 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -2.9; // 大偏左，偏移約 29mm
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 1 && IR_RR_read() == 1)
      {
        error = 2.9; // 大偏右，偏移約 29mm
      }
      else if (IR_LL_read() == 1 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 0)
      {
        error = -4.3; // 極偏左，偏移約 43mm
      }
      else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 0 && IR_RR_read() == 1)
      {
        error = 4.3; // 極偏右，偏移約 43mm
      }
      else
      {
        error = lastError;
      }
    }
    else
    {
      // 誤差值基於感測器物理間距等比計算 (L:-15, M:0, R:+15 mm)
      if (IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 0)
      {
        error = 0; // 正中
      }
      else if (IR_L_read() == 1 && IR_M_read() == 1 && IR_R_read() == 0)
      {
        error = -0.75; // 略偏左，偏移約 7.5mm
      }
      else if (IR_L_read() == 0 && IR_M_read() == 1 && IR_R_read() == 1)
      {
        error = 0.75; // 略偏右，偏移約 7.5mm
      }
      else if (IR_L_read() == 1 && IR_M_read() == 0 && IR_R_read() == 0)
      {
        error = -1.83; // 偏左，偏移約 18.3mm
      }
      else if (IR_L_read() == 0 && IR_M_read() == 0 && IR_R_read() == 1)
      {
        error = 1.83; // 偏右，偏移約 18.3mm
      }
      else
      {
        error = lastError;
      }
    }

    // 計算積分項
    integral += error;

    // 計算微分項
    float derivative = error - lastError;

    // 計算調整值
    float adjustment = Kp * error + Ki * integral + Kd * derivative;

    // 計算新的馬達速度
    int speedL = (int)(baseSpeed + adjustment);
    int speedR = (int)(baseSpeed - adjustment);

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

// ===== PID 原地旋轉對準函式 =====
// 功能：在原地旋轉，使用PID控制使車頭對準黑線中心
// 參數說明：
//   baseRotateSpeed: 基礎旋轉速度 (0~255)
//   Kp, Kd, Ki: PID參數
//   direction: 旋轉方向 (1=順時針右轉, -1=逆時針左轉)
//   ms: 旋轉時間限制，0表示靠exitCondition判斷
//   lastError: 上一次的偏差值
//   exitCondition: 退出條件函式指標
// 回傳值：最後一次的偏差值

float PID_spin(int baseRotateSpeed, float Kp, float Kd, float Ki, int direction,
               unsigned long ms, float lastError, bool (*exitCondition)())
{
  const int minimumSpeed = -255; // 最小速度
  const int maximumSpeed = 255;  // 最大速度
  float integral = 0.0f;         // 積分項

  unsigned long start_time = millis();

  while (true)
  {
    // 時間限制判斷
    if (ms > 0 && millis() - start_time >= ms)
    {
      break;
    }

    if (exitCondition && exitCondition())
    {
      if (ms == 0)
      {
        break;
      }
      stop();
      integral = 0.0f;
      lastError = 0.0f;
      continue;
    }

    // 計算偏差值（基於五個紅外線感測器的組合）
    float error = 0.0f;

    // 誤差值基於感測器物理間距等比計算 (LL:-43, L:-15, M:0, R:+15, RR:+43 mm)
    // 完美中心對齊 - 只有中間感測器看到黑線
    if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 1 &&
        IR_R_read() == 0 && IR_RR_read() == 0)
    {
      error = 0; // 正中，偏移 0mm
    }
    // 略微左偏
    else if (IR_LL_read() == 0 && IR_L_read() == 1 && IR_M_read() == 1 &&
             IR_R_read() == 0 && IR_RR_read() == 0)
    {
      error = -0.75; // 偏移約 7.5mm
    }
    // 略微右偏
    else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 1 &&
             IR_R_read() == 1 && IR_RR_read() == 0)
    {
      error = 0.75; // 偏移約 7.5mm
    }
    // 左側較強
    else if (IR_LL_read() == 0 && IR_L_read() == 1 && IR_M_read() == 0 &&
             IR_R_read() == 0 && IR_RR_read() == 0)
    {
      error = -1.83; // 偏移約 18.3mm
    }
    // 右側較強
    else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 &&
             IR_R_read() == 1 && IR_RR_read() == 0)
    {
      error = 1.83; // 偏移約 18.3mm
    }
    // 最左邊
    else if (IR_LL_read() == 1 && IR_L_read() == 1 && IR_M_read() == 0 &&
             IR_R_read() == 0 && IR_RR_read() == 0)
    {
      error = -2.9; // 偏移約 29mm
    }
    // 最右邊
    else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 &&
             IR_R_read() == 1 && IR_RR_read() == 1)
    {
      error = 2.9; // 偏移約 29mm
    }
    // 極左邊
    else if (IR_LL_read() == 1 && IR_L_read() == 0 && IR_M_read() == 0 &&
             IR_R_read() == 0 && IR_RR_read() == 0)
    {
      error = -4.3; // 偏移約 43mm
    }
    // 極右邊
    else if (IR_LL_read() == 0 && IR_L_read() == 0 && IR_M_read() == 0 &&
             IR_R_read() == 0 && IR_RR_read() == 1)
    {
      error = 4.3; // 偏移約 43mm
    }
    else
    {
      error = lastError; // 無法判斷時保持上一次的偏差
    }

    // 計算積分項
    integral += error;

    // 計算微分項
    float derivative = error - lastError;

    // 計算PID調整值
    float adjustment = Kp * error + Ki * integral + Kd * derivative;

    // === 原地旋轉邏輯 ===
    // direction = 1: 順時針右轉
    //   左輪正速度 (前進)，右輪負速度 (後退)
    // direction = -1: 逆時針左轉
    //   左輪負速度 (後退)，右輪正速度 (前進)
    // 當error > 0（線在右邊）：調整使左輪加速、右輪減速 → 向右修正
    // 當error < 0（線在左邊）：調整使左輪減速、右輪加速 → 向左修正

    int spinSpeedL = (int)(baseRotateSpeed * direction + adjustment);
    int spinSpeedR = (int)(-baseRotateSpeed * direction - adjustment);

    // 限制速度在允許範圍內
    spinSpeedL = constrain(spinSpeedL, minimumSpeed, maximumSpeed);
    spinSpeedR = constrain(spinSpeedR, minimumSpeed, maximumSpeed);

    // 執行原地旋轉
    motor(spinSpeedL, spinSpeedR);

    // 更新上一次的偏差值
    lastError = error;

    // 檢測退出條件
  }

  stop(); // 旋轉完成後停止馬達
  return lastError;
}

// ===== 簡化版原地旋轉對準 =====
// 功能：在原地旋轉或微調，直到車頭對準黑線中心並驗證IR_M持續看到500毫秒
// 參數說明：
//   baseRotateSpeed: 旋轉速度 (0~255)
//   Kp, Kd: PID參數
//   mode: 旋轉模式
//        0 = 左轉模式
//        1 = 右轉模式
//        2 = 單純對準模式（只做微調，不做整體旋轉）
// 說明：所有模式完成後，都會驗證IR_M持續看到線500毫秒才視為完成
// 使用示例：
//   PID_spin_to_center(100, 70, 50, 0, 500);        // 左轉對齊+驗證500ms
//   PID_spin_to_center(100, 70, 50, 1, 500);        // 右轉對齊+驗證500ms
//   PID_spin_to_center(50, 70, 50, 2, 500);         // 單純對準+驗證500ms
void PID_spin_to_center(int baseRotateSpeed, float Kp, float Kd, int mode, unsigned long confirmMs)
{
  // 根據mode選擇旋轉方向
  int direction = 0;

  if (mode == 0) // 左轉模式
    direction = -1;
  else if (mode == 1) // 右轉模式
    direction = 1;
  else if (mode == 2) // 單純對準模式（只做微調，baseRotateSpeed設為0）
  {
    direction = 0;
    baseRotateSpeed = 0; // 單純對準時不做整體旋轉
  }

  PID_spin(baseRotateSpeed, Kp, Kd, 0, direction, confirmMs, 0, []()
           { return (IR_M_read() == 1 && IR_L_read() == 0 && IR_R_read() == 0); });
}

void turn_turn(int direction, int delayTime, unsigned long confirmMs)
{
  if (direction == 0) // 左轉
    b_Left();
  else // 右轉
    b_Right();
  delay(delayTime);
  // direction 0=左轉 對應 PID mode 0=左轉對齊
  // direction 1=右轉 對應 PID mode 1=右轉對齊
  PID_spin_to_center(30, 18, 0, direction, confirmMs);
}

// ============ 速度閉環控制函式 ============
// 原理：不再設定固定 PWM，而是設定「目標速度」
//       程式會根據實際速度動態調整 PWM
//       這樣不管電池電量如何，都能維持相同的實際速度

// 全域變數：儲存當前左右輪的 PWM 值（供閉環控制累加調整）
float L_pwm = 0; // 左輪當前 PWM（0~255）
float R_pwm = 0; // 右輪當前 PWM（0~255）

void speed_control(float L_target, float R_target)
{
  //* 速度閉環控制：根據目標速度動態調整 PWM
  // ===== 速度閉環控制（單次呼叫）=====
  // 輸入：L_target = 左輪目標速度（c/週期）
  //       R_target = 右輪目標速度（c/週期）
  // 原理：比較「目標速度」與「實際速度」的差異（誤差）
  //       根據誤差調整 PWM：
  //       - 實際速度 < 目標 → 加大 PWM
  //       - 實際速度 > 目標 → 減小 PWM

  // --- 記錄起始計數 ---
  long L_start = leftEncoder.getCount();
  long R_start = rightEncoder.getCount();

  // --- 等待一個控制週期 ---
  delay(SPEED_CONTROL_PERIOD);

  // --- 計算實際速度（這段時間內的計數變化量）---
  long L_actual = leftEncoder.getCount() - L_start;  // 左輪實際速度
  long R_actual = rightEncoder.getCount() - R_start; // 右輪實際速度

  // --- 計算誤差（目標 - 實際）---
  // 正誤差 = 跑太慢，需要加速
  // 負誤差 = 跑太快，需要減速
  float L_error = L_target - L_actual;
  float R_error = R_target - R_actual;

  // --- 根據誤差調整 PWM（比例控制）---
  // PWM 調整量 = 誤差 × 比例係數（Kp）
  // Kp 越大，調整越激進；Kp 越小，調整越平緩
  L_pwm = L_pwm + L_error * SPEED_KP;
  R_pwm = R_pwm + R_error * SPEED_KP;

  // --- 限制 PWM 範圍（0~255）---
  L_pwm = constrain(L_pwm, -255, 255);
  R_pwm = constrain(R_pwm, -255, 255);

  // --- 輸出到馬達 ---
  motor((int)L_pwm, (int)R_pwm);
}

void p_fw_v2(int distance)
{
  //* 新版前進函式：速度閉環控制
  // ===== 新版前進函式（速度閉環 + 同步修正）=====
  // 輸入：distance = 目標距離（編碼器計數值，約 0.5mm/count）
  // 特點：
  //   1. 速度閉環：根據目標速度動態調整 PWM，不受電量影響
  //   2. 左右同步：即時修正左右輪差異，保持直線
  //
  // ===== 校正流程（請依序進行）=====
  //
  // 【步驟 1】測極限速度 → 決定 BASE_SPEED
  //    - 呼叫 test_max_speed()，記錄左右輪 c/20ms
  //    - 以較慢的輪子為基準，乘 70~80% 作為 BASE_SPEED
  //    - 例：左輪 85、右輪 88 → BASE_SPEED = 85 * 0.7 ≈ 60
  //
  // 【步驟 2】關閉 SYNC_KP → 單獨調 SPEED_KP
  //    - 先把 SYNC_KP 設為 0（排除左右同步的干擾）
  //    - 觀察車子運動是否平順（不抖、不頓）
  //    - 若一頓一頓 → SPEED_KP 太大，往下調（例：5.0 → 0.5 → 0.1）
  //    - 若反應太慢 → SPEED_KP 太小，往上調
  //    - 目標：平順加速、穩定巡航
  //
  // 【步驟 3】調 MIN_SPEED（從低往高調）
  //    - 觀察減速階段是否「停了又動」（速度降太低，馬達停轉再啟動）
  //    - 若有此現象 → MIN_SPEED 太低，往上調（例：10 → 20 → 30）
  //    - 目標：減速過程平滑連續，不會中途停頓
  //
  // 【步驟 4】調 TOLERANCE（補償慣性超距）
  //    - 讓車跑完後，讀取編碼器計數，看超過目標多少
  //    - 若超距 50 → TOLERANCE 設 50（提早停止補償慣性）
  //    - 可同時調 DECEL_START：提早減速 = 減少超距
  //
  // 【步驟 5】開啟 SYNC_KP → 調到走直線不晃(尚未完成)
  //    - 確認步驟 2-4 完成後，將 SYNC_KP 設為小值（例：0.1）
  //    - 若走歪 → 加大 SYNC_KP
  //    - 若左右晃動 → SYNC_KP 太大，調小
  //    - 目標：直線行駛，不偏移也不晃
  //
  // ===== 參數說明 =====

  // ===== 測試結果 =====
  // ===== 測量結果 =====
  // 左輪: 116 c/100ms | 右輪: 105 c/100ms
  // 建議 c/20ms: 左 23 / 右 21

  // --- 參數設定（根據上述流程校正後的值）---
  // 實測極限：左輪 85 c/20ms、右輪 88 c/20ms
  // 以較慢的左輪為基準，設定 70%（保守）
  //? 調參：根據 test_max_speed() 結果調整，取較慢輪子的 70%
  const float BASE_SPEED = 9; // 基礎目標速度（c/週期），約 70% 極限
  // const float SYNC_KP = 0.1;  // 【步驟 5】左右同步修正係數（目前關閉）

  // --- 清除編碼器 ---
  leftEncoder.clearCount();
  rightEncoder.clearCount();

  // --- 重置 PWM 累積值 ---
  L_pwm = 50; // 給一個初始 PWM，加速啟動
  R_pwm = 50;

  // --- 主控制迴圈 ---
  while (true)
  {
    // 讀取當前計數
    long L_count = leftEncoder.getCount();
    long R_count = rightEncoder.getCount();
    long avgCount = (L_count + R_count) / 2; // 平均計數（代表行進距離）

    // === 終止條件：到達目標 ===
    if (avgCount >= distance)
    {
      break;
    }

    // === 呼叫速度閉環控制（左右輪目標速度相同）===
    speed_control(BASE_SPEED, BASE_SPEED);

    // 【步驟 5】若要開啟左右同步修正，取消以下註解：
    // long diffError = L_count - R_count;  // 左輪 - 右輪
    // float correction = diffError * SYNC_KP;
    // speed_control(BASE_SPEED - correction, BASE_SPEED + correction);
  }

  // --- 停止 ---
  stop();
  L_pwm = 0;
  R_pwm = 0;
}

void p_right(int degree)
{
  // 目標距離（編碼器計數值）
  int distance = degree * (960 / 180);
  long targetCount = distance;
  // p_left_dis = distance;
  // 清除編碼器計數器
  leftEncoder.clearCount();
  rightEncoder.clearCount();

  // 階段 1：快速右轉到接近目標
  const int DECEL_COUNT = 1000; // 開始減速的計數值，方便調整
  b_Right();

  while (true)
  {
    long leftCount = leftEncoder.getCount();
    long rightCount = abs(rightEncoder.getCount());
    // 當計數接近目標時停止快速階段
    if ((leftCount >= targetCount - DECEL_COUNT) || (rightCount >= targetCount - DECEL_COUNT))
    {
      break;
    }
    delay(1);
  }

  // 階段 2：低速精調
  motor(35, -35); // 低速右轉
  delay(60);
  stop();

  // 階段 3：反復調整至誤差範圍內
  const int TOLERANCE = 10;       // 容差範圍（±10 計數）
  const int L_MIN_SPEED = 30;     // 最小驅動速度
  const int R_MIN_SPEED = -50;    // 最小驅動速度
  unsigned long maxAttempts = 25; // 最多調整 25 次
  unsigned long attempts = 0;

  while (attempts < maxAttempts)
  {
    long leftCount = leftEncoder.getCount();
    long rightCount = abs(rightEncoder.getCount());
    long L_error = leftCount - targetCount; // 計算誤差
    long R_error = rightCount - targetCount;

    // 誤差在容差範圍內，完成
    if ((abs(L_error) < TOLERANCE) && (abs(R_error) < TOLERANCE))
    {
      break;
    }

    // 判斷各輪是否超過或未達目標
    bool L_over = L_error > TOLERANCE;   // 左輪超過
    bool L_under = L_error < -TOLERANCE; // 左輪未達
    bool R_over = R_error > TOLERANCE;   // 右輪超過
    bool R_under = R_error < -TOLERANCE; // 右輪未達

    // 動態計算調整速度
    int L_adjustSpeed = map(abs(L_error), TOLERANCE, 100, L_MIN_SPEED, 50);
    L_adjustSpeed = constrain(L_adjustSpeed, L_MIN_SPEED, 50);
    int R_adjustSpeed = map(abs(R_error), TOLERANCE, 100, R_MIN_SPEED, -50);
    R_adjustSpeed = constrain(R_adjustSpeed, R_MIN_SPEED, -50);

    // 根據各輪狀態同時調整
    int L_speed = 0;
    int R_speed = 0;

    // 左輪修正
    if (L_over)
      L_speed = -L_adjustSpeed; // 超過 → 反轉
    else if (L_under)
      L_speed = L_adjustSpeed; // 未達 → 正轉

    // 右輪修正
    if (R_over)
      R_speed = -R_adjustSpeed; // 超過 → 反轉
    else if (R_under)
      R_speed = R_adjustSpeed; // 未達 → 正轉

    motor(L_speed, R_speed);
    delay(30);
    stop();
    delay(10);
    attempts++;
  }
  Serial.print("Right Turn Completed. Final Counts - Left: ");
  Serial.print(leftEncoder.getCount());
  Serial.print(", Right: ");
  Serial.println(abs(rightEncoder.getCount()));
}

void p_left(int degree)
{
  // 目標距離（編碼器計數值）
  int distance = degree * (930 / 180);
  long targetCount = distance;
  // p_left_dis = distance;
  // 清除編碼器計數器
  leftEncoder.clearCount();
  rightEncoder.clearCount();

  //* 階段 1：快速前進到接近目標
  const int DECEL_COUNT = 1000; // 開始減速的計數值，方便調整
  b_Left();

  while (true)
  {
    long leftCount = abs(leftEncoder.getCount());
    long rightCount = rightEncoder.getCount();
    // 當計數接近目標時停止快速階段
    if ((leftCount >= targetCount - DECEL_COUNT) || (rightCount >= targetCount - DECEL_COUNT))
    {
      break;
    }
    delay(1);
  }

  //* 階段 2：低速精調
  motor(-35, 35); // 低速前進
  delay(60);
  stop();

  //* 階段 3：反復調整至誤差範圍內
  const int TOLERANCE = 10;       // 容差範圍（±10 計數）
  const int L_MIN_SPEED = -50;    // 最小驅動速度
  const int R_MIN_SPEED = 30;     // 最小驅動速度
  unsigned long maxAttempts = 25; // 最多調整 25 次
  unsigned long attempts = 0;

  while (attempts < maxAttempts)
  {
    long leftCount = abs(leftEncoder.getCount());
    long rightCount = rightEncoder.getCount();
    long L_error = leftCount - targetCount; // 計算誤差
    long R_error = rightCount - targetCount;

    // 誤差在容差範圍內，完成
    if ((abs(L_error) < TOLERANCE) && (abs(R_error) < TOLERANCE))
    {
      break;
    }

    // 判斷各輪是否超過或未達目標
    bool L_over = L_error > TOLERANCE;   // 左輪超過
    bool L_under = L_error < -TOLERANCE; // 左輪未達
    bool R_over = R_error > TOLERANCE;   // 右輪超過
    bool R_under = R_error < -TOLERANCE; // 右輪未達

    // 動態計算調整速度
    int L_adjustSpeed = map(abs(L_error), TOLERANCE, 100, L_MIN_SPEED, -50);
    L_adjustSpeed = constrain(L_adjustSpeed, L_MIN_SPEED, -50);
    int R_adjustSpeed = map(abs(R_error), TOLERANCE, 100, R_MIN_SPEED, 50);
    R_adjustSpeed = constrain(R_adjustSpeed, R_MIN_SPEED, 50);

    // 根據各輪狀態同時調整
    int L_speed = 0;
    int R_speed = 0;

    // 左輪修正
    if (L_over)
      L_speed = -L_adjustSpeed; // 超過 → 反轉
    else if (L_under)
      L_speed = L_adjustSpeed; // 未達 → 正轉

    // 右輪修正
    if (R_over)
      R_speed = -R_adjustSpeed; // 超過 → 反轉
    else if (R_under)
      R_speed = R_adjustSpeed; // 未達 → 正轉

    motor(L_speed, R_speed);
    delay(30);
    stop();
    delay(10);
    attempts++;
  }
}

// --- 伺服馬達控制 ---
void arm_up()
{
  arm.write(ARM_UP);
}

void arm_down()
{
  arm.write(ARM_DOWN);
}

void arm_down_unload()
{
  arm.write(ARM_DOWN_UNLOAD);
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
  claw_open();
  delay(200);
}

// --- 伺服馬達控制 ---
void camera_front()
{
  camera.write(CAMERA_FRONT);
}

void camera_left()
{
  camera.write(CAMERA_LEFT);
}

void camera_right()
{
  camera.write(CAMERA_RIGHT);
}

// --- 物品辨識 ---
int color_detect()
{
  int target = -1;
  while (target == -1)
  {
    huskylens.request();
    if (huskylens.countBlocks() > 0)
    {
      if (huskylens.countBlocks(2) > 0)
      {
        target = 1; // 胡蘿蔔
        return target;
      }
      else if (huskylens.countBlocks(1) > 0)
      {
        target = 0; // 番茄
        return target;
      }
      else if (huskylens.countBlocks(3) > 0)
      {
        target = 2; // 玉米
        return target;
      }
    }
    delay(100);
  }
}

// ===== 主程式 =====
void setup()
{
  Serial.begin(9600);

  // --- 伺服馬達定時器分配 (Timer 0 給伺服馬達) ---
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1); // Timer 1 預留給其他用途（如LEDC PWM）

  // --- 伺服馬達初始化 ---
  arm.setPeriodHertz(50);               // 標準 50Hz 伺服馬達
  arm.attach(ARM_PIN, 500, 2400);       // SG90 脈寬範圍 500~2400us
  claw.setPeriodHertz(50);              // 標準 50Hz 伺服馬達
  claw.attach(CLAW_PIN, 500, 2400);     // SG90 脈寬範圍 500~2400us
  camera.setPeriodHertz(50);            // 標準 50Hz 伺服馬達
  camera.attach(CAMERA_PIN, 500, 2400); // SG90

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

  Wire.begin();
  while (!(huskylens.begin(Wire)))
  {
    Serial.println("HuskyLens not detected. Please check wiring.");
    delay(100);
  }

  int ProductB = -1;
  // 0 - 番茄 | 1 - 胡蘿蔔 | 2 - 玉米

  // PID OK 參數
  // 40, 0, 0, 80
  //======================================================================決賽程式開始（灰車）==============================================================
  camera_front();
  claw_open();
  delay(200);
  arm_down();
  delay(200);
  int error = 0;
  int second_down_back_delay = 150; // 第二次下降後的後退時間
  int third_down_back_delay = 350;  // 第三次下降後的後退時間
  int all_kp = 42;
  int all_kd = 1;
  int turn_turn_delay = 1000;
  int turn_turn_90_delay = 250; // 350 -> 250 (電池滿電時)

  p_fw_v2(3000);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  turn_turn(0, 200, 1000); // 左轉0ms之後進行PID對齊1000ms
  Padilla_trail(false, []()
                { return (IR_RR_read() == 1 || IR_LL_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  delay(50);
  p_fw_v2(225);
  turn_turn(0, turn_turn_90_delay, turn_turn_delay); // 左轉300ms之後進行PID對齊
  Padilla_trail(false, []()
                { return (IR_RR_read() == 1 || IR_LL_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  big_stop();
  delay(500);
  ProductB = color_detect();
  pick_up();
  p_right(90);
  stop();
  p_fw_v2(2500);
  Padilla_trail(false, []()
                { return (IR_L_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  turn_turn(0, 200, 1000); // 左轉200ms之後進行PID對齊1000ms
  error = Padilla_trail(true, []()
                        { return (IR_RR_read() == 1 || IR_LL_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  if (ProductB == 0)
  {
    leftEncoder.clearCount();
    // Padilla_trail(true, []()
    //               { return (false); }, all_kp, all_kd, 0, 80, 1200, error);
    Padilla_trail(true, []()
                  { return (leftEncoder.getCount() >= 3000); }, all_kp, all_kd, 0, 80, 0, error);
    stop();
    p_right(110);
    put_down();
    delay(200);
    backward();
    delay(50);
    p_left(180);
    stop();
    p_fw_v2(200);
    while (!(IR_L_read() == 1 || IR_M_read() == 1 || IR_R_read() == 1))
    {
      forward();
    }
    p_fw_v2(50);
    stop();
    delay(1000);
    turn_turn(1, 250, turn_turn_delay); // 右轉300ms之後進行PID對齊
    // Padilla_trail(true, []()
    //               { return (false); }, all_kp, all_kd, 0, 80, 1000, 0);
    Padilla_trail(false, []()
                  { return (IR_RR_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  }
  else if (ProductB == 1)
  {
    leftEncoder.clearCount();
    // Padilla_trail(true, []()
    //               { return (false); }, all_kp, all_kd, 0, 80, 2450, error);
    Padilla_trail(true, []()
                  { return (leftEncoder.getCount() >= 7500); }, all_kp, all_kd, 0, 80, 0, error);
    stop();
    stop();
    p_right(115);
    put_down();
    delay(200);
    backward();
    delay(50);
    turn_turn(1, 100, turn_turn_delay); // 右轉100ms之後進行PID對齊
    motor(-200, 200);
    delay(5);
    p_left(10);
    stop();
    delay(500);
    p_fw_v2(2500);
    while (!(IR_RR_read() == 1))
    {
      forward();
    }
    turn_turn(1, 150, turn_turn_delay);
    Padilla_trail(false, []()
                  { return (IR_RR_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  }
  else
  {
    leftEncoder.clearCount();
    turn_turn(1, turn_turn_90_delay, turn_turn_delay);
    Padilla_trail(true, []()
                  { return (leftEncoder.getCount() >= 3000); }, all_kp, all_kd, 0, 80, 0, error);
    stop();
    p_left(115);
    p_fw_v2(20);
    put_down();
    delay(200);
    backward();
    delay(50);
    turn_turn(1, 200, turn_turn_delay); // 左轉100ms之後進行PID對齊
    leftEncoder.clearCount();
    Padilla_trail(true, []()
                  { return (leftEncoder.getCount() >= 3000); }, all_kp, all_kd, 0, 80, 1250, 0);
    stop();
    turn_turn(0, 50, turn_turn_delay);
    motor(-200, 200);
    delay(5);
    p_left(20);
    p_fw_v2(2500);
    while (!(IR_RR_read() == 1))
    {
      forward();
    }
    turn_turn(1, 150, turn_turn_delay);
    Padilla_trail(false, []()
                  { return (IR_RR_read() == 1); }, all_kp, all_kd, 0, 80, 0, 0);
  }
  stop();
}

void loop()
{
  // TODO: 在此撰寫主程式邏輯
}

// TODO:如何使用PID_spin_to_center函式
// 功能：自動旋轉/對準黑線並驗證IR_M持續500毫秒
//
// 使用示例：
//   PID_spin_to_center(100, 70, 50, 0, 500);        // 左轉對齊+驗證500ms
//   PID_spin_to_center(100, 70, 50, 1, 500);        // 右轉對齊+驗證500ms
//   PID_spin_to_center(50, 70, 50, 2, 500);         // 單純對準+驗證500ms
//
// 說明：
//   - 所有模式完成後都會自動驗證IR_M持續看到線500毫秒
//   - mode 0: 逆時針左轉
//   - mode 1: 順時針右轉
//   - mode 2: 只做微調（baseRotateSpeed自動為0）
