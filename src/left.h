// 左側模組介面 - 不修改右側檔案
#ifndef LEFT_H
#define LEFT_H

#include <Arduino.h>

// 初始化左側硬體（腳位、編碼器、PWM）
void left_init();

// 設定左馬達速度，範圍 -255 ~ 255（正值前進、負值後退）
void left_motor(int speed);

// 取得左側編碼器計數（若尚未實作則回傳 0）
int left_getEncoder();

#endif // LEFT_H
