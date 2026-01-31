#include "left.h"

// 簡單的左側實作範例（可依實際硬體補齊）
static volatile int encoder_count_left = 0;

void left_init() {
    // TODO: 設定腳位、PWM、編碼器中斷等
}

void left_motor(int speed) {
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    // TODO: 實作 PWM 控制，將 speed 映射到實際 PWM 輸出
    // 範例：正值為前進、負值為後退
}

int left_getEncoder() {
    return encoder_count_left;
}
