#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include <stdint.h>
#include <math.h>
#include "sys.h"
#include <stdlib.h>

typedef struct {
    int deadZone; // 死区大小，建议 100~200
    int centerVal; // 中点值，通常 2048
    float fMaxStep; // 最大步长（每次循环改变的角度）
} Joystick_TypeDef;

void Joystick_Init(Joystick_TypeDef *joy, int centerVal, float fMaxStep, int joyDeadZone);

#endif
