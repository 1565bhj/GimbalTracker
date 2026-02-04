#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "sys.h"

typedef struct {
    // 参数 (可通过触摸屏调节)
    volatile float Kp; // 比例系数
    volatile float Ki; // 积分系数
    volatile float Kd; // 微分系数
	
    // 限制
    float out_max; // 输出增量限幅（防止舵机单次动作过大，保护机械结构）
    float dead_zone; // 死区（误差在这个范围内不动作，防止云台抖动）
		
		float integral_max;
		//float Ts; // 周期
	
		float i_term_last;

    // 历史误差
    float err_now; // 当前误差
    float err_last; // 上一次误差
    float err_prev; // 上上次误差
	
		float lastTime;

} PID_TypeDef;

// 函数声明
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float dead_zone, float integral_max);
float PID_Inc(PID_TypeDef *pid, float err);

float PID_Loc(PID_TypeDef *pid, float err);
#endif
