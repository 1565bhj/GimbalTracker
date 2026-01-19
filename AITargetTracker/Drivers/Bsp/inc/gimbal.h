#ifndef __GIMBALCONTROL_H
#define __GIMBALCONTROL_H

#include <stdint.h>
#include <stdio.h>
#include "pid.h"
#include "sys.h"
#include "joystick.h"

// 舵机云台结构体
typedef struct {
    float current_x_angle; // 当前 X 轴角度
    float current_y_angle; // 当前 Y 轴角度
    
		float fIncX;	// X增量
		float fIncY;	// Y增量
	
    // 舵机物理限制 (例如 0度 到 180度)
    float servo_min_angle;
    float servo_max_angle;
} Gimbal_TypeDef;

// 系统初始化时调用
void Gimbal_Init(PID_TypeDef* xPidx, PID_TypeDef* xPidy, Joystick_TypeDef* xJoy, Gimbal_TypeDef* xGim);
void Servo_Set_Angle(u8 angle, u8 CH);
// target_x, target_y 是 AI 模型返回的目标中心坐标
void Control_Loop(Gimbal_TypeDef* xGim, float* pfDelte);

#endif
