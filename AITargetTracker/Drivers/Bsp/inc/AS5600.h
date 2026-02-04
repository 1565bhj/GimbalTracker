#ifndef __AS5600_H
#define __AS5600_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
#include "pid.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// AS5600 I2C 地址（固定）
#define AS5600_I2C_ADDR    (0x36 << 1) // 7位地址是 0x36，HAL库需要左移一位

// AS5600 关键寄存器地址
#define AS5600_REG_RAW_ANGLE  0x0C
#define AS5600_REG_ANGLE      0x0E
#define AS5600_REG_STATUS     0x0B
#define AS5600_REG_AGC        0x1A
#define AS5600_REG_MAGNITUDE  0x1B


typedef struct{
	I2C_HandleTypeDef* hi2c;
	
	uint8_t rxBuf[2];

	float curAngle;
	float lastAngle;
	
	float velocity;
	
	float lastTime;
	
	int32_t rotations_full; // 总圈数计数
  int32_t rotations_last; //用于速度计算的先前完整旋转圈数
}AS5600_HandleTypedef;

// 函数声明
uint8_t  AS5600_IsConnected(AS5600_HandleTypedef* as);
uint16_t AS5600_GetRawAngle(AS5600_HandleTypedef* as);
float    AS5600_GetDegAngle(AS5600_HandleTypedef* as);
float    AS5600_GetRadAngle(AS5600_HandleTypedef* as);
float 	 AS5600_GetVelocity(AS5600_HandleTypedef* as, float d_angle);
void 		 AS5600_Update(AS5600_HandleTypedef* as);


#endif
