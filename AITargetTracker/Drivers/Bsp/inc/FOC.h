#ifndef __SVPWM_H
#define __SVPWM_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
#include "pid.h"
#include "AS5600.h"
#include "LowPassFilter.h"

#define VOLTAGE_SUPPLY 11.0f
#define POLE_PAIRS 		 7
#define VOLTAGE_LIMIT  10.0f
#define TIM_ARR				 (200-1)
#define DIR						 1//(-1)

#define _CONSTRAIN(val,low,high) ((val)<=(low)?(low):((val)>=(high)?(high):(val)))

typedef struct {	
		PID_TypeDef* anglePID;
		PID_TypeDef* velPID;
		PID_TypeDef* curPID;
	
		AS5600_HandleTypedef* as;
	
		TIM_HandleTypeDef* htim;
		u32 CHa;
	  u32 CHb;
	  u32 CHc;
		
		float ta;
		float tb;
		float tc;
		
		float Uq;
		float Ud;
	
		float Ualpha;
	  float Ubeta;

		u8 sector;
	
		float T1;
		float T2;
		float T0;

		float lastAngle;
		u32 time_last;	
		
} FOC_HandleTypeDef;


float GetElectricalAngle(float mechanical_angle, float pole_pairs);
float	NormalizeElecAngle(float elec_angle);
void ParkInverse(float Uq, float Ud, float elec_angle, float* Uab);
void ClarkInverse(float Ualpha, float Ubeta, float* Uabc);
void 	SVPWM_GetSector(FOC_HandleTypeDef* foc);
void 	SVPWM_CalcVectorTime(FOC_HandleTypeDef* foc);
void 	SVPWM_CalcDuty(FOC_HandleTypeDef* foc);
void 	SetPWM(FOC_HandleTypeDef* foc);
	
void 	SVPWM_CalcForTargetVelocity(FOC_HandleTypeDef* foc, float refVelocity);
void 	SVPWM_CalcForTargetAngle(FOC_HandleTypeDef* foc, float refAngle);

void 	SPWM_CalcForTargetVelocity(FOC_HandleTypeDef* foc, float refVelocity);
void 	SPWM_CalcForTargetAngle(FOC_HandleTypeDef* foc, float refAngle);

void 	SVPWM_Init(void);



void  SVPWM_setTorque(FOC_HandleTypeDef* foc, float elecAngle);
float GetAngle(AS5600_HandleTypedef* as);
float GetElecAngle(AS5600_HandleTypedef* as);
float GetVelocity(AS5600_HandleTypedef* as);
#endif
