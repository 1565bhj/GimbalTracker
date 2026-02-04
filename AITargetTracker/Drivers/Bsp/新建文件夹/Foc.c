#include "FOC.h"

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;

PID_TypeDef velPID = {1, 0, 0, VOLTAGE_LIMIT/3.0f, 0, 0};
PID_TypeDef anglePID = {4, 0, 0, VOLTAGE_LIMIT/3.0f, 0, 0};

PID_TypeDef SPWMvelPID = {1, 0, 0, VOLTAGE_LIMIT/3.0f, 0, 0};
PID_TypeDef SPWManglePID = {4, 0, 0, VOLTAGE_LIMIT/3.0f, 0, 0};

AS5600_HandleTypedef as = {&hi2c2};
FOC_HandleTypeDef xFOC = {&anglePID, &velPID, NULL, &as, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
FOC_HandleTypeDef xSPWMFOC = {&SPWManglePID, &SPWMvelPID, NULL, &as, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
SVPWM_HandleTypeDef xSVPWM = {&xFOC};
LowPassFilter_TypeDef xVelocityFilter = {0.2, 0, 0};

void SVPWM_Init(void)
{	
	//初始化PID结构体
	
	
	//初始化AS5600结构体
	
	
	//初始化SVPWM结构体
	

	AS5600_Update(xSVPWM.foc->as);
	
	xSVPWM.foc->Ud = 0;
	
	HAL_TIM_PWM_Start(xSVPWM.foc->htim,xSVPWM.foc->CHa);
	HAL_TIM_PWM_Start(xSVPWM.foc->htim,xSVPWM.foc->CHb);
	HAL_TIM_PWM_Start(xSVPWM.foc->htim,xSVPWM.foc->CHc);
}

float GetElectricalAngle(float mechanical_angle, float pole_pairs)
{
	return (mechanical_angle * pole_pairs);
}

float	NormalizeElecAngle(float elec_angle)
{
	float a = fmodf(elec_angle, _2PI);
	a = (a >= 0) ? a : (a + _2PI);
	return a;
}

void ParkInverse(float Uq, float Ud, float elec_angle, float* Uab)
{
	float Ualpha = Ud * cosf(elec_angle) - Uq * sinf(elec_angle); 
  float Ubeta  = Ud * sinf(elec_angle) + Uq * cosf(elec_angle); 
	Uab[0] = Ualpha;
	Uab[1] = Ubeta;
}

void ClarkInverse(float Ualpha, float Ubeta, float* Uabc)
{
	float Ua = Ualpha + VOLTAGE_SUPPLY/2;
  float Ub = (sqrt(3)*Ubeta-Ualpha)/2 + VOLTAGE_SUPPLY/2;
  float Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + VOLTAGE_SUPPLY/2;
	Uabc[0] = Ua;
	Uabc[1] = Ub;
	Uabc[2] = Uc;
}

void SetPWM(FOC_HandleTypeDef* foc)
{            	
	// 限幅并赋值最终三相占空比
	foc->ta = _CONSTRAIN(foc->ta, 0.0f, 1.0f);
  foc->tb = _CONSTRAIN(foc->tb, 0.0f, 1.0f);
  foc->tc = _CONSTRAIN(foc->tc, 0.0f, 1.0f);
	//+0.5f四舍五入
	u32 cmp_a = foc->ta * TIM_ARR + 0.5f;
  u32 cmp_b = foc->tb * TIM_ARR + 0.5f;
  u32 cmp_c = foc->tc * TIM_ARR + 0.5f;
	// 2. 限幅：防止浮点误差导致超出ARR范围
  cmp_a = _CONSTRAIN(cmp_a, 0U, (uint32_t)TIM_ARR);
  cmp_b = _CONSTRAIN(cmp_b, 0U, (uint32_t)TIM_ARR);
  cmp_c = _CONSTRAIN(cmp_c, 0U, (uint32_t)TIM_ARR);
	//写入PWM
	__HAL_TIM_SET_COMPARE(foc->htim, foc->CHa, cmp_a);
	__HAL_TIM_SET_COMPARE(foc->htim, foc->CHb, cmp_b);
	__HAL_TIM_SET_COMPARE(foc->htim, foc->CHc, cmp_c);
}

void SVPWM_GetSector(SVPWM_HandleTypeDef* xSVPWM)
{
	u8 sec = 0;
	
	float u1 = xSVPWM->foc->Ubeta;
	float u2 = (SQRT3*xSVPWM->foc->Ualpha - xSVPWM->foc->Ubeta)/2.0f;
  float u3 = (-SQRT3*xSVPWM->foc->Ualpha - xSVPWM->foc->Ubeta)/2.0f;
	
	// 扇区编码
  if(u1 > 0) sec |= 1;
  if(u2 > 0) sec |= 2;
  if(u3 > 0) sec |= 4;
	
	switch(sec)
  {												  
    case 1: sec = 2; break; 
    case 2: sec = 5; break; 
    case 3: sec = 1; break; 
    case 4: sec = 3; break; 
    case 5: sec = 6; break; 
    case 6: sec = 4; break; 
    default: sec = 1; break;
  }
	xSVPWM->sector = sec;
}

///**
// * @brief  SVPWM矢量作用时间计算（T1/T2/T0）
// * @param  Ualpha/Ubeta: αβ轴电压, Udc: 直流母线电压
// */
void SVPWM_CalcVectorTime(SVPWM_HandleTypeDef* xSVPWM)
{
  // 归一化αβ电压
  float ua = xSVPWM->foc->Ualpha / VOLTAGE_SUPPLY;
  float ub = xSVPWM->foc->Ubeta  / VOLTAGE_SUPPLY;
  // 计算T1/T2（通用公式，基于当前扇区）
  xSVPWM->T1 = SQRT3 * (ua * sinf(xSVPWM->sector*_PI_3) - ub * cosf(xSVPWM->sector*_PI_3));
  xSVPWM->T2 = SQRT3 * (ub * cosf((xSVPWM->sector-1)*_PI_3) - ua * sinf((xSVPWM->sector-1)*_PI_3));

	float Tsum = xSVPWM->T1 + xSVPWM->T2;
	//等比例缩放
	if(Tsum > 1 && Tsum > 1e-6f)
	{
		xSVPWM->T1 /= Tsum;
		xSVPWM->T2 /= Tsum;
	}
	// 3. 限幅（防止浮点误差导致T1/T2为负）
  xSVPWM->T1 = _CONSTRAIN(xSVPWM->T1, 0.0f, 1.0f);
  xSVPWM->T2 = _CONSTRAIN(xSVPWM->T2, 0.0f, 1.0f);
  // 计算零矢量时间T0 = 1 - T1 - T2
  xSVPWM->T0 = 1.0f - xSVPWM->T1 - xSVPWM->T2;
  xSVPWM->T0 = _CONSTRAIN(xSVPWM->T0, 0.0f, 1.0f);
}

/**
 * @brief  SVPWM 7段式占空比分配（核心：7段时序）
 */
void SVPWM_CalcDuty(SVPWM_HandleTypeDef* xSVPWM)
{
  float T0_2 = xSVPWM->T0 / 2.0f; // 零矢量均分到PWM周期前后（7段式核心）
  // 根据扇区分配T1/T2/T0到三相
  switch(xSVPWM->sector)
  {
    case 1: xSVPWM->foc->ta=T0_2;      
						xSVPWM->foc->tb=T0_2+xSVPWM->T1; 
						xSVPWM->foc->tc=T0_2+xSVPWM->T1+xSVPWM->T2; 
						break;
    case 2: xSVPWM->foc->ta=T0_2+xSVPWM->T2; 
						xSVPWM->foc->tb=T0_2;      
						xSVPWM->foc->tc=T0_2+xSVPWM->T1+xSVPWM->T2; 
						break;
    case 3: xSVPWM->foc->ta=T0_2+xSVPWM->T1+xSVPWM->T2; 
						xSVPWM->foc->tb=T0_2;      
						xSVPWM->foc->tc=T0_2+xSVPWM->T1; 
						break;
    case 4: xSVPWM->foc->ta=T0_2+xSVPWM->T1+xSVPWM->T2; 
						xSVPWM->foc->tb=T0_2+xSVPWM->T2; 
						xSVPWM->foc->tc=T0_2;      
						break;
    case 5: xSVPWM->foc->ta=T0_2+xSVPWM->T1; 
						xSVPWM->foc->tb=T0_2+xSVPWM->T1+xSVPWM->T2; 
						xSVPWM->foc->tc=T0_2;      
						break;
    case 6: xSVPWM->foc->ta=T0_2;      
						xSVPWM->foc->tb=T0_2+xSVPWM->T1+xSVPWM->T2; 
						xSVPWM->foc->tc=T0_2+xSVPWM->T2; 
						break;
    default: xSVPWM->foc->ta=0.5f; xSVPWM->foc->tb=0.5f; xSVPWM->foc->tc=0.5f; break;
  }
  
}

void SVPWM_CalcForTargetVelocity(SVPWM_HandleTypeDef* xSVPWM, float refVelocity)
{
	//当前角速度
	float currentVelocity = GetVelocity(xSVPWM->foc->as);
	
	//计算err
	float errVelocity = refVelocity - currentVelocity;
	//速度环
	xSVPWM->foc->Uq = PID_Calculate_Locational(xSVPWM->foc->velPID, (errVelocity) * 180 / _PI);
	
	//当前角度rad值
	float elecAngle = GetElecAngle(xSVPWM->foc->as);
	
	SVPWM_setTorque(xSVPWM, elecAngle);
}

void SVPWM_CalcForTargetAngle(SVPWM_HandleTypeDef* xSVPWM, float refAngle)
{
		float currentAngle, refVelocity;
		float errAngle;

	//获取当前角度rad
	currentAngle = GetAngle(xSVPWM->foc->as);
	//计算err
	errAngle = refAngle - currentAngle;
	if(errAngle > _PI) errAngle-=_PI;
	if(errAngle < -_PI) errAngle+=_PI;
	
	printf("errAngle=%.2f\r\n", errAngle);
	//位置环
	refVelocity = PID_Calculate_Locational(xSVPWM->foc->anglePID, errAngle * 180.0f / _PI);
	
	printf("refVelocity=%.2f\r\n", refVelocity);
	SVPWM_CalcForTargetVelocity(xSVPWM, refVelocity);
}

void SPWM_CalcForTargetVelocity(FOC_HandleTypeDef* foc, float refVelocity)
{
	//当前角速度
	float currentVelocity = GetVelocity(foc->as);

	printf("currentVelocity=%.2f\r\n", currentVelocity);
	//计算err
	float errVelocity = refVelocity - currentVelocity;
	printf("errVelocity=%.2f\r\n", errVelocity);
	//速度环
	foc->Uq = PID_Calculate_Locational(foc->velPID, errVelocity * 180.0f / _PI);
	printf("Uq=%.2f\r\n", foc->Uq);

	//计算电角度
	float radAngle = GetElecAngle(foc->as);
		
	//归一化电角度
	radAngle = NormalizeElecAngle(radAngle);
	
	//SPWM_setTorque(foc. );
}

void SPWM_CalcForTargetAngle(FOC_HandleTypeDef* foc, float refAngle)
{
		
	float currentAngle, refVelocity;
	float errAngle;

	//获取当前角度rad
	currentAngle = GetAngle(foc->as);
	//计算err
	errAngle = refAngle - currentAngle;
	if(errAngle > _PI) errAngle-=_PI;
	if(errAngle < -_PI) errAngle+=_PI;
	
	printf("errAngle=%.2f\r\n", errAngle);
	//位置环
	refVelocity = PID_Calculate_Locational(foc->anglePID, errAngle * 180.0f / _PI);
	
	printf("refVelocity=%.2f\r\n", refVelocity);

	SPWM_CalcForTargetVelocity(foc, refVelocity*DIR);
}

/***************************************************
*********************接口函数***********************
***************************************************/
float GetVelocity(AS5600_HandleTypedef* as)
{
	return LowPass_Filter(&xVelocityFilter, as->velocity) * DIR;
}

float GetElecAngle(AS5600_HandleTypedef* as)
{
	return NormalizeElecAngle(GetElectricalAngle(as->curAngle, POLE_PAIRS)) * DIR;
}

float GetAngle(AS5600_HandleTypedef* as)
{
	return as->curAngle * DIR;
}

void SVPWM_setTorque(SVPWM_HandleTypeDef* xSVPWM, float elecAngle)
{
	if(xSVPWM->foc->Uq < 0) elecAngle+=_PI;
	xSVPWM->foc->Uq = fabsf(xSVPWM->foc->Uq);
	
	//归一化电角度
	elecAngle = NormalizeElecAngle(elecAngle);
	
	//Park逆变换
	float Uab[2] = {0};
	ParkInverse(xSVPWM->foc->Uq, xSVPWM->foc->Ud, elecAngle, Uab);
	xSVPWM->foc->Ualpha = Uab[0];
	xSVPWM->foc->Ubeta = Uab[1];
	
	//获取当前扇区
	SVPWM_GetSector(xSVPWM);
	//计算矢量作用时间
	SVPWM_CalcVectorTime(xSVPWM);
	//7段占空比
	SVPWM_CalcDuty(xSVPWM);
	//发送PWM
	SetPWM(xSVPWM->foc);
	
	AS5600_Update(xSVPWM->foc->as);
}

void SPWM_setTorque(FOC_HandleTypeDef* foc, float elecAngle)
{
	//Park逆变换
	float Uab[2] = {0};
	ParkInverse(foc->Uq, foc->Ud, elecAngle, Uab);
	foc->Ualpha = Uab[0];
	foc->Ubeta = Uab[1];
	
	//Clark逆变换
	float Uabc[3] = {0};
	ClarkInverse(foc->Ualpha, foc->Ubeta, Uabc);
	foc->ta = Uabc[0] / VOLTAGE_SUPPLY;
	foc->tb = Uabc[1] / VOLTAGE_SUPPLY;
	foc->tc = Uabc[2] / VOLTAGE_SUPPLY;

	//发送PWM
	SetPWM(foc);
}
