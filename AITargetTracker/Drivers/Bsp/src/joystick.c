#include "joystick.h"

void Joystick_Init(Joystick_TypeDef* xJoy, int centerVal, float fMaxStep, int joyDeadZone)
{                                   
	xJoy->deadZone = joyDeadZone;
	xJoy->centerVal = centerVal;
	xJoy->fMaxStep = fMaxStep;
}

/**
 * @brief 处理摇杆数据并计算舵机步长
 */
void Process_Joystick(Joystick_TypeDef *joy, u16* pJoy_Buf, float* pfDelte) {
    // 1. 处理 X 轴 (水平)
    int offset_x = (int)pJoy_Buf[0] - joy->centerVal;
    
    if (abs(offset_x) <= joy->deadZone) {
        pfDelte[0] = 0; // 处于死区内，静止
    } else {
        // 将偏移量归一化并映射到步长 (0 ~ fmaxStep)
        // 映射公式: fmaxStep * (当前值 - 死区边界) / (最大值 - 死区边界)
				float absInc_x = (float)(abs(offset_x) - joy->deadZone) / (joy->centerVal - joy->deadZone) * joy->fMaxStep;
				pfDelte[0] = offset_x > 0 ? absInc_x : -absInc_x;
       
    }

    // 2. 处理 Y 轴 (垂直) - 逻辑同上
    int offset_y = (int)pJoy_Buf[1] - joy->centerVal;

    if (abs(offset_y) <= joy->deadZone) {
        pfDelte[1] = 0;
    } else {
        float absInc_y = (float)(abs(offset_y) - joy->deadZone) / (joy->centerVal - joy->deadZone) * joy->fMaxStep;
				pfDelte[1] = offset_y > 0 ? absInc_y : -absInc_y;
    }
		
}

