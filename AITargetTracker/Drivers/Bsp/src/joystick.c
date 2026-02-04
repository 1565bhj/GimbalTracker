#include "joystick.h"



void Joystick_Init(Joystick_TypeDef* xJoy, int centerVal, float fMaxStep, int joyDeadZone)
{                                   
	xJoy->deadZone = joyDeadZone;
	xJoy->centerVal = centerVal;
	xJoy->fMaxStep = fMaxStep;
}



