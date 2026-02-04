#ifndef __LOWPASSFILTER_H
#define __LOWPASSFILTER_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
#include "pid.h"
#include "projdefs.h"
#include <stdio.h>

typedef struct
{
	float Tf;
	float val_last;
	float tick_last;
}LowPassFilter_TypeDef;

float LowPass_Filter(LowPassFilter_TypeDef* xLowPassFilter, float val);


#endif
