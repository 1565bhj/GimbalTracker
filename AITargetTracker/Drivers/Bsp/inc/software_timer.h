#ifndef __SOFTWARETIMER_H_
#define __SOFTWARETIMER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"
#include <stdio.h>


void vDebounceTimerCallback(TimerHandle_t xTimer);

#endif
