#include "software_timer.h"

extern SemaphoreHandle_t xModeChangeSem;

TimerHandle_t xDebounceTimer = NULL;

//Essentially in the task environment
void vDebounceTimerCallback(TimerHandle_t xTimer)
{
	if(HAL_GPIO_ReadPin(JOYSTICKBTN_GPIO_Port, JOYSTICKBTN_Pin) == RESET)
	{
			printf("BTN pressed and comfirmed!\r\n");
			//xSemaphoreGiveFromISR(pxModeChange, &xHigherPriorityTaskWoken);
			xSemaphoreGive(xModeChangeSem);
	}
}

