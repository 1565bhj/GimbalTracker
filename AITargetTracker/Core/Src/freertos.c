/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
  #include <stdio.h>
	#include <string.h>
	#include <stdlib.h>
	#include "projdefs.h"
	#include "sys.h"
	#include "SDRAM.h"
	#include "dcmi_ov5640.h"
	#include "stm32h7xx_hal.h"
	#include "gimbal.h"
	#include "software_timer.h"
	#include "joystick.h"
	#include "transmit_image.h"
	#include "FOC.h"
	#include "AS5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAMERA_BUF_ADDR	(u32)0xC0000000
#define CAMERA_BUF_SIZE	(128*96*2)/4

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern ADC_HandleTypeDef 	hadc1;
extern DMA_HandleTypeDef 	hdma_adc1;
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef 	htim4;
extern TIM_HandleTypeDef 	htim2;
extern TimerHandle_t 			xDebounceTimer;
extern TIM_HandleTypeDef  htim6;
extern TIM_HandleTypeDef  htim7;

extern  PID_TypeDef velPID;
extern  AS5600_HandleTypedef as;
extern  FOC_HandleTypeDef xFOC;

static PID_TypeDef 			xPID_x;
static PID_TypeDef 			xPID_y;
static Gimbal_TypeDef 	xGim;
static Joystick_TypeDef xJoy;
static u16 							pTarget[2];
static u16 							pJoyBuf[2];
static float 						pfRequire[2];
static FlagStatus 			control_mode = RESET;
static QueueHandle_t xServoQueue = NULL;
//static SVPWM_HandleTypeDef foc;

SemaphoreHandle_t xModeChangeSem = NULL;
SemaphoreHandle_t xWaitDMASem = NULL;
SemaphoreHandle_t xMotorSem = NULL;
QueueHandle_t xRxQueue = NULL;

char pUartRx_buf[128];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void OV5640Task(void *);
void Servotest(void *);
void vModeChangeTask(void *argument);
void test(void *argument);
void ProccessTargetTask(void *argument);
void ProccessRxTask(void *argument);
void ControlTask(void *argument);


HAL_StatusTypeDef SplitKeyVal(char* key, char* val, char* str);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	//Calibration ADC
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
	Gimbal_Init(&xPID_x, &xPID_y, &xJoy, &xGim);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (u8*)pUartRx_buf, sizeof(pUartRx_buf));
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
	
	SVPWM_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	xModeChangeSem = xSemaphoreCreateBinary();
	xWaitDMASem = xSemaphoreCreateBinary();
	xMotorSem = xSemaphoreCreateBinary();
	
	xSemaphoreGive(xModeChangeSem);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	xDebounceTimer = xTimerCreate("DebounceTimer", pdMS_TO_TICKS(20), pdFALSE, (void*)0, vDebounceTimerCallback);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	xServoQueue = xQueueCreate(1, sizeof(ServoInc_TypeDef));
	xRxQueue = xQueueCreate(5, sizeof(pUartRx_buf));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
  xTaskCreate(test, "test", 256, NULL, osPriorityAboveNormal, NULL);
  //xTaskCreate(OV5640Task, "OV5640Task", 128, NULL, osPriorityAboveNormal, NULL);
  //xTaskCreate(vModeChangeTask, "vModeChangeTask", 128, NULL, osPriorityAboveNormal, NULL);
	//xTaskCreate(ProccessTargetTask, "ProccessTargetTask", 128, NULL, osPriorityAboveNormal, NULL);
	xTaskCreate(ProccessRxTask, "ProccessRxTask", 256, NULL, osPriorityAboveNormal, NULL);
	//xTaskCreate(ControlTask, "ControlTask", 128, NULL, osPriorityAboveNormal, NULL);
	
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }  
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

float velocity = 10;
float angle = 0;
extern FOC_HandleTypeDef xSPWMFOC;
void test(void *argument)
{
	SVPWM_Init();
	float angle = 0;
	while(1)
	{
		xSemaphoreTake(xMotorSem, portMAX_DELAY);
		SVPWM_CalcForTargetVelocity(&xFOC, velocity);
		//SVPWM_CalcForTargetAngle(&foc, angle);
		//SPWM_CalcForTargetVelocity(&xSPWMFOC, velocity);
		//SPWM_CalcForTargetAngle(&xSPWMFOC, angle);		
	}
}

void vModeChangeTask(void *argument)
{
	while(1)
	{
			xSemaphoreTake(xModeChangeSem, portMAX_DELAY);

			control_mode = control_mode ? RESET : SET;
			if(control_mode)
			{	
				//Turn on LED
				HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)pJoyBuf, 2);
			}
			else
			{
				//Turn off LED
				HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
				// stop ADC2DMA
//				HAL_ADC_Stop_DMA(&hadc1);
//    
//				// stop ADC Collect
//				HAL_ADC_Stop(&hadc1);
//    
//				// clear DMA flag
//				__HAL_DMA_CLEAR_FLAG(&hdma_adc1, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_adc1));
			}		
			
		printf("JOYSTICKBTN Trigger:%s\r\n", control_mode ? "Manual" : "Auto");
		
	}
}

void ProccessTargetTask(void *argument)
{
	ServoInc_TypeDef xServoInc;
	while(1)
	{
		if(control_mode)
		{
			//Mannul
			Process_Joystick(&xJoy, pJoyBuf, &xServoInc);
		}else
		{
			//Auto-----
			Process_AIResult(&xPID_x, &xPID_y, pfRequire, pTarget, &xServoInc);
		}
		vTaskDelay(pdMS_TO_TICKS(12));
		xQueueOverwrite(xServoQueue, &xServoInc);
		
	}
}

void ProccessRxTask(void *argument)
{
	char pData[128];
	char key[8];
	char val[5];
	float value;
	while(1)
	{
		xQueueReceive(xRxQueue, pData, portMAX_DELAY);
		
		if(SplitKeyVal(key, val, pData) == HAL_ERROR) continue;
		value = atof(val);
		if(strcmp(key, "uq") == 0)
		{
			xFOC.Uq = value;
		}else if(strcmp(key, "vel") == 0)
		{
			velocity = value;
		}else if(strcmp(key, "vp") == 0)
		{
			velPID.Kp = value;
		}else if(strcmp(key, "vi") == 0)
		{
			velPID.Ki = value;
		}else if(strcmp(key, "vd") == 0)
		{
			velPID.Kd = value;
		}else if(strcmp(key, "angle") == 0)
		{
			angle = value;
		}
		
	}
}

void ControlTask(void *argument)
{
	ServoInc_TypeDef xServoInc;
	while(1)
	{
		xQueueReceive(xServoQueue, &xServoInc, portMAX_DELAY);
		Control_Loop(&xGim, xServoInc);
	}
}


void OV5640Task(void *argument)
{
	//OV5640_DMA_Transmit_Continuous(Camera_Buffer_addr, Camera_Buffer_Size);
	OV5640_DMA_Transmit_Snapshot(CAMERA_BUF_ADDR, CAMERA_BUF_SIZE);
	while(1)
	{
		vTaskDelay(2000);
		printf("task..\r\n");
		if ( OV5640_FrameState == 1 )
		{		
  			OV5640_FrameState = 0;
//				printf("array:");
//				for(int i = 0; i < 100;i++)
//					printf("%x ",array[i]);
		}
	}
}

HAL_StatusTypeDef SplitKeyVal(char* key, char* val, char* str)
{
	char *key_ptr = NULL,*val_ptr = NULL;
	
	key_ptr = strtok(str, "=");
	if(key_ptr == NULL || strlen(key_ptr) == 0) return HAL_ERROR;
	
	val_ptr = strtok(NULL, "=");
	if(val_ptr == NULL || strlen(val_ptr) == 0) return HAL_ERROR;
	
	if(strlen(key_ptr) > 5 || strlen(val_ptr) > 5) return HAL_ERROR;
	
	strcpy(key, key_ptr);
	strcpy(val, val_ptr);
	return HAL_OK;
}


/* USER CODE END Application */
