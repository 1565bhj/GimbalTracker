#include "nonblock_print.h"

extern UART_HandleTypeDef huart1;

//存储打印信息的队列
static QueueHandle_t xPrintQueue = NULL;
//发送完成信号量
SemaphoreHandle_t xPrintDoneSem = NULL;

void Print_Init(void)
{
	xPrintQueue = xQueueCreate(PRINT_QUEUE_LEN, PRINT_EACH_SIZE);
	xPrintDoneSem = xSemaphoreCreateBinary();
	xSemaphoreGive(xPrintDoneSem);
}

void NonBlock_PrintTask(void *argument)
{
	char pcRecvBuf[PRINT_EACH_SIZE];
	while(1)
	{
		xQueueReceive(xPrintQueue, pcRecvBuf, portMAX_DELAY);
		xSemaphoreTake(xPrintDoneSem, portMAX_DELAY);
		
		HAL_UART_Transmit_DMA(&huart1, (u8*)pcRecvBuf, strlen(pcRecvBuf));
	}
}
	
void print(const char *format, ...)
{
  char pcPrintBuf[PRINT_EACH_SIZE];
	va_list args;
	
	//格式化字符串(vsnprintf 防止缓冲区溢出)
	va_start(args, format);
	vsnprintf(pcPrintBuf, PRINT_EACH_SIZE, format, args);
	va_end(args);
	//将字符串放入队列
	xQueueSend(xPrintQueue, pcPrintBuf, 0);
	
}
