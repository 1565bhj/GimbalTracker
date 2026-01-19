#ifndef _NONBLOCK_PRINT_H
#define _NONBLOCK_PRINT_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define PRINT_QUEUE_LEN 32
#define PRINT_EACH_SIZE 128

void Print_Init(void);
void NonBlock_PrintTask(void *argument);
void print(const char *format, ...);

#endif
