#ifndef __TRANSMIT_IMAGE_H
#define __TRANSMIT_IMAGE_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>


// 图像参数定义
#define IMG_WIDTH    128    // 图像宽度
#define IMG_HEIGHT   96    // 图像高
#define PACKET_SIZE  1024   // 每次发送的字节数（分包，避免丢包）

// 帧头/帧尾（用于电脑端识别数据起始/结束）
#define FRAME_HEAD   0xAA55
#define FRAME_TAIL   0x55AA

enum{
    IMG_RGB565 = 0,
    IMG_RGB888,
    IMG_JPEG,      
};

void UART_Send_Image(uint8_t *img_buf, uint8_t img_format);
void UART_Send_JPEG(uint8_t *img_buf);
void UART_Send_RGB(uint8_t *img_buf, uint8_t img_format);


#endif
