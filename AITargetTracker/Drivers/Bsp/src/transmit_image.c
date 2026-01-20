#include "transmit_image.h"

extern UART_HandleTypeDef huart2;

//等待DMA传输完成的信号量
extern SemaphoreHandle_t xWaitDMASem;

static uint8_t send_buf[PACKET_SIZE];

/**
 * @brief  发送图像数据到串口
 * @param  img_buf: SDRAM中RGB565图像数据的起始地址
 * @param  img_format: 数据格式
		       0 RGB565
		       1 RGB888
		       2 JPEG
 */
void UART_Send_Image(uint8_t *img_buf, uint8_t img_format)
{
		if(img_buf == NULL)
		{
			printf("指针为空");
			return;
		}
    if(img_format == 2)
    {
			UART_Send_JPEG(img_buf);
    }else if(img_format < 2)
    {
			printf("Start to send RGBimage...\r\n");
			UART_Send_RGB(img_buf, img_format);
    }else
    {
			printf("请使用正确的格式");
    	return;
    }
}


void UART_Send_JPEG(uint8_t *img_buf)
{
    return;
}

void UART_Send_RGB(uint8_t *img_buf, uint8_t img_format)
{  
    
    uint32_t total_bytes;

    total_bytes = IMG_WIDTH * IMG_HEIGHT * (img_format == 0 ? 2 : 3);

    uint32_t send_index = 0;

    // 第一步：发送帧头+图像参数（固定16字节，方便电脑端解析）
    uint8_t frame_header[16] = {0};
    // 帧头（2字节）
    frame_header[0] = (FRAME_HEAD >> 8) & 0xFF;
    frame_header[1] = FRAME_HEAD & 0xFF;
    // 宽度（2字节）
    frame_header[2] = (IMG_WIDTH >> 8) & 0xFF;
    frame_header[3] = IMG_WIDTH & 0xFF;
    // 高度（2字节）
    frame_header[4] = (IMG_HEIGHT >> 8) & 0xFF;
    frame_header[5] = IMG_HEIGHT & 0xFF;
    // 格式（1字节）
    frame_header[6] = img_format;
    // 总字节数（4字节）
    frame_header[7] = (total_bytes >> 24) & 0xFF;
    frame_header[8] = (total_bytes >> 16) & 0xFF;
    frame_header[9] = (total_bytes >> 8) & 0xFF;
    frame_header[10] = total_bytes & 0xFF;
    // 预留5字节（校验/扩展）
		
		printf("send frame header...\r\n");
    if(HAL_UART_Transmit_DMA(&huart2, frame_header, 16) != HAL_OK)
		{
			printf("DMA传输失败");
		}
		xSemaphoreTake(xWaitDMASem, portMAX_DELAY);

    // 第二步：分包发送像素数据'
		int i = 0;
    while (send_index < total_bytes)
    {
        uint32_t remain_bytes = total_bytes - send_index;
        uint32_t send_bytes = (remain_bytes > PACKET_SIZE) ? PACKET_SIZE : remain_bytes;

        // 从SDRAM读取数据到发送缓冲区
//        for (uint32_t i = 0; i < send_bytes; i++)
//        {
//            send_buf[i] = img_buf[send_index + i];
//        }
				memcpy(send_buf, img_buf + send_index, send_bytes);

        // 串口发送当前包
				printf("send the %d packet...\r\n", i++);
        
				if(HAL_UART_Transmit_DMA(&huart2, send_buf, send_bytes) != HAL_OK)
				{
					printf("DMA传输失败");
				}
				xSemaphoreTake(xWaitDMASem, portMAX_DELAY);
		
        send_index += send_bytes;
    }
		printf("packet send finish...\r\n");

    // 第三步：发送帧尾（2字节）
    uint8_t frame_tail[2] = {(FRAME_TAIL >> 8) & 0xFF, FRAME_TAIL & 0xFF};
		printf("send the frame tail...\r\n");
    
		if(HAL_UART_Transmit_DMA(&huart2, frame_tail, 2) != HAL_OK)
		{
			printf("DMA传输失败");
		}
		xSemaphoreTake(xWaitDMASem, portMAX_DELAY);
		printf("传输完成！");
}
