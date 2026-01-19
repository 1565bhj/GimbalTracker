# AITracker Project

## STM32配置

### 	1、时钟配置

​		总时钟：400MHz

​		FMC时钟：240MHz

​		

### 	2、SDRAM的配置

![image-20260106231146678](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260106231146678.png)

### 	3、DMA配置	![image-20260106233527700](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260106233527700.png)

### 	4、usart配置

![image-20260106233758882](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260106233758882.png)

### 5、舵机TIM4配置

​	![image-20260107204706278](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260107204706278.png)

两个引脚：

![image-20260107205137132](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260107205137132.png)





如何实现非阻塞防抖？

​	最佳实践：软件定时器防抖

​	![image-20260112233501383](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260112233501383.png)

Enable Software timer 然后定义句柄TimerHandle_t xDebounceTimer = NULL





![image-20260110023207047](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260110023207047.png)

会占用CPU资源！

​	合并为一个任务



在.Bak文件将自定义段（`.sdram_buffer`）分配到不同的内存区域

![image-20260111130035867](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260111130035867.png)

![image-20260111131144793](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260111131144793.png)

![image-20260111131218210](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260111131218210.png)



测试SDRAM的时候发现代码在队列断言的时候卡死，以为是SDRAM的问题，后来查看

Call Stack, 发现是另一个任务的问题





变量定义应该在.c文件，不然其他文件包含该头文件会显示重复定义



ADC-DMA传输没反应？

​	数据一致性，加volatile？不行

​	SCB_InvalidateDCache_by_Addr()刷新缓存？ 不行

![image-20260114002258655](C:\Users\26490\AppData\Roaming\Typora\typora-user-images\image-20260114002258655.png)



# HAL_ADC_Start_DMA卡死

发现是在反复DMA的中断

​	关闭ADC的DMA中断

​	



包含了头文件但是还是报错说 identifier "xxxx" is undefined?

​	循环包含头文件



```c
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
				xSemaphoreTake(xWaitDMA, portMAX_DELAY);
				printf("send the %d packet...\r\n", i++);
        HAL_UART_Transmit_DMA(&huart2, send_buf, send_bytes);
        send_index += send_bytes;
    }
```

进入HardFault_Handler中断？

​	// 错误：栈上分配大数组 → 栈溢出 // uint8_t send_buf[PACKET_SIZE];？

​	静态分配数组 or xTaskCreate(UART_Send_Task, "UART_Send", 4096, NULL, 3, NULL);
