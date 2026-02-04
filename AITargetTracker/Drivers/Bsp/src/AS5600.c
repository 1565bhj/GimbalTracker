#include "AS5600.h"


// 检测 AS5600 是否存在
uint8_t AS5600_IsConnected(AS5600_HandleTypedef* as) {
    return (HAL_I2C_IsDeviceReady(as->hi2c, AS5600_I2C_ADDR, 3, 10) == HAL_OK);
}

// 读取原始角度值（0-4095）
uint16_t AS5600_GetRawAngle(AS5600_HandleTypedef* as) {
    uint8_t rx_data[2] = {0};
    uint16_t angle = 0;
    
    // 读取 RAW ANGLE 寄存器（0x0C 和 0x0D）
    if (HAL_I2C_Mem_Read(as->hi2c, AS5600_I2C_ADDR, AS5600_REG_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, rx_data, 2, 100) != HAL_OK) {
        return 0;
    }
												 
		angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];	
		
    return angle;
}


// 获取角度（度，0-360）
float AS5600_GetDegAngle(AS5600_HandleTypedef* as) {
    uint16_t raw = AS5600_GetRawAngle(as);
    return (raw * 360.0f) / 4096.0f;
}

// 获取角度（弧度，0-2π）
float AS5600_GetRadAngle(AS5600_HandleTypedef* as) {
    uint16_t raw = AS5600_GetRawAngle(as);
    return (raw * 2.0f * _PI) / 4096.0f;
}

// 读取状态寄存器（可用于判断磁场强度）
//uint8_t AS5600_GetStatus(void) {
//    uint8_t status = 0;
//    HAL_I2C_Mem_Read(&hi2c2, AS5600_I2C_ADDR, AS5600_REG_STATUS,
//                     I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
//    return status;
//}

//void AS5600_ReadAngle_DMA(void)
//{
//    dma_complete = 0;
//    
//    // 使用DMA读取角度寄存器
//    HAL_I2C_Mem_Read_DMA(&hi2c1, 
//                         AS5600_ADDRESS << 1,  // 设备地址（左移1位）
//                         0x0C,                 // RAW ANGLE寄存器地址
//                         I2C_MEMADD_SIZE_8BIT, // 内存地址大小
//                         i2c_rx_buffer,        // 接收缓冲区
//                         2);                   // 读取2个字节
//}

/**
 * @brief  计算角速度
 */
float AS5600_GetVelocity(AS5600_HandleTypedef* as, float deltaAngle)
{
	float curTime = ((float)HAL_GetTick()) / 1000.0f;
	float dt = curTime - as->lastTime;
  float Velocity = deltaAngle / dt;
   
	//更新时间
	as->lastTime = curTime;
	return Velocity;
}

/**
 * @brief  更新角度，角速度，圈数
 * @param  ad5600结构体
 */
void AS5600_Update(AS5600_HandleTypedef* as)
{
	float radAngle = AS5600_GetRadAngle(as);
	float d_angle = radAngle - as->lastAngle;
	//更新圈数
  if(fabsf(d_angle) > (0.6f*2*_PI) ) as->rotations_full += ( d_angle > 0 ) ? -1 : 1;	
	
	float deltaAngle = (as->rotations_full - as->rotations_last)*_2PI + d_angle;
	
	float velocity = AS5600_GetVelocity(as, deltaAngle);
	
	//更新值
	as->rotations_last = as->rotations_full;
	as->lastAngle = as->curAngle;
	as->curAngle = radAngle;
	as->velocity = velocity;
}
