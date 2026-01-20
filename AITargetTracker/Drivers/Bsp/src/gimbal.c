#include "gimbal.h"

extern TIM_HandleTypeDef	htim4;

#define XKP 					0.08f
#define XKI 					0.005f
#define XKD 					0.02f
#define YKP 					0.08f
#define YKI 					0.005f
#define YKD 					0.02f
#define MAXSTEP 			5.0f
#define PIXELDEADZONE 10.0f
#define JOYDEADZONE 	150
#define JOYCENTERVAL 	2048

void Servo_Set_Angle(u8 angle, u8 CH)
{
	//0°--  0.5ms -- 50tick
	//180°-- 2.5ms -- 250tick
	u8 valid_angle = (angle > 180) ? 180 : angle;
	
	uint16_t tick = ((float)valid_angle / 180) * 200 + 50;
	if(CH == 1)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, tick);
	}else if(CH == 2)
	{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, tick);
	}else
	{
		printf("errCH");
	}
	
	
}

// 系统初始化时调用
void Gimbal_Init(PID_TypeDef* xPidx, PID_TypeDef* xPidy, Joystick_TypeDef* xJoy, Gimbal_TypeDef* xGim) {
    // 初始化 PID
    // 参数需要根据实际调节: Kp, Ki, Kd, MaxStep(单次最大转角), PixelDeadZone(像素死区)
    // X轴通常需要响应快一点
    PID_Init(xPidx, XKP, XKI, XKD, MAXSTEP, PIXELDEADZONE); 
    // Y轴受重力影响，参数可能略有不同
    PID_Init(xPidy, YKP, YKI, YKD, MAXSTEP, PIXELDEADZONE); 

		// 参数需要根据实际调节: CenterVal(中间值), MaxStep(单次最大步长), JoyDeadZone(舵机死区)
		// 摇杆结构体的初始化
		Joystick_Init(xJoy, JOYCENTERVAL, MAXSTEP, JOYDEADZONE);
	
    // 初始化云台状态 (假设初始在中间 90度)
    xGim->current_x_angle = 90.0f;
    xGim->current_y_angle = 90.0f;
    xGim->servo_min_angle = 0.0f;
    xGim->servo_max_angle = 180.0f;
    
    Servo_Set_Angle(xGim->current_x_angle, 1);
    Servo_Set_Angle(xGim->current_y_angle, 2);
}

void Control_Loop(Gimbal_TypeDef* xGim, float* pfDelte) {
	
    // 注意方向：摄像头的镜像关系决定是 加 还是 减
    // 如果人脸在画面左边(坐标小)，云台应该往左转还是往右转？
    // 假设：X坐标小意味着目标在左侧，需要减少角度向左看
    xGim->current_x_angle += pfDelte[0]; // 或者 -= delta_x，根据实际安装方向调整
    
    // Y轴方向调整
    xGim->current_y_angle += pfDelte[1]; // 或者 -= delta_y

    /* ---------------- 物理限位保护 ---------------- */
    if (xGim->current_x_angle > xGim->servo_max_angle) xGim->current_x_angle = xGim->servo_max_angle;
    if (xGim->current_x_angle < xGim->servo_min_angle) xGim->current_x_angle = xGim->servo_min_angle;

    if (xGim->current_y_angle > xGim->servo_max_angle) xGim->current_y_angle = xGim->servo_max_angle;
    if (xGim->current_y_angle < xGim->servo_min_angle) xGim->current_y_angle = xGim->servo_min_angle;

    /* ---------------- 执行动作 ---------------- */
    Servo_Set_Angle(xGim->current_x_angle, 1);
    Servo_Set_Angle(xGim->current_y_angle, 2);
		
}


