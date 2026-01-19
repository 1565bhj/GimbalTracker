#include "pid.h"

/**
 * @brief  初始化 PID 参数
 */
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_out, float dead_zone) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    
    pid->err_now = 0;
    pid->err_last = 0;
    pid->err_prev = 0;
    
    pid->out_max = max_out;
    pid->dead_zone = dead_zone;
}

/**
 * @brief  增量式 PID 计算
 * @param  pid: PID 结构体指针
 * @param  set_point: 设定目标值 (通常是屏幕中心坐标，例如 400)
 * @param  measure_point: 实际测量值 (AI 识别到的人脸中心坐标)
 * @return 舵机角度的变化量 (Delta Angle)
 */
float PID_Calculate_Incremental(PID_TypeDef *pid, float set_point, float target_point) {
    float delta_output = 0.0f;

    // 1. 计算当前误差
    pid->err_now = set_point - target_point;

    // 2. 死区检测 (防抖)
    // 如果误差非常小，认为已经对准，不需要调整，直接返回0
    if (fabsf(pid->err_now) < pid->dead_zone) {
        pid->err_now = 0; // 这里的处理可以灵活，有时为了积分项不建议清零，但增量式主要看变化
        return 0.0f;      
    }

    // 3. 增量式 PID 公式
    // P项: Kp * (e(k) - e(k-1))
    // I项: Ki * e(k)
    // D项: Kd * (e(k) - 2*e(k-1) + e(k-2))
    float p_term = pid->Kp * (pid->err_now - pid->err_last);
    float i_term = pid->Ki * pid->err_now;
    float d_term = pid->Kd * (pid->err_now - 2 * pid->err_last + pid->err_prev);

    delta_output = p_term + i_term + d_term;

    // 4. 更新历史误差
    pid->err_prev = pid->err_last;
    pid->err_last = pid->err_now;

    // 5. 输出限幅 (防止瞬间变化过大导致舵机过载或丢步)
    if (delta_output > pid->out_max) {
        delta_output = pid->out_max;
    } else if (delta_output < -pid->out_max) {
        delta_output = -pid->out_max;
    }

    return delta_output;
}

void Process_AIResult(PID_TypeDef* xPidx, PID_TypeDef* xPidy, float* pfRequire, u16* pTarget, float *pfDelta)
{
    /* ---------------- X 轴计算 (Pan) ---------------- */
    pfDelta[0] = PID_Calculate_Incremental(xPidx, pfRequire[0], (float)pTarget[0]);
		/* ---------------- Y 轴计算 (Tilt) ---------------- */
    pfDelta[1] = PID_Calculate_Incremental(xPidy, pfRequire[1], (float)pTarget[1]);
	
}





