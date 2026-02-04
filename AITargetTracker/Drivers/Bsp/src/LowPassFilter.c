#include "LowPassFilter.h"


float LowPass_Filter(LowPassFilter_TypeDef* filter, float val)
{
	// 1. 空指针保护：直接返回原始值，避免崩溃
    if (filter == NULL) {
        return val;
    }
	
	float val_last = filter->val_last;
	unsigned long timestamp = HAL_GetTick();
	float dt = ((float)(timestamp - filter->tick_last)) * 1e-3f;
	
	dt = (dt<=0)?1e-3f:dt;
 
	if(dt < 0.3f)
	{
		float alpha = filter->Tf / (filter->Tf + dt);
		val = alpha * val_last + (1 - alpha) * val;
	}
	
	filter->tick_last = timestamp;
	filter->val_last = val;
	return val;
}

