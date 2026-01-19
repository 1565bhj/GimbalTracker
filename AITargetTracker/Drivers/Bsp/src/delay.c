#include "delay.h"
//#include "sys.h"

#define SYS_CLK 400
 
/* 
//roughly delay
void delay_us(uint32_t nus)
{		
	uint32_t i;
	uint16_t temp;
	temp = nus*SYS_CLK/15;
	for(i=0;i<temp;i++);
}

//延时nms
//nms:要延时的ms数
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}
*/

void delay_init(void)
{
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	/* Configure the SysTick to have interrupt in 1ms time basis*/
	HAL_SYSTICK_Config(SystemCoreClock / (1000U / uwTickFreq));
}

#if OS_SUPPORT 						    								   
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				   	 
	ticks=nus*SYS_CLK; 						 
	delay_osschedlock();					
	told=SysTick->VAL;        				
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;//这里注意一下SYSTICK是一个递减的计数器就可以了.	
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			
		}  
	};
	delay_osschedunlock();															    
}  

void delay_ms(uint16_t nms)
{	
	if(delay_osrunning&&delay_osintnesting==0)    
	{		 
		if(nms>=fac_ms)						
		{ 
   			delay_ostimedly(nms/fac_ms);	
		}
		nms%=fac_ms;						 
	}
	delay_us((uint32_t)(nms*1000));			
}
#else  
	 
void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;				    	 
	ticks=nus*SYS_CLK; 						
	told=SysTick->VAL;        				
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			
		}  
	}
}

void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}
#endif
