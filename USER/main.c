#include "led.h"
#include "delay.h"
#include "sys.h"
//#include "pwm.h"
#include "usart.h"
#include "ws2812b.h"
//ALIENTEK Mini STM32开发板范例代码8
//PWM输出实验   
//技术支持：www.openedv.com
//广州市星翼电子科技有限公司
#define NUM_LEDS    30
RGB_t leds[NUM_LEDS];





int main(void)
{	
	u16 led0pwmval=0;    
	u8 dir=0, i;	
	delay_init();	    	 //延时函数初始化	  
	uart_init(115200);
	ws2812b_Init();
	LED_Init();		  	//初始化与LED连接的硬件接口
//	TIM3_PWM_Init(899,0);//不分频。PWM频率=72000/(899+1)=80Khz 
//	TIM2_Int_Init(4999,7199);
   	while(1)
	{
		while (!ws2812b_IsReady()); 
		if(dir > 250) dir =0;
		for(i=0;i<NUM_LEDS;i++)
		{
			leds[i].r = 100;
			leds[i].g = dir;
			leds[i].b = 200;
		}
// 		delay_ms(1);	 
//		if(dir)led0pwmval++;
//		else led0pwmval--;	 
// 		if(led0pwmval>300)dir=0;
//		if(led0pwmval==0)dir=1;	   					 
//		TIM_SetCompare3(TIM3,led0pwmval);	 
		dir++;
		ws2812b_SendRGB(leds, NUM_LEDS);		
	} 
}

