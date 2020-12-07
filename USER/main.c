#include "led.h"
#include "delay.h"
#include "sys.h"
//#include "pwm.h"
#include "usart.h"
#include "ws2812b.h"
//ALIENTEK Mini STM32�����巶������8
//PWM���ʵ��   
//����֧�֣�www.openedv.com
//������������ӿƼ����޹�˾
#define NUM_LEDS    30
RGB_t leds[NUM_LEDS];





int main(void)
{	
	u16 led0pwmval=0;    
	u8 dir=0, i;	
	delay_init();	    	 //��ʱ������ʼ��	  
	uart_init(115200);
	ws2812b_Init();
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
//	TIM3_PWM_Init(899,0);//����Ƶ��PWMƵ��=72000/(899+1)=80Khz 
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

