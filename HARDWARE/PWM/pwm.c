#include "pwm.h"
#include "led.h"
#include <stdio.h>
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//PWM  驱动代码			   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/12/03
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
#define TIM2_CCR1_Address 0x40000034
	
//#define TIMING_ONE  75
//#define TIMING_ZERO 20
uint16_t LED_BYTE_Buffer[600];
uint8_t rgb[48]={0};




#define TIMING_ONE  80	// 0.8us电平时间
#define TIMING_ZERO 30	// 0.3us电平时间

u16 dmaBuffer0[24];
u16 dmaBuffer1[24];



static int current_led = 0;
static int total_led = 0;
static u8(*color_led)[3] = NULL;


void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(  //使能或者失能指定的TIM中断
		TIM2, //TIM2
		TIM_IT_Update ,
		ENABLE  //使能
		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
	
}


//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	//GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); //Timer3部分重映射  TIM3_CH2->PB5    
 
   //设置该引脚为复用输出功能,输出TIM3 CH2的PWM脉冲波形	GPIOB.5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TIM_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
 
   //初始化TIM3
	TIM_TimeBaseStructure.TIM_Period = 89; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器 ------------------?????
//	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel2);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM3->CCR3;//TIM2->CCR2;//TIM2_CCR1_Address;	// physical address of Timer 3 CCR1 TIM3->CCR3
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)LED_BYTE_Buffer;		// this is the buffer memory 
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;						// data shifted from memory to peripheral
	DMA_InitStructure.DMA_BufferSize = 235;//42
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					// automatically increase buffer index
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode =DMA_Mode_Normal;							// stop DMA feed after buffer size is reached
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* TIM3 CC1 DMA Request enable */
	TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE);
	DMA_ITConfig(DMA1_Channel2,DMA_IT_TC,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

void WS2812_send(uint8_t (*color)[3], uint16_t len)
{
	uint8_t i;
	uint16_t memaddr;
	uint16_t buffersize;
	buffersize = (len*24)+43+146;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	
		while(memaddr < 146)
		{
			LED_BYTE_Buffer[memaddr] = 0;
			memaddr++;
		}
	
	len=8;
	while (len)
	{	
		for(i=0; i<8; i++) // GREEN data
		{
			LED_BYTE_Buffer[memaddr] = ((color[0][0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // RED
		{
				LED_BYTE_Buffer[memaddr] = ((color[0][1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr++;
		}
		for(i=0; i<8; i++) // BLUE
		{
				LED_BYTE_Buffer[memaddr] = ((color[0][2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr++;
		}
		len--;
	}
	
	len=8;
	while (len)
	{	
		for(i=0; i<8; i++) // GREEN data
		{
			LED_BYTE_Buffer[memaddr] = ((color[1][0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // RED
		{
				LED_BYTE_Buffer[memaddr] = ((color[1][1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr++;
		}
		for(i=0; i<8; i++) // BLUE
		{
				LED_BYTE_Buffer[memaddr] = ((color[1][2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr++;
		}
		len--;
	}

  	LED_BYTE_Buffer[memaddr] = ((color[0][2]<<8) & 0x0080) ? TIMING_ONE:TIMING_ZERO;

	  memaddr++;	
		while(memaddr < buffersize)
		{
			LED_BYTE_Buffer[memaddr] = 0;
			memaddr++;
		}
//		DMA_Cmd(DMA1_Channel2, DISABLE);
//		DMA_SetCurrDataCounter(DMA1_Channel2, buffersize);
//		DMA_Cmd(DMA1_Channel2, ENABLE);
    //LED_BYTE_Buffer[0]=30;
	DMA_SetCurrDataCounter(DMA1_Channel2, buffersize); 	// load number of bytes to be transferred
  	DMA_Cmd(DMA1_Channel2, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	//	DMA_Cmd(DMA1_Channel2, ENABLE);
	//	while(!DMA_GetFlagStatus(DMA1_FLAG_TC2)) ; 	// wait until transfer complete
//		TIM_Cmd(TIM2, DISABLE); 	// disable Timer 3
//		DMA_Cmd(DMA1_Channel2, DISABLE); 			// disable DMA channel 6
//		DMA_ClearFlag(DMA1_FLAG_TC2); 				// clear DMA1 Channel 6 transfer complete flag
}


void WS2812_n_send(uint8_t* color, uint16_t len)
{
	uint8_t i;
	uint16_t memaddr;
	uint16_t buffersize;
	buffersize = (len*24)+43+148;	// number of bytes needed is #LEDs * 24 bytes + 42 trailing bytes
	memaddr = 0;				// reset buffer memory index
	
	while(memaddr < 148)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}
	
	while (len)
	{
		for(i=0; i<8; i++) // GREEN data
		{
			LED_BYTE_Buffer[memaddr] = ((color[(16-len)*3]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
			memaddr++;
		}
		for(i=0; i<8; i++) // RED
		{
				LED_BYTE_Buffer[memaddr] = ((color[(16-len)*3+1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr++;
		}
		for(i=0; i<8; i++) // BLUE
		{
				LED_BYTE_Buffer[memaddr] = ((color[(16-len)*3+2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
				memaddr++;
		}
		len--;
	}
//===================================================================//	
//bug：最后一个周期波形不知道为什么全是高电平，故增加一个波形
  	LED_BYTE_Buffer[memaddr] = ((color[(16-len)*3+2]<<8) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
//===================================================================//	
	memaddr++;	
	while(memaddr < buffersize)
	{
		LED_BYTE_Buffer[memaddr] = 0;
		memaddr++;
	}
//		DMA_Cmd(DMA1_Channel2, DISABLE);
//		DMA_SetCurrDataCounter(DMA1_Channel2, buffersize);
//		DMA_Cmd(DMA1_Channel2, ENABLE);
    //LED_BYTE_Buffer[0]=30;
	DMA_SetCurrDataCounter(DMA1_Channel2, buffersize); 	// load number of bytes to be transferred
  	DMA_Cmd(DMA1_Channel2, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE); 						// enable Timer 3
	//	DMA_Cmd(DMA1_Channel2, ENABLE);
	//	while(!DMA_GetFlagStatus(DMA1_FLAG_TC2)) ; 	// wait until transfer complete
//		TIM_Cmd(TIM2, DISABLE); 	// disable Timer 3
//		DMA_Cmd(DMA1_Channel2, DISABLE); 			// disable DMA channel 6
//		DMA_ClearFlag(DMA1_FLAG_TC2); 				// clear DMA1 Channel 6 transfer complete flag
}


void TIM2_IRQHandler(void)   //TIM3中断
{
	uint8_t rgb[8] = {0xff};
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
		WS2812_n_send(rgb,8);//LED1=!LED1;
		}
}




//ws2812颜色填充
static void fillLed(u16 *buffer, u8 *color)
{
    int i;

    for(i=0; i<8; i++) // GREEN
	{
	    buffer[i] = ((color[1]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // RED
	{
	    buffer[8+i] = ((color[0]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
	for(i=0; i<8; i++) // BLUE
	{
	    buffer[16+i] = ((color[2]<<i) & 0x0080) ? TIMING_ONE:TIMING_ZERO;
	}
}

//DMA中断处理
void ws2812DmaIsr(void)
{
	if (total_led == 0)
	{
		TIM_Cmd(TIM3, DISABLE);
		DMA_Cmd(DMA1_Channel2, DISABLE); 
	}
	
	if (DMA_GetFlagStatus(DMA1_FLAG_TC2)==SET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC2); 
		if (current_led<total_led)
			fillLed(dmaBuffer1, color_led[current_led]);
		else
			memset(dmaBuffer1, 0, sizeof(dmaBuffer1));
		current_led++;
	}

	if (current_led >= total_led + 2) //多传输2个LED产生60us的低电平
	{
		TIM_Cmd(TIM3, DISABLE); 					
		DMA_Cmd(DMA1_Channel2, DISABLE); 
		total_led = 0;
	}
}


//ws2812颜色发送至DMA
void ws2812Send(u8 (*color)[3], u16 len)
{
	if(len<1) return;

	//xSemaphoreTake(allLedDone, portMAX_DELAY);//等待上一次发送完成

	current_led = 0;
	total_led = len;
	color_led = color;
	
	fillLed(dmaBuffer0, color_led[current_led]);
	current_led++;
	fillLed(dmaBuffer1, color_led[current_led]);
	current_led++;
	
	DMA_Cmd(DMA1_Channel2, ENABLE); 			// enable DMA channel 6
	TIM_Cmd(TIM3, ENABLE);			//使能定时器
}


void DMA1_Channel2_IRQHandler(void)
{
	ws2812DmaIsr();
}












