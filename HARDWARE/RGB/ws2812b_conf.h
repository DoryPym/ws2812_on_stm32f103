#ifndef __WS2812B_CONF_H
#define __WS2812B_CONF_H

#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dma.h>
#include <misc.h>

#define WS2812B_USE_GAMMA_CORRECTION
#define WS2812B_USE_PRECALCULATED_GAMMA_TABLE

#define WS2812B_BUFFER_SIZE     60
#define WS2812B_START_SIZE      2

#define WS2812B_APB1_RCC        RCC_APB1Periph_TIM3
#define WS2812B_APB2_RCC        RCC_APB2Periph_GPIOB

#define WS2812B_AHB_RCC         RCC_AHBPeriph_DMA1

#define WS2812B_GPIO            GPIOB
#define WS2812B_GPIO_PIN        GPIO_Pin_0

#define WS2812B_TIM             TIM3
#define WS2812B_TIM_OCINIT      TIM_OC3Init
#define WS2812B_TIM_OCPRELOAD   TIM_OC3PreloadConfig
#define WS2812B_TIM_DMA_CC      TIM_DMA_CC3
#define WS2812B_TIM_DMA_CCR     (WS2812B_TIM->CCR3)

#define WS2812B_DMA             DMA1
#define WS2812B_DMA_CHANNEL     DMA1_Channel2
#define WS2812B_DMA_IRQ         DMA1_Channel2_IRQn

#define WS2812B_DMA_HANDLER     DMA1_Channel2_IRQHandler
#define WS2812B_DMA_IT_TC       DMA1_IT_TC2
#define WS2812B_DMA_IT_HT       DMA1_IT_HT2

#define WS2812B_IRQ_PRIO        0
#define WS2812B_IRQ_SUBPRIO     0

#define WS2812B_FREQUENCY       24000000
#define WS2812B_PERIOD          30

#define WS2812B_PULSE_HIGH      21
#define WS2812B_PULSE_LOW       9

#endif //__WS2812B_CONF_H
