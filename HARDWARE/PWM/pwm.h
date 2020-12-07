#ifndef __PWM_H
#define __PWM_H
#include "sys.h"



void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM2_Int_Init(u16 arr,u16 psc);

void WS2812_n_send(uint8_t* color, uint16_t len);
void WS2812_send(uint8_t (*color)[3], uint16_t len);


#endif
