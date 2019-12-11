#ifndef __EXTI_H
#define __EXTI_H


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "my_include.h"

void NVICx_init(uint8_t NVIC_IRQChannel, uint8_t NVIC_IRQChannelPreemptionPriority, uint8_t NVIC_IRQChannelSubPriority);
void EXTI_ParamsInit(EXTI_InitTypeDef* EXTI_InitStructure);
void EXTIX_init(void);

#endif

