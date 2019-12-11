#ifndef __DELAY_H
#define __DELAY_H


#include "sys.h"

//#define FA_US 	SystemCoreClock/1000000

//void delay_init(uint8_t sysclk);
//void delay_us(u32 nus);
//void delay_ms(u32 nms);

void Delay_init(u8 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u32 nms);
void delay_xms(u32 nms);

#endif

