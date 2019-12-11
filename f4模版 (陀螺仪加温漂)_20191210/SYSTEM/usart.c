/**
 * @file        usart.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        26-March-2019
 * @brief       This file includes the USART driver external functions 
 * 							(based on ST Peripherals Libaray F10x V3.5)
 *
 * @Verison			V1.1 (5-June-2019)
 *								- Add USART DMA(receive random length)
 *							V1.2 (15-June-2019)
 *								- Add USARTx send
 *							V1.3 (3-August-2019)
 *								- Finished USART1~3 API functions
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

 //////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
#define DEBUG

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
//#ifdef DEBUG
//	while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
//    USART3->DR = (uint8_t) ch;      
//	return ch;
//#endif
	return ch;
}

#endif 





