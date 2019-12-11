/*
 * @file        exti.c
 * @author      MarkVimy
 * @Version     V1.0
 * @date        27-March-2019
 * @brief       This file includes the EXTI driver external functions 
 * 				(based on ST Peripherals Libaray F10x V3.5)
 */

/* Includes ------------------------------------------------------------------*/
#include "exti.h"

#include "delay.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static EXTI_InitTypeDef EXTI_DefaultParams = 
{
	.EXTI_Line = EXTI_Line2,
	.EXTI_Mode = EXTI_Mode_Interrupt,
	.EXTI_Trigger = EXTI_Trigger_Falling,
	.EXTI_LineCmd = ENABLE,
};

/* ## Global variables ## ----------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	外部中断参数默认配置初始化
 *	@param	(EXTI_InitTypeDef*)EXTI_InitStructure
 */
void EXTI_ParamsInit(EXTI_InitTypeDef* EXTI_InitStructure)
{
	EXTI_InitStructure->EXTI_Line = EXTI_DefaultParams.EXTI_Line;
	EXTI_InitStructure->EXTI_Mode = EXTI_DefaultParams.EXTI_Mode;
	EXTI_InitStructure->EXTI_Trigger = EXTI_DefaultParams.EXTI_Trigger;
	EXTI_InitStructure->EXTI_LineCmd = EXTI_DefaultParams.EXTI_LineCmd;
}

/**
 *	@brief	嵌套向量中断控制器关于特定 IRQ_Channel 的初始化
 *	@param	uint8_t NVIC_IRQChannel
 *			uint8_t NVIC_IRQChannelPreemptionPriority
 *			uint8_t NVIC_IRQChannelSubPriority
 */
void NVICx_init(uint8_t NVIC_IRQChannel, uint8_t NVIC_IRQChannelPreemptionPriority, uint8_t NVIC_IRQChannelSubPriority)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_IRQChannelPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_IRQChannelSubPriority;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

/**
 *	@brief	外部中断X初始化
 */
void EXTIX_init(void)
{
//	EXTI_InitTypeDef EXTI_InitStructure;
//	
//	/* 使能复用功能时钟 */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	
	/* GPIOE.3 中断线 及 中断初始化配置 */
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);	// PE3 - KEY1
//	EXTI_ParamsInit(&EXTI_InitStructure);
//	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		// 上升沿触发
//	EXTI_Init(&EXTI_InitStructure);
	
	/* EXTI3 NVIC 配置*/
//	NVICX_init(EXTI3_IRQn, KEY_IT_PRIO_PRE, KEY_IT_PRIO_SUB);	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = KEY_IT_PRIO_PRE;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = KEY_IT_PRIO_SUB;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	/* GPIOC.1 中断线 及 中断初始化配置 */
//	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);	// PE3 - KEY1
//	EXTI_ParamsInit(&EXTI_InitStructure);
//	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		// 上升沿触发
//	EXTI_Init(&EXTI_InitStructure);
	
	/* EXTI3 NVIC 配置*/
//	NVICX_init(EXTI3_IRQn, KEY_IT_PRIO_PRE, KEY_IT_PRIO_SUB);	
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = KEY_IT_PRIO_PRE;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = KEY_IT_PRIO_SUB;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
}

///* 外部中断0 服务例程 */
//void EXTI0_IRQHandler(void)
//{
//	delay_ms(10);
//	if(WK_UP == 1) {
//		BEEP = !BEEP;
//	}
//	EXTI_ClearITPendingBit(EXTI_Line0);
//}

/* 外部中断1 服务例程 */
//void EXTI1_IRQHandler(void)
//{
//	
//}

/* 外部中断2 服务例程 */
void EXTI2_IRQHandler(void)
{
}

/* 外部中断3 服务例程 */
void EXTI3_IRQHandler(void)
{
}

/* 外部中断4 服务例程 */
void EXTI4_IRQHandler(void)
{

}
