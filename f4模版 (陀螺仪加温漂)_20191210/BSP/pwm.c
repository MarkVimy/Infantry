/**
 * @file        pwm.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        16-October-2019
 * @brief       This file includes the PWM driver external functions 
 * 							(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 摩擦轮电机 朗宇电机
 *	  
 *	# 摩擦轮电调 小蜜蜂电机
 *
 *	# 舵机
 */

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void TIM3_GPIO_init(void);
static void TIM1_GPIO_init(void);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	摩擦轮电机GPIO配置
 *	@note	PWM1 - PA6
 *			PWM2 - PA7
 */
static void TIM3_GPIO_init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
}

/**
 *	@brief	舵机GPIO配置
 *	@note	SERVO - PE11
 */
static void TIM1_GPIO_init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	摩擦轮
 */
void TIM3_init(void)
{
	TIM_TimeBaseInitTypeDef	tim;
	TIM_OCInitTypeDef		oc;
	
	/* GPIO初始化 */
	TIM3_GPIO_init();
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	tim.TIM_Prescaler = 84-1;	// 1Mhz
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 2499;	// 25ms(每周期) +1代表+1us
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		// 输出极性低
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;	
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
	
	/* PWM1 */
	TIM_OC1Init(TIM3, &oc);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	/* PWM2 */
	TIM_OC2Init(TIM3, &oc);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	
	TIM_Cmd(TIM3, ENABLE);
	
	PWM1 = 1000;	// 解锁摩擦轮(>640)
	PWM2 = 1000;
}

/**
 *	@brief	舵机
 */
void TIM1_init(void)
{
	TIM_TimeBaseInitTypeDef	tim;
	TIM_OCInitTypeDef		oc;
	
	/* GPIO初始化 */
	TIM1_GPIO_init();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	tim.TIM_Prescaler = 3360-1;	// 50Hz
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 1000-1;	// 20ms(每周期)
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputState_Disable;
	oc.TIM_Pulse = 0;
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		// 输出极性低
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;	
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
	
	/* SERVO - TIM1->CCR2 */
	TIM_OC2Init(TIM1, &oc);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_Cmd(TIM1, ENABLE);
	
	SERVO = 0;
}

/**
 *	@brief	摩擦轮PWM输出函数
 *	@note	左右都是正,以后接线的时候注意三条线的接法	
 */
void FRICTION_pwmOut(int16_t pwm1, int16_t pwm2)
{
	PWM1 = pwm1+1000;
	PWM2 = pwm2+1000;
}

/**
 *	@brief	舵机PWM输出函数
 *	@note	
 */
void MAGZINE_pwmOut(int16_t pwm)
{
	SERVO = pwm;
}
