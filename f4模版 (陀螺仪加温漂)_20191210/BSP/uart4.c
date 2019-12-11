/**
 * @file        uart4.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        27-October-2019
 * @brief       This file includes the UART4 driver external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 视觉通信使用USART4作为通讯接口
 */

/* UART5_RX  ----> DMA1 Ch4 Stream0 */
/* UART4_RX  ----> DMA1 Ch4 Stream2 */
/* USART3_RX ----> DMA1 Ch4 Stream1 */
/* USART2_RX ----> DMA1 Ch4 Stream5 */

/* Includes ------------------------------------------------------------------*/
#include "uart4.h"

#include "string.h"
#include "vision.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* TX */
#define    GPIO_TX                   GPIOA
#define    GPIO_PIN_TX               GPIO_Pin_0
#define    GPIO_PINSOURCE_TX         GPIO_PinSource0
#define    RCC_AHB1PERIPH_GPIO_TX    RCC_AHB1Periph_GPIOA

/* RX */
#define    GPIO_RX                   GPIOA
#define    GPIO_PIN_RX               GPIO_Pin_1
#define    GPIO_PINSOURCE_RX         GPIO_PinSource1
#define    RCC_AHB1PERIPH_GPIO_RX    RCC_AHB1Periph_GPIOA

/* DMA */
#define    DMA1_Stream_RX            DMA1_Stream2

/* Buffer Len */
#define    VISION_BUFFER_LEN         100

/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
//裁判系统发过来的数据暂存在这里
uint8_t  Vision_Buffer[ VISION_BUFFER_LEN ] = {0};

/* Private function prototypes -----------------------------------------------*/
static void UART4_DMA_init( void );

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief 串口4 DMA初始化
 */
static void UART4_DMA_init( void )
{		
	DMA_InitTypeDef xCom4DMAInit;
	
	DMA_DeInit( DMA1_Stream_RX );
	xCom4DMAInit.DMA_Channel = DMA_Channel_4;

	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;//方向外设到存储器

	xCom4DMAInit.DMA_PeripheralBaseAddr  = (uint32_t)&(UART4->DR);
	xCom4DMAInit.DMA_Memory0BaseAddr     = (uint32_t)Vision_Buffer;
	xCom4DMAInit.DMA_DIR = DMA_DIR_PeripheralToMemory;
	xCom4DMAInit.DMA_BufferSize = VISION_BUFFER_LEN;
	xCom4DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	xCom4DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable;
	xCom4DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	xCom4DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	xCom4DMAInit.DMA_Mode = DMA_Mode_Circular;
	xCom4DMAInit.DMA_Priority = DMA_Priority_VeryHigh;
	xCom4DMAInit.DMA_FIFOMode = DMA_FIFOMode_Enable;
	xCom4DMAInit.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	xCom4DMAInit.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	xCom4DMAInit.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init( DMA1_Stream_RX, &xCom4DMAInit );	
	DMA_Cmd( DMA1_Stream_RX, ENABLE);  // stream2
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	裁判系统串口初始化(UART4)
 */
void UART4_init( void )
{
	USART_InitTypeDef  xUsartInit;
	GPIO_InitTypeDef   xGpioInit;
	NVIC_InitTypeDef   xNvicInit;

	RCC_AHB1PeriphClockCmd( RCC_AHB1PERIPH_GPIO_TX | RCC_AHB1PERIPH_GPIO_RX, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4, ENABLE );

	GPIO_PinAFConfig( GPIO_TX, GPIO_PINSOURCE_TX, GPIO_AF_UART4 );
	GPIO_PinAFConfig( GPIO_RX, GPIO_PINSOURCE_RX, GPIO_AF_UART4 ); 

	xGpioInit.GPIO_Pin   = GPIO_PIN_TX;
	xGpioInit.GPIO_Mode  = GPIO_Mode_AF;
	xGpioInit.GPIO_OType = GPIO_OType_PP;
	xGpioInit.GPIO_Speed = GPIO_Speed_100MHz;
	xGpioInit.GPIO_PuPd  = GPIO_PuPd_UP;

	GPIO_Init( GPIO_TX, &xGpioInit );

	xGpioInit.GPIO_Pin = GPIO_PIN_RX;
	GPIO_Init( GPIO_RX, &xGpioInit );

	xUsartInit.USART_BaudRate            = 115200;   
	xUsartInit.USART_WordLength          = USART_WordLength_8b;
	xUsartInit.USART_StopBits            = USART_StopBits_1;
	xUsartInit.USART_Parity              = USART_Parity_No;
	xUsartInit.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
	xUsartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	
	USART_Init( UART4, &xUsartInit );
	USART_Cmd( UART4, ENABLE );
	
	USART_ITConfig( UART4, USART_IT_IDLE, ENABLE  ); // 注意要配置成串口空闲中断 

	USART_DMACmd( UART4, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART4, USART_DMAReq_Tx, ENABLE );
	
	UART4_DMA_init( );	// 初始化uart4的DMA
	
	xNvicInit.NVIC_IRQChannel                    = UART4_IRQn;
	xNvicInit.NVIC_IRQChannelPreemptionPriority  = UART4_IT_PRIO_PRE;
	xNvicInit.NVIC_IRQChannelSubPriority         = UART4_IT_PRIO_SUB;
	xNvicInit.NVIC_IRQChannelCmd                 = ENABLE;
	NVIC_Init( &xNvicInit );
}

/**
 *	@brief	串口5中断函数
 */
void UART4_IRQHandler( void )
{
	uint8_t res;
	/* 空闲中断 */
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{		
		//根据ST官方手册,读一下SR和DR寄存器,IDLE才能再次使用,否则会一直进入中断,就会跟串口接收中断没区别
		res = UART4->SR ;
		res = UART4->DR ;
		
		DMA_Cmd(DMA1_Stream_RX, DISABLE);
		
		res = VISION_BUFFER_LEN - DMA_GetCurrDataCounter(DMA1_Stream_RX);
		
		VISION_readData(Vision_Buffer);		// 读取裁判系统数据
		memset(Vision_Buffer, 0, VISION_BUFFER_LEN);	// 读完之后内容清零
		DMA_Cmd(DMA1_Stream_RX, ENABLE);
	}
}


/**
  * @brief  串口一次发送一个字节数据
  * @param  自己打包好的要发给裁判的数据
  * @retval void
  * @attention  串口移位发送
  */
void UART4_sendChar(uint8_t cData)
{
	while (USART_GetFlagStatus( UART4, USART_FLAG_TC ) == RESET);
	
	USART_SendData( UART4, cData );   
}
