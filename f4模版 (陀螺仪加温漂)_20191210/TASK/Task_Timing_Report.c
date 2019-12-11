/**
 * @file        Task_Timing_Report.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        10-October-2019
 * @brief       This file includes the Timing Report external functions 
 */
 
/* Includes ------------------------------------------------------------------*/
#include "Task_Timing_Report.h"

#include "can.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
QueueHandle_t CAN1_Queue;	// CAN1消息队列句柄
QueueHandle_t CAN2_Queue;	// CAN2消息队列句柄

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
