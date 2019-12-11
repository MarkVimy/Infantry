#ifndef	__TASK_TIMING_REPORT_H
#define	__TASK_TIMING_REPORT_H

#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

extern QueueHandle_t CAN1_Queue;	// CAN1消息队列句柄
extern QueueHandle_t CAN2_Queue;	// CAN2消息队列句柄

#endif

