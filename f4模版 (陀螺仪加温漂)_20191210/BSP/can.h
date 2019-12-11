#ifndef __CAN_H
#define __CAN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_can.h"
#include "my_task.h"

/* Global macro --------------------------------------------------------------*/
/* Global TypeDef ------------------------------------------------------------*/
typedef struct{
	uint8_t temperature;
	int16_t speed;
	int16_t angle;
	int16_t current;
	int16_t angle_prev;
	int32_t angle_sum;
}MOTOR_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern MOTOR_Info_t g_Chassis_Motor_Info[CHASSIS_MOTOR_COUNT];
extern MOTOR_Info_t g_Gimbal_Motor_Info[GIMBAL_MOTOR_COUNT];
extern MOTOR_Info_t g_Revolver_Motor_Info;

/* API functions Prototypes --------------------------------------------------*/
void CAN_ParamsInit(CAN_InitTypeDef* CAN_InitStructure);
void CAN_Filter_ParamsInit(CAN_FilterInitTypeDef* CAN_FilterInitStructure);
void CAN1_init(void);
void CAN2_init(void);
void CAN1_send(uint32_t stdID, float *dat);
void CAN2_send(uint32_t stdID, float *dat);

#endif
