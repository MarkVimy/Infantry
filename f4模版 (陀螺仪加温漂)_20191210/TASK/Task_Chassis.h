#ifndef __TASK_CHASSIS_H
#define __TASK_CHASSIS_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

#include "my_app.h"
/* Global macro --------------------------------------------------------------*/
#define CHASSIS_MODE_COUNT	PID_MODE_COUNT

#define CHASSIS_SPEED_IOUT_MAX		6000	// ���̵���ٶȻ�pid�����޷�
#define CHASSIS_ANGLE_IOUT_MAX		3000	// ���̵��λ�û�pid�����޷�
#define CHASSIS_PID_OUT_MAX			9000	// ���̵��pid������ֵ(��������)

#define WARNING_REMAIN_POWER		60
#define CHASSIS_MAX_CURRENT_LIMIT	36000	// �������������9000
/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	LEFT_FRON_201 = 0,  // ��ǰ
	RIGH_FRON_202 = 1,  // ��ǰ
	LEFT_BACK_203 = 2,  // ���
	RIGH_BACK_204 = 3,  // �Һ�
	CHASSIS_MOTOR_COUNT = 4
}Chassis_Motor_Names;

typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			Out;
}Chassis_PID_t;

typedef struct {
	float maxLimit;		// �������
	float currentLimit;	// ��ǰ����
}Chassis_Power_t;


/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Chassis_PID_t Chassis_PID[CHASSIS_MOTOR_COUNT];
extern PID_Object_t Chassis_Z_Speed_PID;

/* API functions Prototypes --------------------------------------------------*/
void Chassis_updateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void Chassis_updateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void Chassis_updateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);

void Chassis_PID_ParamsInit(Chassis_PID_t *pid, uint8_t motor_cnt);
void Chassis_Z_Speed_PID_ParamsInit(PID_Object_t *pid);
void Chassis_stop(Chassis_PID_t *pid);
void Chassis_Speed_PID_calculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
void Chassis_Angle_PID_calculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx);
float Chassis_Z_Speed_PID_calculate(PID_Object_t *pid);
void Chassis_PID_out(Chassis_PID_t *pid);

/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_init(void);
//void CHASSIS_powerLimit(Chassis_Power_t *power, Chassis_PID_t *pid, Judge_Info_t *judge_info);

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
void CHASSIS_keyControlTask(void);
void CHASSIS_selfProtect(void);	// ����ʧ�ر���
void CHASSIS_control(void);	// ���̿�������

#endif
