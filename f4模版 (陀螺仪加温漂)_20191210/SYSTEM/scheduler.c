#include "scheduler.h"

/* ## Global variables ## ------------------------------------------------------*/
Flag_t	Flag = {
	/* Remote */
	.remote.FLAG_rcLost = 0,
	.remote.FLAG_mode = RC,
	/* Chassis */
	.chassis.FLAG_pidStart = 0,
	.chassis.FLAG_angleTurnOk = 0,
	/* Gimbal */
	.gimbal.FLAG_pidStart = 0,
	.gimbal.FLAG_pidMode = MECH,	// Ĭ���ǻ�еģʽ
	.gimbal.FLAG_resetOK = false,
	.gimbal.FLAG_angleRecordStart = 0,
	.gimbal.FLAG_angleTurnOk = 0,
	/* Revolver */
	.revolver.FLAG_pidStart = 0,
	/* Friction */
	.friction.FLAG_resetOK = false,
};

Cnt_t	Cnt = {
	/* System */
	.system.CNT_err = 0,
	.system.CNT_reset = 2,	// # ע������ĳ�ʼֵҪ��BM_systemResetֵ��������ͬ
	/* Remote */
	.remote.CNT_rcLost = 0,
	.remote.CNT_rcLostRecover = 0,
	/* Chassis */
	.chassis.CNT_angleTurnOk = 0,
	/* Gimbal */
	.gimbal.CNT_resetOK = 0,
	.gimbal.CNT_enableChangePidMode = 0,
	.gimbal.CNT_angleTurnOk = 0,
};

BitMask_t	BitMask = {
	/* System */
	.system.BM_systemReset = BM_SYSTEM_RESET_GIMBAL | BM_SYSTEM_RESET_FRIC,
	/* Chassis */
	.chassis.BM_rxReport = 0,
	/* Gimbal */
	.gimbal.BM_rxReport = 0,
	/* Revolver */
	.revolver.BM_rxReport = 0,
};

Mpu_Info_t Mpu_Info = {
	.pitch = 0,
	.roll = 0,
	.yaw = 0,
	.ratePitch = 0,
	.rateRoll = 0,
	.rateYaw = 0,
	.pitchOffset = 0,
	.rollOffset = 0,
	.yawOffset = 0,
	.ratePitchOffset = 0,
	.rateRollOffset = 0,
	.rateYawOffset = 0,
};

/* # ϵͳ״̬ # */
System_State_t System_State = SYSTEM_STATE_RCLOST;	// �ϵ��ʱ��Ϊң�ض�ʧ״̬

/* ## Task Manangement Table ## ------------------------------------------------*/
//--- Start Task ---//
// Defined in the init.c file => #define START_TASK_PRIO	1
//--- System State Task ---//
#define SYSTEM_STATE_TASK_PRIO				2		// �������ȼ�
#define SYSTEM_STATE_STK_SIZE				128		// �����ջ��С
TaskHandle_t SystemStateTask_Handler;				// ������
void system_state_task(void *p_arg);
//--- Chassis Task ---//
#define CHASSIS_TASK_PRIO					3		// �������ȼ�
#define CHASSIS_STK_SIZE					256		// �����ջ��С
TaskHandle_t ChassisTask_Handler;					// ������
void chassis_task(void *p_arg);
//--- Gimbal Task ---//
#define GIMBAL_TASK_PRIO					4		// �������ȼ�
#define GIMBAL_STK_SIZE						256		// �����ջ��С
TaskHandle_t GimbalTask_Handler;					// ������
void gimbal_task(void *p_arg);
//--- Revolver Task ---//
#define REVOLVER_TASK_PRIO					5		// �������ȼ�
#define REVOLVER_STK_SIZE					256		// �����ջ��С
TaskHandle_t RevolverTask_Handler;					// ������
void revolver_task(void *p_arg);
//--- Friction Task ---//
#define FRICTION_TASK_PRIO					6		// �������ȼ�
#define FRICTION_STK_SIZE					128		// �����ջ��С
TaskHandle_t FrictionTask_Handler;					// ������
void friction_task(void *p_arg);
//--- Remote Task ---//
#define REMOTE_TASK_PRIO					7		// �������ȼ�
#define REMOTE_STK_SIZE						256		// �����ջ��С
TaskHandle_t RemoteTask_Handler;					// ������
void remote_task(void *p_arg);
//--- Vision Task ---//
#define VISION_TASK_PRIO					8		// �������ȼ�
#define VISION_STK_SIZE						256		// �����ջ��С
TaskHandle_t VisionTask_Handler;					// ������
void vision_task(void *p_arg);

/* ## Semphore Manangement Table ## --------------------------------------------*/

/* ## Task List ## -------------------------------------------------------------*/
/*!# Start Task #!*/
extern TaskHandle_t StartTask_Handler;				// ������(��init.c�ж���)
void start_task(void *pvParameters)
{
	taskENTER_CRITICAL();	// �����ٽ���
	/* ����ϵͳ״̬������ */
	xTaskCreate((TaskFunction_t		)system_state_task,						// ������
							(const char*		)"system_state_task",		// ��������
							(uint16_t			)SYSTEM_STATE_STK_SIZE,		// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)SYSTEM_STATE_TASK_PRIO,	// �������ȼ�
							(TaskHandle_t*		)&SystemStateTask_Handler);	// ������
	/* ������������ */
	xTaskCreate((TaskFunction_t		)chassis_task,							// ������
							(const char*		)"chassis_task",			// ��������
							(uint16_t			)CHASSIS_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)CHASSIS_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&ChassisTask_Handler);		// ������
	/* ������̨���� */
	xTaskCreate((TaskFunction_t		)gimbal_task,							// ������
							(const char*		)"gimbal_task",				// ��������
							(uint16_t			)GIMBAL_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)GIMBAL_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&GimbalTask_Handler);		// ������							
//	/* �������̵������ */
//	xTaskCreate((TaskFunction_t		)revolver_task,							// ������
//							(const char*		)"revovler_task",			// ��������
//							(uint16_t			)REVOLVER_STK_SIZE,			// �����ջ��С
//							(void*				)NULL,						// ���ݸ��������Ĳ���
//							(UBaseType_t		)REVOLVER_TASK_PRIO,		// �������ȼ�
//							(TaskHandle_t*		)&RevolverTask_Handler);	// ������							
//	/* ����Ħ�������� */
//	xTaskCreate((TaskFunction_t		)friction_task,							// ������
//							(const char*		)"friction_task",			// ��������
//							(uint16_t			)FRICTION_STK_SIZE,			// �����ջ��С
//							(void*				)NULL,						// ���ݸ��������Ĳ���
//							(UBaseType_t		)FRICTION_TASK_PRIO,		// �������ȼ�
//							(TaskHandle_t*		)&FrictionTask_Handler);	// ������
	/* ����ң������ */
	xTaskCreate((TaskFunction_t		)remote_task,							// ������
							(const char*		)"remote_task",				// ��������
							(uint16_t			)REMOTE_STK_SIZE,			// �����ջ��С
							(void*				)NULL,						// ���ݸ��������Ĳ���
							(UBaseType_t		)REMOTE_TASK_PRIO,			// �������ȼ�
							(TaskHandle_t*		)&RemoteTask_Handler);		// ������
	/* �����Ӿ����� */
//	xTaskCreate((TaskFunction_t		)vision_task,							// ������
//							(const char*		)"vision_task",				// ��������
//							(uint16_t			)VISION_STK_SIZE,			// �����ջ��С
//							(void*				)NULL,						// ���ݸ��������Ĳ���
//							(UBaseType_t		)VISION_TASK_PRIO,			// �������ȼ�
//							(TaskHandle_t*		)&VisionTask_Handler);		// ������
							
	vTaskDelete(StartTask_Handler);	
	taskEXIT_CRITICAL();	// �˳��ٽ���
}

/**	!# 1 - System State Task #!
 *	@note 
 *		Loop time:	2ms
 */
void system_state_task(void *p_arg)	// ϵͳ״̬��
{
	portTickType currentTime;
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		GIMBAL_IMU_recordFeedback(Gimbal_PID);
		/* ϵͳ״̬�� */
		switch(System_State) {
			case SYSTEM_STATE_NORMAL:	LED_ALL_OFF();LED_GREEN_ON();break;
			case SYSTEM_STATE_RCERR: 
			case SYSTEM_STATE_RCLOST: 	LED_ALL_ON();break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//������ʱ
	}
}

/**	!# 2 - Chassis Task #!
 *	@note 
 *		Loop time:	2ms
 */
void chassis_task(void *p_arg)
{
	portTickType currentTime;
	CHASSIS_init();
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		switch(System_State)
		{												
			case SYSTEM_STATE_NORMAL:					
				if(Flag.chassis.FLAG_pidStart == 1) {
					CHASSIS_control();
				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				CHASSIS_selfProtect();
				break;
		}
//		vTaskDelay(2);	// 2ms
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}

/**	!# 3 - Gimbal Task #!
 *	@note 
 *		Loop time:	2ms
 */
void gimbal_task(void *p_arg)
{
	portTickType currentTime;
	GIMBAL_init();
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		//GIMBAL_IMU_recordFeedback(Gimbal_PID);
		switch(System_State)
		{
			case SYSTEM_STATE_NORMAL:
//				if(Flag.gimbal.FLAG_pidStart == 1) {
					GIMBAL_control();
//				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				GIMBAL_selfProtect();
				break;
		}
//		vTaskDelay(2);	// 2ms
		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}

/**	!# 4 - Revolver Task #!
 *	@note 
 *		Loop time:	5ms
 */
void revolver_task(void *p_arg)
{
	portTickType currentTime;
	while(1) {
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		switch(System_State)
		{
			case SYSTEM_STATE_NORMAL:
				if(Flag.revolver.FLAG_pidStart == 1) {
					REVOLVER_control();
				}
				break;
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
				REVOLVER_selfProtect();
				break;
		}
		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//������ʱ
	}
}

/**	!# 5 - Friction Task #!
 *	@note 
 *		Loop time:	10ms
 */
void friction_task(void *p_arg)
{
	//FRICTION_pwmOut(1500, 1500);
//	PWM1 = 1500;
//	PWM2 = 1500;
//	delay_ms(1000);
//	delay_ms(1000);
//	delay_ms(1000);
//	delay_ms(1000);
//	delay_ms(1000);
//	delay_ms(1000);
//	delay_ms(1000);
//	delay_ms(1000);
//	PWM1 = 0;
//	PWM2 = 0;
	
	while(1) {		
		switch(System_State) 
		{
			case SYSTEM_STATE_NORMAL:
			{
				FRICTION_control();
				MAGZINE_control();
				break;
			}
			case SYSTEM_STATE_RCERR:
			case SYSTEM_STATE_RCLOST:
			{
				FRICTION_selfProtect();
				MAGZINE_selfProtect();
				break;
			}
		}
		vTaskDelay(10);	// 10ms
	}
}

/**	!# 6 - Remote Task #!
 *	@note 
 *		Loop time:	20ms
 */
void remote_task(void *p_arg)
{		
	while(1) {
		REMOTE_control();
		vTaskDelay(20);	// 20ms
	}
}

/**	!# 7 - Vision Task #!
 *	@note 
 *		Loop time:	100ms
 */
void vision_task(void *p_arg)
{
	VISION_init();
	while(1) {
		VISION_control();
		vTaskDelay(10);	// 10ms
	}
}
