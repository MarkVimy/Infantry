/**
 * @file        Task_Gimbal.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        1-October-2019
 * @brief       This file includes the Gimbal(��̨) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 */

/**
 *	 ����PID	�ο���ַ��https://www.jianshu.com/p/4b0fa85cd353
 */

/* Includes ------------------------------------------------------------------*/
#include "Task_Gimbal.h"

#include "laser.h"
#include "can.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "delay.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "remote.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define GIMBAL_NORMAL_SPEED_PID_IOUT_MAX 18000

int GIMBAL_ANGLE_PID_IOUT_MAX	= 18000;
int GIMBAL_ANGLE_PID_OUT_MAX	= 8000;
int GIMBAL_SPEED_PID_IOUT_MAX	= GIMBAL_NORMAL_SPEED_PID_IOUT_MAX;
int GIMBAL_SPEED_PID_OUT_MAX	= 500;

/*���׿�����*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

/* Private variables ---------------------------------------------------------*/
/* �������˲��� */
extKalman_t Gimbal_kalmanError[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extKalman_t Mpu_kalmanError[2];
extKalman_t Gimbal_Auto_kalmanError[GIMBAL_MOTOR_COUNT];

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//��ʼ��yaw�Ĳ���kalman����

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//��ʼ��pitch�Ĳ���kalman����

float GIMBAL_BUFF_PITCH_COMPENSATION =	0;
float GIMBAL_BUFF_YAW_COMPENSATION = -175;
float GIMBAL_BUFF_YAW_RAMP = 180;	// 185	160	185	180
float GIMBAL_BUFF_PITCH_RAMP = 140;	// 130	105	120	140

/* �����ʱ��pitch������ */
float BUFF_PITCH_MODIFY_TABLE[11] = 
{
	// �����λ�ÿ�ʼ��4.4��(��е�� ��-�� = 4472-3045 = 1067 �ֳ�11�ݣ�100Ϊ��е�ǶȲ���)Ϊ�ǶȲ���
	/* �����ʱ���� [3]90 ~ [7]80Ϊ��Ч��Χ */
	/* ͷ̧��Խ�� []�±�ֵԽ�� => MAX_UP -> [11] MAX_DOWN -> [0] */
	/* ����ֵԽ�� -> ͷѹ��Խ��(����Ϊ0��ʱ��ͷ��ƫ�ñȽϸ�) */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* ## Global variables ## ----------------------------------------------------*/

/**
 *	@pid �����ֲ���
 *	1. ����ģʽ	- ��е+������
 *	2. ����ģʽ - ������
 *	3. ���ģʽ - ��е+������
 *	4. �������ģʽ	- ��е
 */

// 34.40 46.25 10.45
/**
 *	@brief	��̨PID
 */
Gimbal_PID_t	Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT] = {
	{	// MECH - ��еģʽ
		{	// YAW(��еģʽ)
			/* �ٶȻ� */
			.Speed.kp = 34.32,		
			.Speed.ki = 38.10,		
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 90000,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* λ�û� */
			.Angle.kp = 10.51,	
			.Angle.ki = 0,			
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,	// ����
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,	
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,
		},
		{	// PITCH(��еģʽ)
			/* �ٶȻ� */
			.Speed.kp = 15.10,		
			.Speed.ki = 83.5,		
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 54000,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* λ�û� */
			.Angle.kp = 8.21,		
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,		
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,		
		},
	},	// MECH - ��еģʽ
	{	// GYRO - ������ģʽ
		{	// YAW(������ģʽ)
			/* �ٶȻ� */
			.Speed.kp = 34.5,	
			.Speed.ki = 43.5,			
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 18000,	// 18000
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* λ�û� */
			.Angle.kp = 10.72,		
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = 0,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,		
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,		
		},
		{	// PITCH(������ģʽ) - ���û�еģʽ��һ��
			/* �ٶȻ� */
			.Speed.kp = 15.10,		
			.Speed.ki = 83.5,		
			.Speed.kd = 0,	
			.Speed.target = 0,
			.Speed.feedback = 0,
			.Speed.erro = 0,
			.Speed.last_erro = 0,
			.Speed.integrate = 0,
			.Speed.integrate_max = 54000,
			.Speed.pout = 0,
			.Speed.iout = 0,
			.Speed.dout = 0,
			.Speed.out = 0,
			/* λ�û� */
			.Angle.kp = 8.21,		
			.Angle.ki = 0,
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT,
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,		
			/* б������ */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* ��� */
			.Out = 0,
		},
	}	// GYRO - ������ģʽ
};

/**
 *	@brief	��̨�ۺ���Ϣ
 */
Gimbal_Info_t 	Gimbal = {
	.State.mode = GIMBAL_MODE_NORMAL,
	
	.Auto.Yaw.kp = 1,		
	.Auto.Yaw.target = 0,
	.Auto.Yaw.erro = 0,
	
	.Auto.Pitch.kp = 1,		
	.Auto.Pitch.target = 0,
	.Auto.Pitch.erro = 0,
	
	.Buff.Yaw.kp = 1,		
	.Buff.Yaw.target = 0,
	.Buff.Yaw.erro = 0,
	
	.Buff.Pitch.kp = 1,		
	.Buff.Pitch.target = 0,
	.Buff.Pitch.erro = 0,
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	��̨���PID������ʼ��
 */
void GIMBAL_pidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt)
{
	uint8_t i;
	for(i = 0; i < motor_cnt; i++) {
		pid[i].Speed.target = 0;
		pid[i].Speed.feedback = 0,
		pid[i].Speed.erro = 0;
		pid[i].Speed.last_erro = 0;
		pid[i].Speed.integrate = 0;
		pid[i].Speed.pout = 0;
		pid[i].Speed.dout = 0;
		pid[i].Speed.iout = 0;
		pid[i].Speed.out = 0;

		if(i == YAW_205) {
			pid[i].Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;
		} else if(i == PITCH_206) {
			pid[i].Angle.target = GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT;
		}
		pid[i].Angle.feedback = 0,
		pid[i].Angle.erro = 0;
		pid[i].Angle.last_erro = 0;
		pid[i].Angle.integrate = 0;
		pid[i].Angle.pout = 0;
		pid[i].Angle.dout = 0;
		pid[i].Angle.iout = 0;
		pid[i].Angle.out = 0;
		pid[i].AngleRampTarget = 0;
		pid[i].AngleRampFeedback = 0;
		pid[i].Out = 0;
	}	
}

/**
 *	@brief	������̨��ǰģʽ����PID�������л�
 */
float auto_yaw_angle_kp = 7.f;	// 7(ramp280)
float auto_yaw_speed_kp = 31.5;	// 31.5
float auto_yaw_speed_ki = 550;	// 550

float auto_pitch_angle_kp = 10.5f;				// 10.5		10.5(ramp80)
float auto_pitch_speed_kp = 18.50f;	// 16.65	// 11.15	18.5
float auto_pitch_speed_ki = 250.f;				// 250		250

float buff_yaw_angle_kp = 6.15f;	// 6			3.5(ramp125)	3.65(ramp115)		4.5		3.75		3.85	10
float buff_yaw_speed_kp = 29.5f;	// 28.5			28.5							31.5	30.5		30.5	33.5
float buff_yaw_speed_ki = 500.f;		// 40			40								500		300			350.	500

float buff_pitch_angle_kp = 5.85f;	// 6		4.5(ramp75)		4.65(ramp70)	4.55(ramp70)	4.55		4.05	3			4.05	6
float buff_pitch_speed_kp = 13.45f;	// 13.5		13.5							13.5			13.5		13.5	16.5		13.5	13.65
float buff_pitch_speed_ki = 500.f;	// 30		30								100				60			250.	85			250.f	400

float normal_mech_yaw_angle_kp = 11.55f;	// 		10.35	10.65			11.25		11.55
float normal_mech_yaw_speed_kp = 28.5f;	// 34.32	18.75	26.5			27.5		28.5
float normal_mech_yaw_speed_ki = 200.f;	// 38.10	32.5	90				225			200

float normal_gyro_yaw_angle_kp = 11.5f;	// 9.5		9.5	6.5		7.15	7.25	7.25		8		6.5
float normal_gyro_yaw_speed_kp = 31.5f;	// 26.75	20	13.5	14.25	14		14.25	14.35		20
float normal_gyro_yaw_speed_ki = 500.f;	// 33.50	0	240		250		250		325			375		505

float normal_pitch_angle_kp = 12.f;	// 8.35			12.50			12		11.85		11.75
float normal_pitch_speed_kp = 12.85f;	// 15.10	16.65	14.25		12.85	11.45		11.15
float normal_pitch_speed_ki = 150.f;	// 39.5		55	100				150		120	

void GIMBAL_pidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	switch(gimbal->State.mode)
	{
		case GIMBAL_MODE_NORMAL:
		
			pid[MECH][YAW_205].Speed.kp = normal_mech_yaw_speed_kp;		
			pid[MECH][YAW_205].Speed.ki = normal_mech_yaw_speed_ki;		
			pid[MECH][YAW_205].Speed.kd = 0;
			pid[MECH][YAW_205].Speed.out_max = 29999;
			pid[MECH][YAW_205].Angle.kp = normal_mech_yaw_angle_kp;		
			pid[MECH][YAW_205].Angle.ki = 0;		
			pid[MECH][YAW_205].Angle.kd = 0;
			pid[MECH][YAW_205].Angle.out_max = 8000;
		
			pid[MECH][PITCH_206].Speed.kp = normal_pitch_speed_kp;		
			pid[MECH][PITCH_206].Speed.ki = normal_pitch_speed_ki;		
			pid[MECH][PITCH_206].Speed.kd = 0;
			pid[MECH][PITCH_206].Speed.out_max = 29999;
			pid[MECH][PITCH_206].Angle.kp = normal_pitch_angle_kp;		
			pid[MECH][PITCH_206].Angle.ki = 0;		
			pid[MECH][PITCH_206].Angle.kd = 0;
			pid[MECH][PITCH_206].Angle.out_max = 6000;

			pid[GYRO][YAW_205].Speed.kp = normal_gyro_yaw_speed_kp;		
			pid[GYRO][YAW_205].Speed.ki = normal_gyro_yaw_speed_ki;		
			pid[GYRO][YAW_205].Speed.kd = 0;
			pid[GYRO][YAW_205].Speed.out_max = 29999;
			pid[GYRO][YAW_205].Angle.kp = normal_gyro_yaw_angle_kp;		
			pid[GYRO][YAW_205].Angle.ki = 0;		
			pid[GYRO][YAW_205].Angle.kd = 0;
			pid[GYRO][YAW_205].Angle.out_max = 8000;
			
			pid[GYRO][PITCH_206].Speed.kp = normal_pitch_speed_kp;		
			pid[GYRO][PITCH_206].Speed.ki = normal_pitch_speed_ki;		
			pid[GYRO][PITCH_206].Speed.kd = 0;
			pid[GYRO][PITCH_206].Speed.out_max = 29999;
			pid[GYRO][PITCH_206].Angle.kp = normal_pitch_angle_kp;		
			pid[GYRO][PITCH_206].Angle.ki = 0;		
			pid[GYRO][PITCH_206].Angle.kd = 0;
			pid[GYRO][PITCH_206].Angle.out_max = 6000;

			break;
		case GIMBAL_MODE_AUTO:
			if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) // ����Ŀ��
			{
				pid[GYRO][YAW_205].Speed.kp = auto_yaw_speed_kp;		
				pid[GYRO][YAW_205].Speed.ki = auto_yaw_speed_ki;		
				pid[GYRO][YAW_205].Speed.kd = 0;
				pid[GYRO][YAW_205].Angle.kp = auto_yaw_angle_kp;		
				pid[GYRO][YAW_205].Angle.ki = 0;		
				pid[GYRO][YAW_205].Angle.kd = 0;
				
				pid[GYRO][PITCH_206].Speed.kp = auto_pitch_speed_kp;		
				pid[GYRO][PITCH_206].Speed.ki = auto_pitch_speed_ki;		
				pid[GYRO][PITCH_206].Speed.kd = 0;
				pid[GYRO][PITCH_206].Angle.kp = auto_pitch_angle_kp;		
				pid[GYRO][PITCH_206].Angle.ki = 0;		
				pid[GYRO][PITCH_206].Angle.kd = 0;
							
			}
			else	// δ����Ŀ���������ƶ���̨
			{
				pid[GYRO][YAW_205].Speed.kp = normal_gyro_yaw_speed_kp;		
				pid[GYRO][YAW_205].Speed.ki = normal_gyro_yaw_speed_ki;		
				pid[GYRO][YAW_205].Speed.kd = 0;
				pid[GYRO][YAW_205].Angle.kp = normal_gyro_yaw_angle_kp;		
				pid[GYRO][YAW_205].Angle.ki = 0;		
				pid[GYRO][YAW_205].Angle.kd = 0;

				pid[GYRO][PITCH_206].Speed.kp = normal_pitch_speed_kp;		
				pid[GYRO][PITCH_206].Speed.ki = normal_pitch_speed_ki;		
				pid[GYRO][PITCH_206].Speed.kd = 0;
				pid[GYRO][PITCH_206].Angle.kp = normal_pitch_angle_kp;		
				pid[GYRO][PITCH_206].Angle.ki = 0;		
				pid[GYRO][PITCH_206].Angle.kd = 0;			
				
			}
			
			break;
			
		case GIMBAL_MODE_BIG_BUFF:
		case GIMBAL_MODE_SMALL_BUFF:
				pid[GYRO][YAW_205].Speed.kp = buff_yaw_speed_kp;		
				pid[GYRO][YAW_205].Speed.ki = buff_yaw_speed_ki;		
				pid[GYRO][YAW_205].Speed.kd = 0;
				pid[GYRO][YAW_205].Angle.kp = buff_yaw_angle_kp;		
				pid[GYRO][YAW_205].Angle.ki = 0;		
				pid[GYRO][YAW_205].Angle.kd = 0;
				
				pid[GYRO][PITCH_206].Speed.kp = buff_pitch_speed_kp;		
				pid[GYRO][PITCH_206].Speed.ki = buff_pitch_speed_ki;		
				pid[GYRO][PITCH_206].Speed.kd = 0;
				pid[GYRO][PITCH_206].Angle.kp = buff_pitch_angle_kp;		
				pid[GYRO][PITCH_206].Angle.ki = 0;		
				pid[GYRO][PITCH_206].Angle.kd = 0;			
			break;
		
		default:
			break;
	}
}

/**
 *	@brief	��̨�������ɲ��
 */
void GIMBAL_stop(Gimbal_PID_t *pid)
{
	static float pid_out[4] = {0, 0, 0, 0};
	
	/* �ڻ��ٶȻ�������� */
	pid[YAW_205].Speed.out = 0;
	pid[YAW_205].Out = 0;
	pid[PITCH_206].Speed.out = 0;
	pid[PITCH_206].Out = 0;
	
	CAN1_send(0x1FF, pid_out);	// ��̨CAN���߱�׼��ʶ��
}

/**
 *	@brief	��̨����ٶȻ�
 */
float js_gimbal_yaw_speed_feedback;
float js_gimbal_yaw_speed_target;
float js_gimbal_yaw_speed_out;
float js_gimbal_pitch_speed_feedback;
float js_gimbal_pitch_speed_target;
float js_gimbal_pitch_speed_out;
float js_gimbal_speed_feedback;

float kLPF_speed = 0.5;
void GIMBAL_Speed_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx)
{
	static float feedback=0;
	static float last_feedback = 0;
	
	feedback = pid[YAW_205].Speed.feedback;
	js_gimbal_speed_feedback = kLPF_speed*feedback + (1-kLPF_speed)*last_feedback;
	last_feedback = js_gimbal_speed_feedback;
	
	js_gimbal_yaw_speed_feedback = pid[YAW_205].Speed.feedback;
	js_gimbal_yaw_speed_target = pid[YAW_205].Speed.target;
	js_gimbal_pitch_speed_feedback = pid[PITCH_206].Speed.feedback;
	js_gimbal_pitch_speed_target = pid[PITCH_206].Speed.target;
	
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -pid[MOTORx].Speed.integrate_max, pid[MOTORx].Speed.integrate_max);
		
	/* Pout */
	pid[MOTORx].Speed.pout = pid[MOTORx].Speed.kp * pid[MOTORx].Speed.erro;

//	if(((Gimbal.State.mode == GIMBAL_MODE_AUTO) && (VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true)))
//	{
//		if(((pid[YAW_205].Speed.integrate * pid[YAW_205].Speed.pout) < 0) && (abs(pid[YAW_205].Speed.integrate) > pid[YAW_205].Speed.integrate_max/10.f)) {
//			pid[YAW_205].Speed.integrate = 0;
//		}
//	}

//	if(((pid[YAW_205].Speed.integrate * pid[YAW_205].Speed.pout) < 0) && (abs(pid[YAW_205].Speed.integrate) > pid[YAW_205].Speed.integrate_max/25.f)) {
//		pid[YAW_205].Speed.integrate = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
//	}
	
//	if((Gimbal.State.mode == GIMBAL_MODE_SMALL_BUFF || Gimbal.State.mode == GIMBAL_MODE_BIG_BUFF) && (VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) 
//	{
//		if(((pid[YAW_205].Speed.integrate * pid[YAW_205].Speed.pout) < 0) && (abs(pid[YAW_205].Speed.integrate) > pid[YAW_205].Speed.integrate_max/10.f)) {
//			pid[YAW_205].Speed.integrate = 0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
//		}
//		if(((pid[PITCH_206].Speed.integrate * pid[PITCH_206].Speed.pout) < 0) && (abs(pid[PITCH_206].Speed.integrate) > pid[PITCH_206].Speed.integrate_max/10.f)) {
//			pid[PITCH_206].Speed.integrate = 0;
//		}		
//	}
	
	/* Iout */
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
	/* Iout Limits */
	pid[MOTORx].Speed.iout = constrain(pid[MOTORx].Speed.iout, -GIMBAL_SPEED_PID_IOUT_MAX, GIMBAL_SPEED_PID_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Speed.dout = pid[MOTORx].Speed.kd * (pid[MOTORx].Speed.erro - pid[MOTORx].Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Speed.last_erro = pid[MOTORx].Speed.erro;		
	
	/* Total PID Output*/
	pid[MOTORx].Speed.out = pid[MOTORx].Speed.pout + pid[MOTORx].Speed.iout + pid[MOTORx].Speed.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.out, -GIMBAL_SPEED_PID_OUT_MAX, GIMBAL_SPEED_PID_OUT_MAX);
	/* �ڻ��ٶȻ�������� */
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	js_gimbal_yaw_speed_out = pid[YAW_205].Speed.out;
	js_gimbal_pitch_speed_out = pid[PITCH_206].Speed.out;
}

/**
 *	@brief	��̨���λ�û�
 *	@note
 *			����PID
 *			�ڻ� ����ֵ �������ٶ�(ʵ���Ͼ����ٶȻ�)
 *					 ���ֵ �����Ǽ��ٶ�
 *
 *			�⻷ ����ֵ �����Ƕ�
 *					 ���ֵ �������ٶ�
 */
float js_gimbal_yaw_angle_feedback;
float js_gimbal_yaw_angle_target;
float js_gimbal_pitch_angle_feedback;
float js_gimbal_pitch_angle_target;
void GIMBAL_Angle_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx)
{	
	js_gimbal_yaw_angle_feedback = pid[YAW_205].Angle.feedback;
	js_gimbal_yaw_angle_target = pid[YAW_205].Angle.target;
	js_gimbal_pitch_angle_feedback = pid[PITCH_206].Angle.feedback;
	js_gimbal_pitch_angle_target = pid[PITCH_206].Angle.target;	
	
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;

	if((Flag.gimbal.FLAG_pidMode == MECH) && (MOTORx == YAW_205)) {	// ������Ч���
		if(pid[MOTORx].Angle.erro >= 4096.f) {
			pid[MOTORx].Angle.erro = +(8192.f - pid[MOTORx].Angle.erro);
		} else if(pid[MOTORx].Angle.erro <= -4096.f) {
			pid[MOTORx].Angle.erro = -(8192.f + pid[MOTORx].Angle.erro);
		}
	}	
	
	if((Flag.gimbal.FLAG_pidMode == GYRO) && (MOTORx == YAW_205)) {	// ������Ч���
		if(pid[MOTORx].Angle.erro >= 180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
			pid[MOTORx].Angle.erro = -(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - pid[MOTORx].Angle.erro);
		} else if(pid[MOTORx].Angle.erro <= -180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
			pid[MOTORx].Angle.erro = +(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + pid[MOTORx].Angle.erro);
		}
	}
	
	/* �������п������˲���������Ƶ�ȷ����� */
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {	// ����ģʽ��
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.gimbal.FLAG_pidMode][MOTORx], pid[MOTORx].Angle.erro);
	}
	if(Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.gimbal.FLAG_pidMode][MOTORx], pid[MOTORx].Angle.erro);
	}
	
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;

	/* Pout */
	pid[MOTORx].Angle.pout = pid[MOTORx].Angle.kp * pid[MOTORx].Angle.erro;
	/* Iout */
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // ģ��19��������(0.002f��2ms����˼)
	/* Iout Limits */
	pid[MOTORx].Angle.iout = constrain(pid[MOTORx].Angle.iout, -GIMBAL_ANGLE_PID_IOUT_MAX, GIMBAL_ANGLE_PID_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Angle.dout = pid[MOTORx].Angle.kd * (pid[MOTORx].Angle.erro - pid[MOTORx].Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Angle.last_erro = pid[MOTORx].Angle.erro;

	/* Total PID Output*/
	pid[MOTORx].Angle.out = pid[MOTORx].Angle.pout + pid[MOTORx].Angle.iout + pid[MOTORx].Angle.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Angle.out = constrain(pid[MOTORx].Angle.out, -GIMBAL_ANGLE_PID_OUT_MAX, GIMBAL_ANGLE_PID_OUT_MAX);
}

/**
 *	@brief	��̨���PID���������
 */
void GIMBAL_pidOut(Gimbal_PID_t *pid)
{
	float pidOut[4] = {0, 0, 0, 0};
	
	/* CAN���͵�ѹֵ */
	pidOut[YAW_205] = pid[YAW_205].Out;		// 0x205
	pidOut[PITCH_206] = pid[PITCH_206].Out;	// 0x206
	
	CAN1_send(0x1FF, pidOut);
}

/**
 *	@brief	��̨�������˲�����ʼ��
 */
void GIMBAL_kalmanCreate(void)
{
	/* �������˲�����ʼ�� */
	KalmanCreate(&Gimbal_kalmanError[MECH][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[MECH][PITCH_206], 1, 60);
	KalmanCreate(&Gimbal_kalmanError[GYRO][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[GYRO][PITCH_206], 1, 60);
	/* Mpu6050 */
	KalmanCreate(&Mpu_kalmanError[YAW_205], 1, 80);
	KalmanCreate(&Mpu_kalmanError[PITCH_206], 1, 10);	
	/* ���� */
	KalmanCreate(&Gimbal_Auto_kalmanError[YAW_205], 1, 20);
	KalmanCreate(&Gimbal_Auto_kalmanError[PITCH_206], 1, 10);
	 /*���鿨�����˲�,����*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
}

/* #��Ϣ��# ---------------------------------------------------------------------------------------------------------------------------------------*/
Gimbal_Mode_t GIMBAL_getGimbalMode(void)
{
	return Gimbal.State.mode;
}

/**
 *	@brief	�ж���̨�Ƿ��ڳ���ģʽ
 *	@return true - ����ģʽ
 *			false - �ǳ���ģʽ
 */
bool GIMBAL_ifNormalMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	�ж���̨�Ƿ�������ģʽ
 *	@return true - ����ģʽ
 *			false - ������ģʽ
 */
bool GIMBAL_ifAutoMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	�ж���̨�Ƿ��ڴ��ģʽ
 *	@return true - ���ģʽ
 *			false - �Ǵ��ģʽ
 */
bool GIMBAL_ifBuffMode(void)
{
	if(Gimbal.State.mode == GIMBAL_MODE_BIG_BUFF || Gimbal.State.mode == GIMBAL_MODE_SMALL_BUFF) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	�ж���̨�Ƿ�������׼�ڱ�
 *	@return true - ������׼�ڱ�
 *			false - û����׼�ڱ�
 */
bool GIMBAL_ifAimSentry(void)
{
	/* �ж��ڻ����ڱ�(̧ͷpitch��С) */
	if((Gimbal_PID[GYRO][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE || Gimbal_PID[MECH][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE ) 
		&& Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	�����̨�Ƿ���ٵ�λ
 *	@return true - ���ٵ�λ���Դ�
			false - ����δ��λ��ֹ��
 */
float debug_pix_yaw = 0;
float debug_pix_pitch = 0;
float debug_pix_yaw_ready = 35;
float debug_pix_pitch_ready = 35;
bool GIMBAL_BUFF_chaseReady(void)
{
	bool res = true;
	debug_pix_yaw = Gimbal.Buff.Yaw.erro;
	debug_pix_pitch = Gimbal.Buff.Pitch.erro;
	
	if((abs(debug_pix_yaw) < debug_pix_yaw_ready) && (VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) {
		res &= true;
	} else {
		res &= false;
	}
	
	if((abs(debug_pix_pitch) < debug_pix_pitch_ready) && (VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true)) {
		res &= true;
	} else {
		res &= false;
	}
	return res;
}



/* #Ӧ�ò�# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #ң��# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	����ң��ֵ������̨YAWλ�û�������ֵ(�ۼ�ֵ)
 *	@note	
 */
void REMOTE_setGimbalAngle(RC_Ctl_t *remoteInfo)
{
	float targetAngle;
	if(Flag.gimbal.FLAG_pidMode == MECH) {	// ��еģʽ
		/* Yaw */
		//.. ��̨��������˶�
		/* Pitch */
		targetAngle = (remoteInfo->rc.ch1 - RC_CH_VALUE_OFFSET)*RC_GIMBAL_MECH_PITCH_SENSITIVY;// Pitch��е��Ϊ���ԽǶ�
		Gimbal_PID[MECH][PITCH_206].Angle.target = constrain(Gimbal_PID[MECH][PITCH_206].Angle.target - targetAngle, // ̧ͷ��е�Ƕȼ�С
															 GIMBAL_MECH_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT);	// ����Pitch��(����ֵ)
	} else if(Flag.gimbal.FLAG_pidMode == GYRO) {	// ������ģʽ
		/* Yaw */
		targetAngle = (remoteInfo->rc.ch0 - RC_CH_VALUE_OFFSET)*RC_GIMBAL_GYRO_YAW_SENSITIVY;
		if((Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100))	// �����޸ı߽�ֵ(���Ƶ��̺���̨�ķ���Ƕ�)
			&& (Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100))) 
		{
			/* ң�������߽�ֵ���� */
			Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
		} 
		else if(Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100)) // �ִ���߽�
		{
			/* ң�������߽�ֵ���� */
			if(targetAngle > 0)	// ֻ�����Ұ�
				Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
		} 
		else if(Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100)) // �ִ��ұ߽�
		{
			/* ң�������߽�ֵ���� */
			if(targetAngle < 0)	// ֻ�������
				Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
		}					
		/* Pitch */		
		targetAngle = (remoteInfo->rc.ch1 - RC_CH_VALUE_OFFSET)*RC_GIMBAL_GYRO_PITCH_SENSITIVY;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // ̧ͷPitch�Ƕȼ�С
															 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// ����Pitch��(����ֵ)		
	}
}

/* #�������# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	���ݰ���ֵ������̨YAWλ�û�������ֵ(�ۼ�ֵ)
 *	@note	
 *			�������Ϊ��
 */
void KEY_setGimbalAngle(RC_Ctl_t *remoteInfo)
{
	float targetAngle;
	/* Yaw */
	targetAngle = MOUSE_X_MOVE_SPEED * KEY_GIMBAL_GYRO_YAW_SENSITIVY;	// ���X�����ٶ�*������
	if(Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100)	// �����޸ı߽�ֵ(���Ƶ��̺���̨�ķ���Ƕ�)
		&& Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100)) 
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
	} 
	else if(Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100)) // �ִ���߽�
	{
		if(targetAngle > 0)	// ֻ�����Ұ�
			Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
	} 
	else if(Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100)) // �ִ��ұ߽�
	{
		if(targetAngle < 0)	// ֻ�������
			Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
	}					
	/* Pitch */		
	targetAngle = -MOUSE_Y_MOVE_SPEED * KEY_GIMBAL_GYRO_PITCH_SENSITIVY;// ���Y�����ٶ�*������
	Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // ̧ͷPitch�Ƕȼ�С
														 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
														 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// ����Pitch��(����ֵ)	
}

/**
 *	@brief	���ݰ���ֵ������̨תͷ
 */
void KEY_setGimbalTurn(RC_Ctl_t *remoteInfo)
{
	float targetAngle = 0;
	static uint8_t keyQLockFlag = false;
	static uint8_t keyELockFlag = false;
	static uint8_t keyVLockFlag = false;
	/* ������ʱ��Ӧ,��ֹ�ּ��� */
	static portTickType  keyCurrentTime = 0;	
	static uint32_t keyQLockTime = 0;
	static uint32_t keyELockTime = 0;
	static uint32_t keyVLockTime = 0;
		
	keyCurrentTime = xTaskGetTickCount();
	
	if(IF_KEY_PRESSED_Q) {	// ����Q
		if(keyCurrentTime > keyQLockTime) {	// 250ms��Ӧһ��
			keyQLockTime = keyCurrentTime + TIME_STAMP_250MS;
			if(keyQLockFlag == false) {
				if(IF_KEY_PRESSED_E) {
					// ͬʱ����Q��E
				} else {	// ֻ��Q��û��E
					targetAngle = -90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyQLockFlag = true;
		}
	} else {	// �ɿ�Q
		keyQLockFlag = false;
	}
	
	if(IF_KEY_PRESSED_E) {	// ����E
		if(keyCurrentTime > keyELockTime) {	// 250ms��Ӧһ��
			keyELockTime = keyCurrentTime + TIME_STAMP_250MS;		
			if(keyELockFlag == false) {
				if(IF_KEY_PRESSED_Q) {
					// ͬʱ����Q��E
				} else {	// ֻ��E��û��Q
					targetAngle = +90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyELockFlag = true;
		}
	} else {
		keyELockFlag = false;
	}
	
	if(IF_KEY_PRESSED_V && !IF_KEY_PRESSED_CTRL) {	// ����V
		if(keyCurrentTime > keyVLockTime) {	// 500ms��Ӧһ��
			keyVLockTime = keyCurrentTime + TIME_STAMP_500MS;
			if(keyVLockFlag == false) {
				if(IF_KEY_PRESSED_A) {
					// ͬʱ����AV �� �Ȱ�A�ٰ�V
					targetAngle = -180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				} else {	// ֻ��V��û��A
					targetAngle = +180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyVLockFlag = true;
		}
	} else {
		keyVLockFlag = false;
	}
	
	/* б�º������ۼ���������ֹͻȻ���Ӻܴ������ֵ */
	Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = RAMP_float(Gimbal_PID[GYRO][YAW_205].AngleRampTarget, Gimbal_PID[GYRO][YAW_205].AngleRampFeedback, GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback < Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // �����ۼ�
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback > Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // �����ۼ�
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], -GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else // ���������
	{
		Gimbal_PID[GYRO][YAW_205].AngleRampTarget = 0;
		Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = 0;
	}
}

/**
 *	@brief	���ݰ���ֵ������̨����̧ͷ
 */
void KEY_setQuickPickUp(RC_Ctl_t *remoteInfo)
{
	float targetAngle;
	static uint8_t keyGLockFlag = false;
	
	if(IF_KEY_PRESSED_G) {
		if(keyGLockFlag == false) {
			targetAngle = 15.f/360*8192;	// ����̧ͷ15��
			Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // ̧ͷPitch�Ƕȼ�С
															 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// ����Pitch��(����ֵ)
		}
		keyGLockFlag = true;
	} else {
		keyGLockFlag = false;
	}
}

/**
 *	@brief	����Ҽ���������ģʽ
 *			�ɿ��Ҽ��˳�����ģʽ
 */
uint8_t test_auto_pid = 0;
static uint8_t mouseRLockFlag = false;	// ����Ҽ�����
static uint8_t rcSw1LockFlag = false;
static uint8_t keyCtrlLockFlag = false;
static portTickType  keyCurrentTime = 0;	
static uint32_t keyCtrlFLockTime = 0;
static uint32_t keyCtrlVLockTime = 0;
void KEY_setGimbalMode(RC_Ctl_t *remoteInfo)
{
	keyCurrentTime = xTaskGetTickCount();
	
	if(test_auto_pid == 0) {
		/* ����Ҽ� */
		if(IF_MOUSE_PRESSED_RIGH) {
			if(mouseRLockFlag == false && GIMBAL_ifBuffMode() == false) {
				Gimbal.State.mode = GIMBAL_MODE_AUTO;
				VISION_setMode(VISION_MODE_AUTO);
				Gimbal.Auto.FLAG_first_into_auto = true;
			}
			mouseRLockFlag = true;
		} else {
			if(GIMBAL_ifAutoMode() == true) { // �˳�����ģʽ
				Gimbal.State.mode = GIMBAL_MODE_NORMAL;
				VISION_setMode(VISION_MODE_MANUAL);	// �ֶ�ģʽ
				Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
				Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;
				Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
				Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;
			}
			mouseRLockFlag = false;
		}
	} else {
		if(IF_RC_SW1_MID) {
			if(rcSw1LockFlag == false) {
				Gimbal.State.mode = GIMBAL_MODE_AUTO;
				VISION_setMode(VISION_MODE_AUTO);
				Gimbal.Auto.FLAG_first_into_auto = true;
			}
			rcSw1LockFlag = true;
		} else {
			if(GIMBAL_ifAutoMode() == true) { // �˳�����ģʽ
				Gimbal.State.mode = GIMBAL_MODE_NORMAL;
				VISION_setMode(VISION_MODE_MANUAL);	// �ֶ�ģʽ
				Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
				Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;
				Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
				Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;	
			}
			rcSw1LockFlag = false;
		}
	}
	
	/* Ctrl+V��ϼ� */
	if(IF_KEY_PRESSED_CTRL) {
		if(keyCtrlLockFlag == false) {
			Gimbal.State.mode = GIMBAL_MODE_NORMAL;
			VISION_setMode(VISION_MODE_MANUAL);	// �ֶ�ģʽ
			Flag.gimbal.FLAG_pidMode = MECH;	// ǿ�ƽ����еģʽ
			GIMBAL_keyGyro_To_keyMech();
		}
		if(keyCurrentTime > keyCtrlVLockTime) {	
			keyCtrlVLockTime = keyCurrentTime + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_V) {	// Ctrl+V
				if(Gimbal.State.mode != GIMBAL_MODE_SMALL_BUFF) {
					Gimbal.State.mode = GIMBAL_MODE_SMALL_BUFF;
					VISION_setMode(VISION_MODE_SMALL_BUFF);	// ����С��
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.gimbal.FLAG_pidMode = GYRO;	// ǿ�ƽ���������ģʽ
					GIMBAL_keyMech_To_keyGyro();
				}
			}
		}
		if(keyCurrentTime > keyCtrlFLockTime) {
			keyCtrlFLockTime = keyCurrentTime + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_F) {	// Ctrl+F
				if(Gimbal.State.mode != GIMBAL_MODE_BIG_BUFF) {
					Gimbal.State.mode = GIMBAL_MODE_BIG_BUFF;
					VISION_setMode(VISION_MODE_BIG_BUFF);	// ������
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.gimbal.FLAG_pidMode = GYRO;	// ǿ�ƽ���������ģʽ
					GIMBAL_keyMech_To_keyGyro();
				}
			}
		}
		keyCtrlLockFlag = true;
	} else {
		if(keyCtrlLockFlag == true && Gimbal.State.mode == GIMBAL_MODE_NORMAL) {
			Gimbal.State.mode = GIMBAL_MODE_NORMAL;
			VISION_setMode(VISION_MODE_MANUAL);	// �ֶ�ģʽ
			Flag.gimbal.FLAG_pidMode = GYRO;	// ǿ�ƽ���������ģʽ
			GIMBAL_keyMech_To_keyGyro();
		}
		keyCtrlLockFlag = false;
	}


}

/* #��̨# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	��̨�����ʼ��
 *	@note		
 */
void GIMBAL_init(void)
{
	GIMBAL_kalmanCreate();	// �����������˲���
	GIMBAL_GYRO_calAverageOffset(Mpu_Info);	// �����ǽ��ٶ�����ֵ����	
}

/**
 *	@brief	��̨�����λ
 *	@note	# ���û�еģʽ����
 *			# �߱�5s�ĳ�ʱ�˳����ƣ���ֹ��̨û�и�λ��λһֱ����	
 */

void GIMBAL_reset(void)
{
	static uint32_t resetTime = 0;
	static portTickType tickTime_prev = 0;
	static portTickType tickTime_now = 0;
	
	tickTime_now = xTaskGetTickCount();
	if(tickTime_now  - tickTime_prev > TIME_STAMP_6000MS) {	// ��֤���ϵ�����£��´ο���
		Flag.gimbal.FLAG_resetOK = false;
		Cnt.gimbal.CNT_resetOK = 0;
		Flag.gimbal.FLAG_angleRecordStart = 0;	// ���¼�¼
		//..feedbackֵ�Ƿ���Ҫ����?
	}
	tickTime_prev = tickTime_now;
	
	if(Flag.gimbal.FLAG_angleRecordStart == 0) {	// ��¼�ϵ�ʱ��̨��е�ǶȺ������Ƿ���ֵ
		Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;	
		Flag.gimbal.FLAG_angleRecordStart = 1;
	}	
	
	if(Flag.gimbal.FLAG_resetOK == false) {	// ��̨��λδ���
		resetTime++;	// ��λ��ʱ
		if(Flag.gimbal.FLAG_pidStart == 1) {
			Flag.gimbal.FLAG_pidMode = MECH;
			/* ƽ��������̨�ƶ����м䣬��ֹ�ϵ��˦ */
			Gimbal_PID[MECH][YAW_205].Angle.target = RAMP_float(GIMBAL_MECH_YAW_ANGLE_MID_LIMIT, Gimbal_PID[MECH][YAW_205].Angle.target, GIMBAL_RAMP_BEGIN_YAW);
			Gimbal_PID[MECH][PITCH_206].Angle.target = RAMP_float(GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT, Gimbal_PID[MECH][PITCH_206].Angle.target, GIMBAL_RAMP_BEGIN_PITCH);
		}		
		
		/* �ȴ���̨���� */
		if(abs(Gimbal_PID[MECH][YAW_205].Angle.feedback - GIMBAL_MECH_YAW_ANGLE_MID_LIMIT) <= 1 
			&& abs(Gimbal_PID[MECH][PITCH_206].Angle.feedback - GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT) <= 1) {
			Cnt.gimbal.CNT_resetOK++;
		}
			
		/* ��λ�ɹ����߳�ʱǿ���˳�(5s) */
		if(Cnt.gimbal.CNT_resetOK > 250 || resetTime >= 2500) {	
			Cnt.gimbal.CNT_resetOK = 0;
			Flag.gimbal.FLAG_resetOK = true;// ��̨��λ�ɹ�
			resetTime = 0;					// ��λ��ʱ����
		}		
	} else if(Flag.gimbal.FLAG_resetOK == true) {	// ��̨��λ���
		Flag.gimbal.FLAG_resetOK = false;	// ���״̬��־λ
		BM_reset(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_GIMBAL);
		/* ģʽ�л�(���ϵ�̫���г�������ģʽ����Ϊ�����ǶȻ�δ�ȶ������̨˦ͷ) */			
		if(RC_Ctl_Info.rc.s2 == RC_SW_UP) {
			Flag.remote.FLAG_mode = KEY;
			Flag.gimbal.FLAG_pidMode = GYRO;
			GIMBAL_rcMech_To_keyGyro();
		} else if(RC_Ctl_Info.rc.s2 == RC_SW_MID) {
			Flag.remote.FLAG_mode = RC;
			Flag.gimbal.FLAG_pidMode = MECH;					
		} else if(RC_Ctl_Info.rc.s2 == RC_SW_DOWN) {
			Flag.remote.FLAG_mode = RC;
			Flag.gimbal.FLAG_pidMode = GYRO;
			GIMBAL_rcMech_To_rcGyro();					
		}
	}	
}

/**
 *	@brief	��̨������ģʽ��¼��������
 *	@note
 *					# ����Pitch�Ƕȵ��ٽ�ֵ����
 *					1. ǹ������ - -162.4��
 *					2. ǹ������ - +145.8��
 *					�ȡ�180�õ�һ��С��90�����ֵ
 *					�涨; 
 *					�±�Ϊ��( > -17��)
 *					�ϱ�Ϊ��( < +34��)
 *
 *					# ����Yaw�Ƕȵ��ٽ�ֵ����
 *						# ����:
 *							Yawÿ���ϵ綼�᲻һ�������������Դ���������ϵ
 *							��ͬ�Ĺ�����˳ʱ��Ƕ�ֵһ���Ǽ�С��
 *					1. ǹ������ - -59.6��
 *					2. ǹ������ - +115��
 *					�ȡ�180�õ�һ��С��90�����ֵ
 *						# ���ڣ�
 *					������������yaw�����������ݵ�Ư�Ƶ���Ч�����ѣ���˲��û�еģʽ�µĽǶ�
 *					��������Ϊ������̨����ĽǶ�(����̨�ִ����ƽǶȺ�ֻ��������һ�����������)��
 *					
 *				  # �ϵ��MPU6050���ݻ���һ��ʱ�䲻�ȶ�(�����ȴ�����)
 *					# ��еģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶȷ���ֵ
 *						ԭ�����ֶ��ƶ���̨yaw��pitch����ʱ��
 *							  # Yaw
 *							  ���������ݵķ������ɴ�5000+(���ó�����ϵ��)
 *							  �������ת�ٷ������ɴ�50+
 *							  # Pitch
 *							  ���������ݵķ������ɴ�3000+(���ó�����ϵ��)
 *							  �������ת�ٷ������ɴ�30+
 *							  �Աȿ���֪�������Ƿ����Ľ��ٶ�ֵ���ȸ��ߣ����
 *							  ������ģʽ�ͻ�еģʽ�µ��ٶȻ�������IMU�ķ�������
 *
 *					# YAW����	�Ұ� �����outΪ +
 *								��� �����outΪ -
 *					# PITCH��� ̧ͷ �����outΪ -
 *								��ͷ �����outΪ +
 *
 *	@direction
 *			IMU����
 *					̧ͷ	- ratePitch => -
 *							-	Pitch 	=> + 
 *					��ͷ 	- ratePitch => +
 *							- 	Pitch 	=> -
 *					���  	- rateYaw   => -
 *							- 	Yaw   	=> +
 *					�Ұ�  	- rateYaw   => +
 *							- 	Yaw		=> -
 *			�涨:
 *					̧ͷ	- ratePitch => -
 *								- Pitch => -(������ģʽ) 
 *					��ͷ 	- ratePitch => +
 *								- Pitch => +(������ģʽ)
 *					���  	- rateYaw   => -
 *								- Yaw   => -(������ģʽ)
 *					�Ұ�  	- rateYaw   => +
 *								- Yaw	=> +(������ģʽ)
 */
float pitch;
float yaw;
float rateYaw;
float kLPF_rateYaw = 0.20f;
float ratePitch;
float kLPF_ratePitch = 0.20f;
void GIMBAL_IMU_recordFeedback(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT])
{	
//	static float last_rateYaw = 0.f;
//	static float last_ratePitch = 0.f;
	/* ��ȡMPU6050���������� */
	mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
	MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	
	/* # Yaw # */
	yaw = -Mpu_Info.yaw;
	/* # ������ģʽ�µĽǶȷ���ֵ����IMU�ĽǶ�ֵ */
	pid[GYRO][YAW_205].Angle.feedback = (int)(yaw * GIMBAL_GYRO_ANGLE_ZOOM_INDEX);
	
	/* # Yaw �ٶȷ��� # */
	rateYaw = Mpu_Info.rateYaw - Mpu_Info.rateYawOffset;

	//	rateYaw = kLPF_rateYaw*rateYaw + (1-kLPF_rateYaw)*last_rateYaw;
//	last_rateYaw = rateYaw;
//	if(last_rateYaw != 0.f) {	// ����żȻ�ܴ������
//		if(abs(rateYaw / last_rateYaw) > 50.f)
//			rateYaw = last_rateYaw;
//	}

	/* # ��еģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[MECH][YAW_205].Speed.feedback = rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	/* # ������ģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[GYRO][YAW_205].Speed.feedback = rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	

	
	/* # Pitch �ٶȷ��� # */
	ratePitch = Mpu_Info.ratePitch - Mpu_Info.ratePitchOffset;

//	ratePitch = kLPF_rateYaw*ratePitch + (1-kLPF_ratePitch)*last_ratePitch;
//	last_ratePitch = ratePitch;
//	if(last_ratePitch != 0.f) {	// ����żȻ�ܴ������
//		if(abs(ratePitch / last_ratePitch) > 50.f)
//			ratePitch = last_ratePitch;
//	}	
//	
	
	/* # Pitch # */
	pitch = -Mpu_Info.pitch;
	/* # ������ģʽ�µĽǶȷ���ֵ����Pitch��е�Ƕȷ���ֵ */
	pid[GYRO][PITCH_206].Angle.feedback = g_Gimbal_Motor_Info[PITCH_206].angle;	
	/* # ��еģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[MECH][PITCH_206].Speed.feedback = ratePitch;	// ̧ͷ�ٶ�Ϊ-	GIMBAL_COMPENSATE_RATEPITCH
	/* # ������ģʽ�µ��ٶȷ���ֵ����IMU�Ľ��ٶ�ֵ */
	pid[GYRO][PITCH_206].Speed.feedback = ratePitch;	// ̧ͷ�ٶ�Ϊ-	GIMBAL_COMPENSATE_RATEPITCH
}

/**
 *	@brief	��̨���IMU��������ֵ
 */
void GIMBAL_GYRO_calAverageOffset(Mpu_Info_t mpuInfo)
{
	uint16_t i;
	for(i = 0; i < 50; i++) {
		delay_us(100);
		// ��ȡ�����ǽǶȺͽ��ٶ�
		mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
		MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	}
	for(i = 0; i < 1000; i++) {
		delay_us(200);
		mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
		Mpu_Info.pitchOffset += Mpu_Info.pitch;
		Mpu_Info.yawOffset += Mpu_Info.yaw;
		MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);
		Mpu_Info.ratePitchOffset += Mpu_Info.ratePitch;
		Mpu_Info.rateYawOffset   += Mpu_Info.rateYaw;
	}
	Mpu_Info.pitchOffset = Mpu_Info.pitchOffset/1000;
	Mpu_Info.yawOffset = Mpu_Info.yawOffset/1000;
	Mpu_Info.ratePitchOffset = (Mpu_Info.ratePitchOffset/1000);
	Mpu_Info.rateYawOffset   = (Mpu_Info.rateYawOffset/1000);
	
	if(abs(Mpu_Info.ratePitchOffset - (-21)) > 5) {
		Mpu_Info.ratePitchOffset = -21;
	}
	
	if(abs(Mpu_Info.rateYawOffset - (-20)) > 5) {
		Mpu_Info.rateYawOffset = -20;
	}	
}

/**
 *	@brief	������ģʽ����̨YAW����ֵ(�ۼ�ֵ)�߽紦��
 */
float GIMBAL_GYRO_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target)
{
	float target;
	target = pid->Angle.target + delta_target;
	if(target >= 180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {	// �����ұ߽�
		target = (-360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
	} else if(target <= -180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX){// ������߽�
		target = (+360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
	}
	return target;
}

/**
 *	@brief	ң�ػ�еģʽ -> ң��������ģʽ
 *	@note	�첽�л�
 */
void GIMBAL_rcMech_To_rcGyro(void)
{
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);	
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
//	Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// ��������
//	Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// ��������
	Gimbal_PID[MECH][YAW_205].Out = 0;
	Gimbal_PID[MECH][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	ң��������ģʽ -> ң�ػ�еģʽ
 *	@note	�첽�л�
 */
void GIMBAL_rcGyro_To_rcMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
//	Gimbal_PID[MECH][YAW_205].Speed.integrate = 0;	// ��������
//	Gimbal_PID[MECH][PITCH_206].Speed.integrate = 0;// ��������	
	Gimbal_PID[GYRO][YAW_205].Out = 0;
	Gimbal_PID[GYRO][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	ң�ػ�еģʽ -> ����ģʽ
 *	@note	�첽�л�
 */
void GIMBAL_rcMech_To_keyGyro(void)
{
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);	
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
//	Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// ��������
//	Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// ��������
	Gimbal_PID[MECH][YAW_205].Out = 0;
	Gimbal_PID[MECH][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	����ģʽ -> ң�ػ�еģʽ
 *	@note	�첽�л�
 */
void GIMBAL_keyGyro_To_rcMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
//	Gimbal_PID[MECH][YAW_205].Speed.integrate = 0;	// ��������
//	Gimbal_PID[MECH][PITCH_206].Speed.integrate = 0;// ��������
//	Gimbal_PID[GYRO][YAW_205].Out = 0;
//	Gimbal_PID[GYRO][PITCH_206].Out = 0;
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	����������ģʽ -> ���̻�еģʽ
 *	@note	�첽�л�
 */
void GIMBAL_keyGyro_To_keyMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	Gimbal_PID[MECH][YAW_205].Speed.integrate = 0;	// ��������
	Gimbal_PID[MECH][PITCH_206].Speed.integrate = 0;// ��������
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	���̻�еģʽ -> ����������ģʽ
 *	@note	�첽�л�
 */
void GIMBAL_keyMech_To_keyGyro(void)
{
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
	Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// ��������
	Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// ��������
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	Ŀ���ٶȽ���
 *	@note	������֮֡��ĽǶȲ����Ŀ����ٶ�
 */
float speed_calculate(Speed_Calculate_t *info, uint32_t time, float position)
{
	info->delay_cnt++;
	if(time != info->last_time)
	{
		info->speed = (position - info->last_position)/(time - info->last_time)*2;	// ֡������ٶ�(*2�о����ٶȷŴ����˼)
		
		info->processed_speed = info->speed;
			
		info->last_time = time;
		info->last_position = position;
		info->last_speed = info->speed;
		info->delay_cnt = 0;
	}
	
	/* ����ʱ����������Ӿ����ݸ���������еģ��������Ӿ�ʧ������һ��ʱ��󽫷����ٶ����� */
	if(info->delay_cnt > 250)	// 500ms
	{
		info->processed_speed = 0;
	}
	
	return info->processed_speed;
}

/**
 *	@brief	����ģʽPID����
 *	@note	3.75f -> �������֮����Ҫ���¼���һ������ֵ
 */
float GIMBAL_AUTO_PITCH_COMPENSATION = (0.f/360.0f*8192.0f);
float auto_yaw_ramp = 260;
float auto_pitch_ramp = 80;
void GIMBAL_VISION_AUTO_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	/* ��������֡�� */
	gimbal->Auto.Time[NOW] = xTaskGetTickCount();
	gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* >60ms ����Ϊ��ʧĿ�����ʶ�� */
//	if(gimbal->Auto.Time[DELTA] > TIME_STAMP_60MS) {
//		pid[GYRO][YAW_205].Speed.integrate = 0;
//		pid[GYRO][PITCH_206].Speed.integrate = 0;
//	}
	
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
	}
	
	/* ���ݸ��� */
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* ������ݸ��±�־λ����ֹδ���յ������ݵ�������ظ����� */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw ������ */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_getYawFeedback();
			//gimbal->Auto.Yaw.erro = KalmanFilter(&Gimbal_Auto_kalmanError[YAW_205], gimbal->Auto.Yaw.erro);
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
			/* Pitch ������ */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_getPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
			//gimbal->Auto.Pitch.erro = KalmanFilter(&Gimbal_Auto_kalmanError[PITCH_206], gimbal->Auto.Pitch.erro);
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
		} else {
			/* ���ܴ������� */
			gimbal->Auto.Yaw.erro = 0;
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
			gimbal->Auto.Pitch.erro = 0;
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;
		}
	}
	
	/*----�����޸�----*/
	pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
	gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
}

/**
 *	@brief	����ģʽ���Ԥ���
 *	@note	��̨�Ұ�(Ŀ������) - Ŀ���ٶ�Ϊ��
 *					   			Ŀ��Ƕ�Ϊ��
 */
Speed_Calculate_t	yaw_angle_speed_struct, pitch_angle_speed_struct;
float yaw_angle_raw, pitch_angle_raw;
float *yaw_kf_result, *pitch_kf_result;	// ���׿������˲����,0�Ƕ� 1�ٶ�
float yaw_angle_speed, pitch_angle_speed;
float yaw_angle_predict, pitch_angle_predict;
float yaw_predict_k, pitch_predict_k;


float auto_predict_start_delay;
float auto_predict_start_delay_boundary = 80;

float auto_yaw_predict_max = 14;
float auto_pitch_predict_max;

float auto_yaw_speed_predict_low = 0.35;
float auto_yaw_speed_predict_high = 25;

float js_yaw_angle = 0.f;
float js_yaw_speed = 0.f;
void GIMBAL_AUTO_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
	}
	
	/*----���ݸ���----*/
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* ������ݸ��±�־λ����ֹδ���յ������ݵ�������ظ����� */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		/* ��������֡�� */
		gimbal->Auto.Time[NOW] = xTaskGetTickCount();
		gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];
		gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
		
		/* ʶ��Ŀ�� */
		if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw ���Ŵ� */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_getYawFeedback();
			/* Pitch ���Ŵ� */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_getPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
		} 
		/* δʶ��Ŀ�� */
		else {
			/* Yaw ���Ϊ0 */
			gimbal->Auto.Yaw.erro = 0;
			/* Pitch ���Ϊ0 */
			gimbal->Auto.Pitch.erro = 0;
		}
		/* Yaw Ŀ����� */
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
		/* Pitch Ŀ����� */
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
	}
	
	/*----���¶��׿�����Ԥ��ֵ----*/
	/* ʶ��Ŀ�� */
	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		/* ���¶��׿������ٶ��������ֵ */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Yaw.target);
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Pitch.target);

		/* �ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ� */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, yaw_angle_speed);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, pitch_angle_speed);
	} 
	/* δʶ��Ŀ�� */
	else {
		/* ���¶��׿������ٶ��������ֵ */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), gimbal->Auto.Yaw.target);	
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), gimbal->Auto.Pitch.target);

		/* �ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ� */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, 0);		// ʶ�𲻵�ʱ��ΪĿ���ٶ�Ϊ0
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, 0);	// ʶ�𲻵�ʱ��ΪĿ���ٶ�Ϊ0
	}
	
	js_yaw_speed = yaw_kf_result[KF_SPEED]*100;
	js_yaw_angle = yaw_kf_result[KF_ANGLE]*100;
	
	
	/*----Ԥ��������----*/
	/* ʶ��Ŀ�� */
	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		auto_predict_start_delay++;	// �˲���ʱ����
		
		if(auto_predict_start_delay > auto_predict_start_delay_boundary 
				&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(yaw_kf_result[KF_SPEED]) >=  auto_yaw_speed_predict_low 
						&& abs(yaw_kf_result[KF_SPEED]) <=  auto_yaw_speed_predict_high )
		{
			/* Ŀ������ -> Ԥ����Ϊ��(��̨����) */
			if(yaw_kf_result[KF_SPEED] < 0) {
				yaw_angle_predict = yaw_predict_k * yaw_kf_result[KF_SPEED]; // ʱ�䳣��2ms����Ԥ��ϵ����
			}
		}
	}
	/* δʶ��Ŀ�� */
	else {
	
	}
	
	/*----�����޸�----*/
	pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
}

/**
 *	@brief	���ģʽPID����
 *	@note	�췽����������������
 *			�����׶Σ�
 *			��С��
 *				��飺5:59~4:00������������ɹ�����1.5������������1����
 *				�˶����ԣ�10RPM
 *			�ڴ��
 *				��飺2:59~0:00������������ɹ�����2��������50%�����ӳɣ�����1����
 *			
 *			������Ϣ��
 *			���������1��2,��糵�������2��283,�ھ�70cm,�⾶80cm
 *			���������ؼ����
 *				�������ؿɼ���״̬ʱ������������ռ�첢ͣ��3s��ǹ������ÿ����ȴֵ��Ϊ5��
 *				2.5sδ����/��������װ�װ� -������ʧ��
 *
 *			����˼·��
 *			������ģʽ��ʱ����̨б�¹���(���Ӿ�����������ԭ��Ϊ����)
 *
 *			# �ڼҲ���������
 *			ͶӰֱ�߾��룺7.40m
 *			������ľ�����棺1.55m
 *			ǹ�ܾ��������ģ�7.56m(���ɶ���)
 *	
 *			# ����ͷ����̨������ͷ�ڵ��̵Ŀɲο�������룩
 */

void GIMBAL_VISION_BUFF_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float yaw_gyro_mid, yaw_mech_mid;
	static float pitch_gyro_mid, pitch_mech_mid;
	static uint16_t into_buff_time = 0;
	static uint16_t lost_cnt = 0;
	/* ������֡�� */
	gimbal->Buff.Time[NOW] = xTaskGetTickCount();
	gimbal->Buff.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

	/* �л�pitch������ */
	if((GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT - pid[GYRO][PITCH_206].Angle.feedback) > 0)
		GIMBAL_BUFF_PITCH_COMPENSATION = BUFF_PITCH_MODIFY_TABLE[(uint8_t)((GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT - pid[GYRO][PITCH_206].Angle.feedback)/100)];
	else
		GIMBAL_BUFF_PITCH_COMPENSATION = 0;
	
	if(gimbal->Buff.FLAG_first_into_buff == true) {
		gimbal->Buff.FLAG_first_into_buff = false;
		into_buff_time = 0;
		/* ��¼�ս�����ģʽʱ�������ǽǶ� */
		yaw_gyro_mid = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		pitch_gyro_mid = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
		/* ��¼�ս�����ģʽʱ�Ļ�е�Ƕ� */
		yaw_mech_mid = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		pitch_mech_mid = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	} else {
		/* ��ֹ�ӵ�̫�� */
		if(into_buff_time < 250)	
			into_buff_time++;
	}
	
	/* ���ݸ��� */
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* ������ݸ��±�־λ����ֹδ���յ������ݵ�������ظ����� */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	
		if(VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true) {	// ʶ��Ŀ����������
			lost_cnt = 0;
			if(into_buff_time > 100) {
				/* Yaw ������ */
				gimbal->Buff.Yaw.erro = (gimbal->Buff.Yaw.kp * VISION_BUFF_getYawFeedback()) + GIMBAL_BUFF_YAW_COMPENSATION;
				gimbal->Buff.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Buff.Yaw.erro;
				/* Pitch ������ */
				gimbal->Buff.Pitch.erro = (gimbal->Buff.Pitch.kp * VISION_BUFF_getPitchFeedback()) + GIMBAL_BUFF_PITCH_COMPENSATION;
				gimbal->Buff.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Buff.Pitch.erro;
				/*----�����޸�----*/
				pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Buff.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, GIMBAL_BUFF_YAW_RAMP);
				pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Buff.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, GIMBAL_BUFF_PITCH_RAMP);				
			} else {
				pid[GYRO][YAW_205].Angle.target = yaw_gyro_mid;
				pid[GYRO][PITCH_206].Angle.target = pitch_gyro_mid;
			}
		} else {	// δʶ��Ŀ���򱣳�ԭλ��
			gimbal->Buff.Yaw.erro = 0;
			gimbal->Buff.Pitch.erro = 0;
			if(lost_cnt > 50) {	// ����50֡��ʧ
//				pid[GYRO][YAW_205].Angle.target = yaw_gyro_mid;
//				pid[GYRO][PITCH_206].Angle.target = pitch_gyro_mid;	
				if(lost_cnt == 51) {	// ֻ��ֵһ��
					pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
					pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], 
													((yaw_mech_mid - Gimbal_PID[MECH][YAW_205].Angle.feedback)*360/8192.f));
					pid[GYRO][PITCH_206].Angle.target = pitch_mech_mid;	// Pitch���û�еģʽ
					lost_cnt++;
				}
			} else {
				lost_cnt++;
			}
		}
	}
	
	/*----�����޸�----*/
	//..�Զ���
	
	gimbal->Buff.Time[PREV] = gimbal->Buff.Time[NOW];
}

/**
 *	@brief	��̨�������
 */
void GIMBAL_normalControl(void)
{
	KEY_setGimbalAngle(&RC_Ctl_Info);
	KEY_setGimbalTurn(&RC_Ctl_Info);
	KEY_setQuickPickUp(&RC_Ctl_Info);	
}

/**
 *	@brief	��̨�������
 */
void GIMBAL_autoControl(void)
{
	/* �Ӿ����ݿ��� && ����ģʽ�� */
	if( (VISION_isDataValid()) && (Flag.remote.FLAG_mode == KEY) ) {
		/*----�����޸�----*/
		/* �Ӿ�Ԥ��� */
		//GIMBAL_VISION_AUTO_pidCalculate(Gimbal_PID, &Gimbal);
		/* ���Ԥ��� */
		GIMBAL_AUTO_pidCalculate(Gimbal_PID, &Gimbal);
	}
	
//	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == false) {
//		if(lost_cnt > 250) {
//			KEY_setGimbalAngle(&RC_Ctl_Info);
//		} else {
//			lost_cnt++;
//		}
//	} else {
//		lost_cnt = 0;
//	}
	
}

/**
 *	@brief	��̨�������
 *	@note	
 *	1.�Ȱ�һ��װ�׳����ŵ������棬�����⣬��offsetx��offsety�Ѻ���㵽װ�װ����ġ�
 *
 *	2.��ת���װ�װ壬����ĺ����ǲ���һ��������Բ������˵��������ľ��������ʱ�򣬲���װ�װ���ת�Ǳߣ������һֱ���š�
 *
 *	3.��̧ͷ������
 *
 *	4.���ʱ����Ϊ������ǹ�ܾ�����ƽ�⣬���Դ򵯣���offsetx��offsety�������С�
 *
 *	5.����λ�����Ŵ�
 *
 *	6.����ת��
 *	
 *	7.pid�����ĵã�pitch��yaw��Ҫ�ر�Ӳ�����ԼӺܴ�Ļ��֣�ע�͵������������Ĳ������ȶ�һ�㣩
 */
void GIMBAL_buffControl(void)
{
	/* ����WSADQEV(���ⷽ���)���˳����ģʽ */
	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 
	{
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;	// ��������ģʽ
		Flag.gimbal.FLAG_pidMode = GYRO;		// ����������ģʽ
		Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;	// GIMBAL_GYRO_PITCH_ANGLE_MID_LIMIT
		Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// ��������
		Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// ��������
		REVOLVER_setAction(SHOOT_NORMAL);	// ��ɳ������ģʽ
	}
	
	/* �Ӿ����ݿ��� && ����ģʽ�� */
	if( (VISION_isDataValid()) && (Flag.remote.FLAG_mode == KEY) ) {
		/*----�����޸�----*/
		GIMBAL_VISION_BUFF_pidCalculate(Gimbal_PID, &Gimbal);	

	}
}

/* #�����# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	pid�������������
 */
uint8_t test_yaw_pid = 0;
uint8_t test_pitch_pid = 0;
float   test_yaw_speed_max_target = 8000;
float   test_pitch_speed_max_target = 4000;
void GIMBAL_pidControlTask(void)
{
	/* YAW �ǶȻ� */
	GIMBAL_Angle_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], YAW_205);
	/* YAW �ٶȻ� */
	if(test_yaw_pid == 0) {
			Gimbal_PID[Flag.gimbal.FLAG_pidMode][YAW_205].Speed.target = Gimbal_PID[Flag.gimbal.FLAG_pidMode][YAW_205].Angle.out;
			GIMBAL_SPEED_PID_IOUT_MAX = 18000;
	} else {
		Gimbal_PID[Flag.gimbal.FLAG_pidMode][YAW_205].Speed.target = (RC_Ctl_Info.rc.ch0 - 1024)/660.f * test_yaw_speed_max_target;
		GIMBAL_SPEED_PID_IOUT_MAX = 18000;
	}
	GIMBAL_Speed_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], YAW_205);
	
	/* PITCH �ǶȻ� */
	GIMBAL_Angle_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], PITCH_206);
	/* PITCH �ٶȻ� */
	if(test_pitch_pid == 0) {
			Gimbal_PID[Flag.gimbal.FLAG_pidMode][PITCH_206].Speed.target = Gimbal_PID[Flag.gimbal.FLAG_pidMode][PITCH_206].Angle.out;
			GIMBAL_SPEED_PID_IOUT_MAX = 18000;
	} else {
		Gimbal_PID[Flag.gimbal.FLAG_pidMode][PITCH_206].Speed.target = -(RC_Ctl_Info.rc.ch1 - 1024)/660.f * test_pitch_speed_max_target;
		GIMBAL_SPEED_PID_IOUT_MAX = 18000;
	}
	GIMBAL_Speed_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], PITCH_206);
	
	GIMBAL_pidOut(Gimbal_PID[Flag.gimbal.FLAG_pidMode]);	
}

/**
 *	@brief	ң�ؿ�����̨
 */
void GIMBAL_rcControlTask(void)
{
	REMOTE_setGimbalAngle(&RC_Ctl_Info);	
}

/**
 *	@brief	���̿�����̨
 */
void GIMBAL_keyControlTask(void)
{
	/* ������̨��ģʽ */
	KEY_setGimbalMode(&RC_Ctl_Info);
	switch(Gimbal.State.mode)
	{
		case GIMBAL_MODE_NORMAL:
			GIMBAL_normalControl();
			break;
		case GIMBAL_MODE_AUTO:
			GIMBAL_autoControl();
			break;
		case GIMBAL_MODE_BIG_BUFF:
		case GIMBAL_MODE_SMALL_BUFF:
			GIMBAL_buffControl();
			break;
	}
}

/**
 *	@brief	��̨ʧ�ر���
 */
void GIMBAL_selfProtect(void)
{	
	GIMBAL_stop(Gimbal_PID[MECH]);
	GIMBAL_stop(Gimbal_PID[GYRO]);
	GIMBAL_pidParamsInit(Gimbal_PID[MECH], GIMBAL_MOTOR_COUNT);
	GIMBAL_pidParamsInit(Gimbal_PID[GYRO], GIMBAL_MOTOR_COUNT);	
}

/**
 *	@brief	��̨����
 */
void GIMBAL_control(void)
{
	/*----��Ϣ����----*/
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);

	/*----�����޸�----*/
	if(BM_ifSet(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_GIMBAL)) {	// ��λ״̬
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;
		GIMBAL_reset(); // ��̨��λ
	} else {
		if(Flag.remote.FLAG_mode == RC) {
			GIMBAL_rcControlTask();
		} else if(Flag.remote.FLAG_mode == KEY) {
			GIMBAL_keyControlTask();
		}
	}

	/* ������̨ģʽ�л�PID���� */
	GIMBAL_pidParamsSwitch(Gimbal_PID, &Gimbal);

	/*----�������----*/
	GIMBAL_pidControlTask();
}
