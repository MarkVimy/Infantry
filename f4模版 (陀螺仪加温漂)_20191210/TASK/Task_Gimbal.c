/**
 * @file        Task_Gimbal.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        1-October-2019
 * @brief       This file includes the Gimbal(云台) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 */

/**
 *	 串级PID	参考网址：https://www.jianshu.com/p/4b0fa85cd353
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

/*二阶卡尔曼*/
#define KF_ANGLE	0
#define KF_SPEED	1
#define KF_ACCEL	2

/* Private variables ---------------------------------------------------------*/
/* 卡尔曼滤波器 */
extKalman_t Gimbal_kalmanError[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extKalman_t Mpu_kalmanError[2];
extKalman_t Gimbal_Auto_kalmanError[GIMBAL_MOTOR_COUNT];

kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}//500 1000
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002/*0.001*/, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 400}
};//初始化pitch的部分kalman参数

float GIMBAL_BUFF_PITCH_COMPENSATION =	0;
float GIMBAL_BUFF_YAW_COMPENSATION = -175;
float GIMBAL_BUFF_YAW_RAMP = 180;	// 185	160	185	180
float GIMBAL_BUFF_PITCH_RAMP = 140;	// 130	105	120	140

/* 打符的时候pitch补偿表 */
float BUFF_PITCH_MODIFY_TABLE[11] = 
{
	// 从最低位置开始以4.4°(机械角 底-顶 = 4472-3045 = 1067 分成11份，100为机械角度步进)为角度步进
	/* 打符的时候发现 [3]90 ~ [7]80为有效范围 */
	/* 头抬得越高 []下标值越大 => MAX_UP -> [11] MAX_DOWN -> [0] */
	/* 补偿值越大 -> 头压得越低(补偿为0的时候头会偏得比较高) */
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* ## Global variables ## ----------------------------------------------------*/

/**
 *	@pid 参数分布表
 *	1. 正常模式	- 机械+陀螺仪
 *	2. 自瞄模式 - 陀螺仪
 *	3. 打符模式 - 机械+陀螺仪
 *	4. 吊射基地模式	- 机械
 */

// 34.40 46.25 10.45
/**
 *	@brief	云台PID
 */
Gimbal_PID_t	Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT] = {
	{	// MECH - 机械模式
		{	// YAW(机械模式)
			/* 速度环 */
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
			/* 位置环 */
			.Angle.kp = 10.51,	
			.Angle.ki = 0,			
			.Angle.kd = 0,
			.Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,	// 归中
			.Angle.feedback = 0,
			.Angle.erro = 0,
			.Angle.last_erro = 0,
			.Angle.integrate = 0,
			.Angle.integrate_max = 0,
			.Angle.pout = 0,
			.Angle.iout = 0,
			.Angle.dout = 0,
			.Angle.out = 0,	
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,
		},
		{	// PITCH(机械模式)
			/* 速度环 */
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
			/* 位置环 */
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
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,		
		},
	},	// MECH - 机械模式
	{	// GYRO - 陀螺仪模式
		{	// YAW(陀螺仪模式)
			/* 速度环 */
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
			/* 位置环 */
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
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,		
		},
		{	// PITCH(陀螺仪模式) - 采用机械模式那一套
			/* 速度环 */
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
			/* 位置环 */
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
			/* 斜坡数据 */
			.AngleRampTarget = 0,
			.AngleRampFeedback = 0,
			/* 输出 */
			.Out = 0,
		},
	}	// GYRO - 陀螺仪模式
};

/**
 *	@brief	云台综合信息
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
 *	@brief	云台电机PID参数初始化
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
 *	@brief	根据云台当前模式进行PID参数的切换
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
			if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) // 锁定目标
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
			else	// 未锁定目标则正常移动云台
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
 *	@brief	云台电机紧急刹车
 */
void GIMBAL_stop(Gimbal_PID_t *pid)
{
	static float pid_out[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	pid[YAW_205].Speed.out = 0;
	pid[YAW_205].Out = 0;
	pid[PITCH_206].Speed.out = 0;
	pid[PITCH_206].Out = 0;
	
	CAN1_send(0x1FF, pid_out);	// 云台CAN总线标准标识符
}

/**
 *	@brief	云台电机速度环
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
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
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
	/* 内环速度环最终输出 */
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	js_gimbal_yaw_speed_out = pid[YAW_205].Speed.out;
	js_gimbal_pitch_speed_out = pid[PITCH_206].Speed.out;
}

/**
 *	@brief	云台电机位置环
 *	@note
 *			串环PID
 *			内环 期望值 期望角速度(实际上就是速度环)
 *					 输出值 期望角加速度
 *
 *			外环 期望值 期望角度
 *					 输出值 期望角速度
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

	if((Flag.gimbal.FLAG_pidMode == MECH) && (MOTORx == YAW_205)) {	// 计算有效误差
		if(pid[MOTORx].Angle.erro >= 4096.f) {
			pid[MOTORx].Angle.erro = +(8192.f - pid[MOTORx].Angle.erro);
		} else if(pid[MOTORx].Angle.erro <= -4096.f) {
			pid[MOTORx].Angle.erro = -(8192.f + pid[MOTORx].Angle.erro);
		}
	}	
	
	if((Flag.gimbal.FLAG_pidMode == GYRO) && (MOTORx == YAW_205)) {	// 计算有效误差
		if(pid[MOTORx].Angle.erro >= 180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
			pid[MOTORx].Angle.erro = -(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX - pid[MOTORx].Angle.erro);
		} else if(pid[MOTORx].Angle.erro <= -180.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {
			pid[MOTORx].Angle.erro = +(360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + pid[MOTORx].Angle.erro);
		}
	}
	
	/* 对误差进行卡尔曼滤波，消除低频等幅抖动 */
	if(Gimbal.State.mode == GIMBAL_MODE_NORMAL) {	// 正常模式下
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.gimbal.FLAG_pidMode][MOTORx], pid[MOTORx].Angle.erro);
	}
	if(Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		pid[MOTORx].Angle.erro = KalmanFilter(&Gimbal_kalmanError[Flag.gimbal.FLAG_pidMode][MOTORx], pid[MOTORx].Angle.erro);
	}
	
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;

	/* Pout */
	pid[MOTORx].Angle.pout = pid[MOTORx].Angle.kp * pid[MOTORx].Angle.erro;
	/* Iout */
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
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
 *	@brief	云台电机PID的最终输出
 */
void GIMBAL_pidOut(Gimbal_PID_t *pid)
{
	float pidOut[4] = {0, 0, 0, 0};
	
	/* CAN发送电压值 */
	pidOut[YAW_205] = pid[YAW_205].Out;		// 0x205
	pidOut[PITCH_206] = pid[PITCH_206].Out;	// 0x206
	
	CAN1_send(0x1FF, pidOut);
}

/**
 *	@brief	云台卡尔曼滤波器初始化
 */
void GIMBAL_kalmanCreate(void)
{
	/* 卡尔曼滤波器初始化 */
	KalmanCreate(&Gimbal_kalmanError[MECH][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[MECH][PITCH_206], 1, 60);
	KalmanCreate(&Gimbal_kalmanError[GYRO][YAW_205], 1, 40);
	KalmanCreate(&Gimbal_kalmanError[GYRO][PITCH_206], 1, 60);
	/* Mpu6050 */
	KalmanCreate(&Mpu_kalmanError[YAW_205], 1, 80);
	KalmanCreate(&Mpu_kalmanError[PITCH_206], 1, 10);	
	/* 自瞄 */
	KalmanCreate(&Gimbal_Auto_kalmanError[YAW_205], 1, 20);
	KalmanCreate(&Gimbal_Auto_kalmanError[PITCH_206], 1, 10);
	 /*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
	
	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
}

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
Gimbal_Mode_t GIMBAL_getGimbalMode(void)
{
	return Gimbal.State.mode;
}

/**
 *	@brief	判断云台是否处于常规模式
 *	@return true - 常规模式
 *			false - 非常规模式
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
 *	@brief	判断云台是否处于自瞄模式
 *	@return true - 自瞄模式
 *			false - 非自瞄模式
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
 *	@brief	判断云台是否处于打符模式
 *	@return true - 打符模式
 *			false - 非打符模式
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
 *	@brief	判断云台是否正在瞄准哨兵
 *	@return true - 正在瞄准哨兵
 *			false - 没有瞄准哨兵
 */
bool GIMBAL_ifAimSentry(void)
{
	/* 判断在击打哨兵(抬头pitch减小) */
	if((Gimbal_PID[GYRO][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE || Gimbal_PID[MECH][PITCH_206].Angle.feedback <= GIMBAL_AUTO_LOCK_SENTRY_ANGLE ) 
		&& Gimbal.State.mode == GIMBAL_MODE_AUTO) {
		return true;
	} else {
		return false;
	}
}

/**
 *	@brief	打符云台是否跟踪到位
 *	@return true - 跟踪到位可以打弹
			false - 跟踪未到位禁止打弹
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



/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/* #遥控# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据遥控值设置云台YAW位置环的期望值(累加值)
 *	@note	
 */
void REMOTE_setGimbalAngle(RC_Ctl_t *remoteInfo)
{
	float targetAngle;
	if(Flag.gimbal.FLAG_pidMode == MECH) {	// 机械模式
		/* Yaw */
		//.. 云台跟随底盘运动
		/* Pitch */
		targetAngle = (remoteInfo->rc.ch1 - RC_CH_VALUE_OFFSET)*RC_GIMBAL_MECH_PITCH_SENSITIVY;// Pitch机械角为绝对角度
		Gimbal_PID[MECH][PITCH_206].Angle.target = constrain(Gimbal_PID[MECH][PITCH_206].Angle.target - targetAngle, // 抬头机械角度减小
															 GIMBAL_MECH_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)
	} else if(Flag.gimbal.FLAG_pidMode == GYRO) {	// 陀螺仪模式
		/* Yaw */
		targetAngle = (remoteInfo->rc.ch0 - RC_CH_VALUE_OFFSET)*RC_GIMBAL_GYRO_YAW_SENSITIVY;
		if((Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100))	// 这里修改边界值(限制底盘和云台的分离角度)
			&& (Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100))) 
		{
			/* 遥控期望边界值处理 */
			Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
		} 
		else if(Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100)) // 抵达左边界
		{
			/* 遥控期望边界值处理 */
			if(targetAngle > 0)	// 只允许右摆
				Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
		} 
		else if(Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100)) // 抵达右边界
		{
			/* 遥控期望边界值处理 */
			if(targetAngle < 0)	// 只允许左摆
				Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
		}					
		/* Pitch */		
		targetAngle = (remoteInfo->rc.ch1 - RC_CH_VALUE_OFFSET)*RC_GIMBAL_GYRO_PITCH_SENSITIVY;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
															 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)		
	}
}

/* #键盘鼠标# -------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	根据按键值设置云台YAW位置环的期望值(累加值)
 *	@note	
 *			鼠标右下为正
 */
void KEY_setGimbalAngle(RC_Ctl_t *remoteInfo)
{
	float targetAngle;
	/* Yaw */
	targetAngle = MOUSE_X_MOVE_SPEED * KEY_GIMBAL_GYRO_YAW_SENSITIVY;	// 鼠标X反馈速度*灵敏度
	if(Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100)	// 这里修改边界值(限制底盘和云台的分离角度)
		&& Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100)) 
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
	} 
	else if(Gimbal_PID[MECH][YAW_205].Angle.feedback < (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT-1100)) // 抵达左边界
	{
		if(targetAngle > 0)	// 只允许右摆
			Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
	} 
	else if(Gimbal_PID[MECH][YAW_205].Angle.feedback > (GIMBAL_MECH_YAW_ANGLE_MID_LIMIT+1100)) // 抵达右边界
	{
		if(targetAngle < 0)	// 只允许左摆
			Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], targetAngle);
	}					
	/* Pitch */		
	targetAngle = -MOUSE_Y_MOVE_SPEED * KEY_GIMBAL_GYRO_PITCH_SENSITIVY;// 鼠标Y反馈速度*灵敏度
	Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
														 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
														 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)	
}

/**
 *	@brief	根据按键值设置云台转头
 */
void KEY_setGimbalTurn(RC_Ctl_t *remoteInfo)
{
	float targetAngle = 0;
	static uint8_t keyQLockFlag = false;
	static uint8_t keyELockFlag = false;
	static uint8_t keyVLockFlag = false;
	/* 按键延时响应,防止手贱狂按 */
	static portTickType  keyCurrentTime = 0;	
	static uint32_t keyQLockTime = 0;
	static uint32_t keyELockTime = 0;
	static uint32_t keyVLockTime = 0;
		
	keyCurrentTime = xTaskGetTickCount();
	
	if(IF_KEY_PRESSED_Q) {	// 按下Q
		if(keyCurrentTime > keyQLockTime) {	// 250ms响应一次
			keyQLockTime = keyCurrentTime + TIME_STAMP_250MS;
			if(keyQLockFlag == false) {
				if(IF_KEY_PRESSED_E) {
					// 同时按下Q和E
				} else {	// 只按Q，没按E
					targetAngle = -90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyQLockFlag = true;
		}
	} else {	// 松开Q
		keyQLockFlag = false;
	}
	
	if(IF_KEY_PRESSED_E) {	// 按下E
		if(keyCurrentTime > keyELockTime) {	// 250ms响应一次
			keyELockTime = keyCurrentTime + TIME_STAMP_250MS;		
			if(keyELockFlag == false) {
				if(IF_KEY_PRESSED_Q) {
					// 同时按下Q和E
				} else {	// 只按E，没按Q
					targetAngle = +90*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyELockFlag = true;
		}
	} else {
		keyELockFlag = false;
	}
	
	if(IF_KEY_PRESSED_V && !IF_KEY_PRESSED_CTRL) {	// 按下V
		if(keyCurrentTime > keyVLockTime) {	// 500ms响应一次
			keyVLockTime = keyCurrentTime + TIME_STAMP_500MS;
			if(keyVLockFlag == false) {
				if(IF_KEY_PRESSED_A) {
					// 同时按下AV 或 先按A再按V
					targetAngle = -180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				} else {	// 只按V，没按A
					targetAngle = +180*GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
					Gimbal_PID[GYRO][YAW_205].AngleRampTarget += targetAngle;
				}
			}
			keyVLockFlag = true;
		}
	} else {
		keyVLockFlag = false;
	}
	
	/* 斜坡函数给累加期望，防止突然增加很大的期望值 */
	Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = RAMP_float(Gimbal_PID[GYRO][YAW_205].AngleRampTarget, Gimbal_PID[GYRO][YAW_205].AngleRampFeedback, GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback < Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // 正向累加
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else if(Gimbal_PID[GYRO][YAW_205].AngleRampFeedback > Gimbal_PID[GYRO][YAW_205].AngleRampTarget) // 反向累加
	{
		Gimbal_PID[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], -GIMBAL_GYRO_ANGLE_ZOOM_INDEX/1.5f);
	} 
	else // 缓冲池清零
	{
		Gimbal_PID[GYRO][YAW_205].AngleRampTarget = 0;
		Gimbal_PID[GYRO][YAW_205].AngleRampFeedback = 0;
	}
}

/**
 *	@brief	根据按键值设置云台快速抬头
 */
void KEY_setQuickPickUp(RC_Ctl_t *remoteInfo)
{
	float targetAngle;
	static uint8_t keyGLockFlag = false;
	
	if(IF_KEY_PRESSED_G) {
		if(keyGLockFlag == false) {
			targetAngle = 15.f/360*8192;	// 快速抬头15°
			Gimbal_PID[GYRO][PITCH_206].Angle.target = constrain(Gimbal_PID[GYRO][PITCH_206].Angle.target - targetAngle, // 抬头Pitch角度减小
															 GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT, 
															 GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT);	// 设置Pitch轴(绝对值)
		}
		keyGLockFlag = true;
	} else {
		keyGLockFlag = false;
	}
}

/**
 *	@brief	鼠标右键进入自瞄模式
 *			松开右键退出自瞄模式
 */
uint8_t test_auto_pid = 0;
static uint8_t mouseRLockFlag = false;	// 鼠标右键锁定
static uint8_t rcSw1LockFlag = false;
static uint8_t keyCtrlLockFlag = false;
static portTickType  keyCurrentTime = 0;	
static uint32_t keyCtrlFLockTime = 0;
static uint32_t keyCtrlVLockTime = 0;
void KEY_setGimbalMode(RC_Ctl_t *remoteInfo)
{
	keyCurrentTime = xTaskGetTickCount();
	
	if(test_auto_pid == 0) {
		/* 鼠标右键 */
		if(IF_MOUSE_PRESSED_RIGH) {
			if(mouseRLockFlag == false && GIMBAL_ifBuffMode() == false) {
				Gimbal.State.mode = GIMBAL_MODE_AUTO;
				VISION_setMode(VISION_MODE_AUTO);
				Gimbal.Auto.FLAG_first_into_auto = true;
			}
			mouseRLockFlag = true;
		} else {
			if(GIMBAL_ifAutoMode() == true) { // 退出自瞄模式
				Gimbal.State.mode = GIMBAL_MODE_NORMAL;
				VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
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
			if(GIMBAL_ifAutoMode() == true) { // 退出自瞄模式
				Gimbal.State.mode = GIMBAL_MODE_NORMAL;
				VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
				Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
				Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;
				Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
				Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;	
			}
			rcSw1LockFlag = false;
		}
	}
	
	/* Ctrl+V组合键 */
	if(IF_KEY_PRESSED_CTRL) {
		if(keyCtrlLockFlag == false) {
			Gimbal.State.mode = GIMBAL_MODE_NORMAL;
			VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
			Flag.gimbal.FLAG_pidMode = MECH;	// 强制进入机械模式
			GIMBAL_keyGyro_To_keyMech();
		}
		if(keyCurrentTime > keyCtrlVLockTime) {	
			keyCtrlVLockTime = keyCurrentTime + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_V) {	// Ctrl+V
				if(Gimbal.State.mode != GIMBAL_MODE_SMALL_BUFF) {
					Gimbal.State.mode = GIMBAL_MODE_SMALL_BUFF;
					VISION_setMode(VISION_MODE_SMALL_BUFF);	// 击打小符
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.gimbal.FLAG_pidMode = GYRO;	// 强制进入陀螺仪模式
					GIMBAL_keyMech_To_keyGyro();
				}
			}
		}
		if(keyCurrentTime > keyCtrlFLockTime) {
			keyCtrlFLockTime = keyCurrentTime + TIME_STAMP_400MS;
			if(IF_KEY_PRESSED_F) {	// Ctrl+F
				if(Gimbal.State.mode != GIMBAL_MODE_BIG_BUFF) {
					Gimbal.State.mode = GIMBAL_MODE_BIG_BUFF;
					VISION_setMode(VISION_MODE_BIG_BUFF);	// 击打大符
					Gimbal.Buff.FLAG_first_into_buff = true;
					Flag.gimbal.FLAG_pidMode = GYRO;	// 强制进入陀螺仪模式
					GIMBAL_keyMech_To_keyGyro();
				}
			}
		}
		keyCtrlLockFlag = true;
	} else {
		if(keyCtrlLockFlag == true && Gimbal.State.mode == GIMBAL_MODE_NORMAL) {
			Gimbal.State.mode = GIMBAL_MODE_NORMAL;
			VISION_setMode(VISION_MODE_MANUAL);	// 手动模式
			Flag.gimbal.FLAG_pidMode = GYRO;	// 强制进入陀螺仪模式
			GIMBAL_keyMech_To_keyGyro();
		}
		keyCtrlLockFlag = false;
	}


}

/* #云台# -----------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	云台电机初始化
 *	@note		
 */
void GIMBAL_init(void)
{
	GIMBAL_kalmanCreate();	// 创建卡尔曼滤波器
	GIMBAL_GYRO_calAverageOffset(Mpu_Info);	// 陀螺仪角速度误差补偿值计算	
}

/**
 *	@brief	云台电机复位
 *	@note	# 先用机械模式归中
 *			# 具备5s的超时退出机制，防止云台没有复位到位一直卡死	
 */

void GIMBAL_reset(void)
{
	static uint32_t resetTime = 0;
	static portTickType tickTime_prev = 0;
	static portTickType tickTime_now = 0;
	
	tickTime_now = xTaskGetTickCount();
	if(tickTime_now  - tickTime_prev > TIME_STAMP_6000MS) {	// 保证不断电情况下，下次可用
		Flag.gimbal.FLAG_resetOK = false;
		Cnt.gimbal.CNT_resetOK = 0;
		Flag.gimbal.FLAG_angleRecordStart = 0;	// 重新记录
		//..feedback值是否需要清零?
	}
	tickTime_prev = tickTime_now;
	
	if(Flag.gimbal.FLAG_angleRecordStart == 0) {	// 记录上电时云台机械角度和陀螺仪反馈值
		Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;	
		Flag.gimbal.FLAG_angleRecordStart = 1;
	}	
	
	if(Flag.gimbal.FLAG_resetOK == false) {	// 云台复位未完成
		resetTime++;	// 复位计时
		if(Flag.gimbal.FLAG_pidStart == 1) {
			Flag.gimbal.FLAG_pidMode = MECH;
			/* 平缓地让云台移动到中间，防止上电狂甩 */
			Gimbal_PID[MECH][YAW_205].Angle.target = RAMP_float(GIMBAL_MECH_YAW_ANGLE_MID_LIMIT, Gimbal_PID[MECH][YAW_205].Angle.target, GIMBAL_RAMP_BEGIN_YAW);
			Gimbal_PID[MECH][PITCH_206].Angle.target = RAMP_float(GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT, Gimbal_PID[MECH][PITCH_206].Angle.target, GIMBAL_RAMP_BEGIN_PITCH);
		}		
		
		/* 等待云台归中 */
		if(abs(Gimbal_PID[MECH][YAW_205].Angle.feedback - GIMBAL_MECH_YAW_ANGLE_MID_LIMIT) <= 1 
			&& abs(Gimbal_PID[MECH][PITCH_206].Angle.feedback - GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT) <= 1) {
			Cnt.gimbal.CNT_resetOK++;
		}
			
		/* 复位成功或者超时强制退出(5s) */
		if(Cnt.gimbal.CNT_resetOK > 250 || resetTime >= 2500) {	
			Cnt.gimbal.CNT_resetOK = 0;
			Flag.gimbal.FLAG_resetOK = true;// 云台复位成功
			resetTime = 0;					// 复位计时清零
		}		
	} else if(Flag.gimbal.FLAG_resetOK == true) {	// 云台复位完成
		Flag.gimbal.FLAG_resetOK = false;	// 清除状态标志位
		BM_reset(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_GIMBAL);
		/* 模式切换(刚上电太快切成陀螺仪模式会因为反馈角度还未稳定造成云台甩头) */			
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
 *	@brief	云台陀螺仪模式记录反馈数据
 *	@note
 *					# 处理Pitch角度的临界值问题
 *					1. 枪管向下 - -162.4°
 *					2. 枪管向上 - +145.8°
 *					先±180得到一个小于90°的数值
 *					规定; 
 *					下边为负( > -17°)
 *					上边为正( < +34°)
 *
 *					# 处理Yaw角度的临界值问题
 *						# 法①:
 *							Yaw每次上电都会不一样，本身的数据源自相对坐标系
 *							相同的规律是顺时针角度值一定是减小的
 *					1. 枪管向左 - -59.6°
 *					2. 枪管向右 - +115°
 *					先±180得到一个小于90°的数值
 *						# 法②：
 *					上述方法由于yaw轴陀螺仪数据的漂移导致效果不佳，因此采用机械模式下的角度
 *					反馈来作为限制云台分离的角度(即云台抵达限制角度后只能增加另一个方向的期望)。
 *					
 *				  # 上电后MPU6050数据会有一段时间不稳定(可做等待处理)
 *					# 机械模式下的速度反馈值利用IMU的角速度反馈值
 *						原因：在手动移动云台yaw和pitch方向时。
 *							  # Yaw
 *							  陀螺仪数据的反馈最大可达5000+(不用乘缩放系数)
 *							  而电机的转速反馈最大可达50+
 *							  # Pitch
 *							  陀螺仪数据的反馈最大可达3000+(不用乘缩放系数)
 *							  而电机的转速反馈最大可达30+
 *							  对比可以知道陀螺仪反馈的角速度值精度更高，因此
 *							  陀螺仪模式和机械模式下的速度环均采用IMU的反馈数据
 *
 *					# YAW轴电机	右摆 电机的out为 +
 *								左摆 电机的out为 -
 *					# PITCH电机 抬头 电机的out为 -
 *								低头 电机的out为 +
 *
 *	@direction
 *			IMU方向：
 *					抬头	- ratePitch => -
 *							-	Pitch 	=> + 
 *					低头 	- ratePitch => +
 *							- 	Pitch 	=> -
 *					左摆  	- rateYaw   => -
 *							- 	Yaw   	=> +
 *					右摆  	- rateYaw   => +
 *							- 	Yaw		=> -
 *			规定:
 *					抬头	- ratePitch => -
 *								- Pitch => -(陀螺仪模式) 
 *					低头 	- ratePitch => +
 *								- Pitch => +(陀螺仪模式)
 *					左摆  	- rateYaw   => -
 *								- Yaw   => -(陀螺仪模式)
 *					右摆  	- rateYaw   => +
 *								- Yaw	=> +(陀螺仪模式)
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
	/* 读取MPU6050传感器数据 */
	mpu_dmp_get_data( &Mpu_Info.roll, &Mpu_Info.pitch, &Mpu_Info.yaw);
	MPU_Get_Gyroscope( &Mpu_Info.ratePitch, &Mpu_Info.rateRoll, &Mpu_Info.rateYaw);		
	
	/* # Yaw # */
	yaw = -Mpu_Info.yaw;
	/* # 陀螺仪模式下的角度反馈值利用IMU的角度值 */
	pid[GYRO][YAW_205].Angle.feedback = (int)(yaw * GIMBAL_GYRO_ANGLE_ZOOM_INDEX);
	
	/* # Yaw 速度反馈 # */
	rateYaw = Mpu_Info.rateYaw - Mpu_Info.rateYawOffset;

	//	rateYaw = kLPF_rateYaw*rateYaw + (1-kLPF_rateYaw)*last_rateYaw;
//	last_rateYaw = rateYaw;
//	if(last_rateYaw != 0.f) {	// 抑制偶然很大的噪声
//		if(abs(rateYaw / last_rateYaw) > 50.f)
//			rateYaw = last_rateYaw;
//	}

	/* # 机械模式下的速度反馈值利用IMU的角速度值 */
	pid[MECH][YAW_205].Speed.feedback = rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	/* # 陀螺仪模式下的速度反馈值利用IMU的角速度值 */
	pid[GYRO][YAW_205].Speed.feedback = rateYaw;	// GIMBAL_COMPENSATE_RATEYAW
	

	
	/* # Pitch 速度反馈 # */
	ratePitch = Mpu_Info.ratePitch - Mpu_Info.ratePitchOffset;

//	ratePitch = kLPF_rateYaw*ratePitch + (1-kLPF_ratePitch)*last_ratePitch;
//	last_ratePitch = ratePitch;
//	if(last_ratePitch != 0.f) {	// 抑制偶然很大的噪声
//		if(abs(ratePitch / last_ratePitch) > 50.f)
//			ratePitch = last_ratePitch;
//	}	
//	
	
	/* # Pitch # */
	pitch = -Mpu_Info.pitch;
	/* # 陀螺仪模式下的角度反馈值利用Pitch机械角度反馈值 */
	pid[GYRO][PITCH_206].Angle.feedback = g_Gimbal_Motor_Info[PITCH_206].angle;	
	/* # 机械模式下的速度反馈值利用IMU的角速度值 */
	pid[MECH][PITCH_206].Speed.feedback = ratePitch;	// 抬头速度为-	GIMBAL_COMPENSATE_RATEPITCH
	/* # 陀螺仪模式下的速度反馈值利用IMU的角速度值 */
	pid[GYRO][PITCH_206].Speed.feedback = ratePitch;	// 抬头速度为-	GIMBAL_COMPENSATE_RATEPITCH
}

/**
 *	@brief	云台电机IMU计算误差补偿值
 */
void GIMBAL_GYRO_calAverageOffset(Mpu_Info_t mpuInfo)
{
	uint16_t i;
	for(i = 0; i < 50; i++) {
		delay_us(100);
		// 读取陀螺仪角度和角速度
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
 *	@brief	陀螺仪模式下云台YAW期望值(累加值)边界处理
 */
float GIMBAL_GYRO_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target)
{
	float target;
	target = pid->Angle.target + delta_target;
	if(target >= 180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX) {	// 超过右边界
		target = (-360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
	} else if(target <= -180.0f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX){// 超过左边界
		target = (+360.f*GIMBAL_GYRO_ANGLE_ZOOM_INDEX + target);
	}
	return target;
}

/**
 *	@brief	遥控机械模式 -> 遥控陀螺仪模式
 *	@note	异步切换
 */
void GIMBAL_rcMech_To_rcGyro(void)
{
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);	
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
//	Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// 积分清零
//	Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// 积分清零
	Gimbal_PID[MECH][YAW_205].Out = 0;
	Gimbal_PID[MECH][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	遥控陀螺仪模式 -> 遥控机械模式
 *	@note	异步切换
 */
void GIMBAL_rcGyro_To_rcMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
//	Gimbal_PID[MECH][YAW_205].Speed.integrate = 0;	// 积分清零
//	Gimbal_PID[MECH][PITCH_206].Speed.integrate = 0;// 积分清零	
	Gimbal_PID[GYRO][YAW_205].Out = 0;
	Gimbal_PID[GYRO][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	遥控机械模式 -> 键盘模式
 *	@note	异步切换
 */
void GIMBAL_rcMech_To_keyGyro(void)
{
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);	
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
//	Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// 积分清零
//	Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// 积分清零
	Gimbal_PID[MECH][YAW_205].Out = 0;
	Gimbal_PID[MECH][PITCH_206].Out = 0;	
	Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	键盘模式 -> 遥控机械模式
 *	@note	异步切换
 */
void GIMBAL_keyGyro_To_rcMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = Gimbal_PID[MECH][YAW_205].Angle.feedback;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
//	Gimbal_PID[MECH][YAW_205].Speed.integrate = 0;	// 积分清零
//	Gimbal_PID[MECH][PITCH_206].Speed.integrate = 0;// 积分清零
//	Gimbal_PID[GYRO][YAW_205].Out = 0;
//	Gimbal_PID[GYRO][PITCH_206].Out = 0;
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	键盘陀螺仪模式 -> 键盘机械模式
 *	@note	异步切换
 */
void GIMBAL_keyGyro_To_keyMech(void)
{
	Gimbal_PID[MECH][YAW_205].Angle.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;
	Gimbal_PID[MECH][PITCH_206].Angle.target = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	Gimbal_PID[MECH][YAW_205].Speed.integrate = 0;	// 积分清零
	Gimbal_PID[MECH][PITCH_206].Speed.integrate = 0;// 积分清零
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	键盘机械模式 -> 键盘陀螺仪模式
 *	@note	异步切换
 */
void GIMBAL_keyMech_To_keyGyro(void)
{
	Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
	Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
	Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// 积分清零
	Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// 积分清零
	//Gimbal.State.mode = GIMBAL_MODE_NORMAL;
}

/**
 *	@brief	目标速度解算
 *	@note	利用两帧之间的角度差算出目标的速度
 */
float speed_calculate(Speed_Calculate_t *info, uint32_t time, float position)
{
	info->delay_cnt++;
	if(time != info->last_time)
	{
		info->speed = (position - info->last_position)/(time - info->last_time)*2;	// 帧差法计算速度(*2感觉是速度放大的意思)
		
		info->processed_speed = info->speed;
			
		info->last_time = time;
		info->last_position = position;
		info->last_speed = info->speed;
		info->delay_cnt = 0;
	}
	
	/* 由于时间更新是在视觉数据更新里面进行的，因此如果视觉失联超过一定时间后将反馈速度清零 */
	if(info->delay_cnt > 250)	// 500ms
	{
		info->processed_speed = 0;
	}
	
	return info->processed_speed;
}

/**
 *	@brief	自瞄模式PID计算
 *	@note	3.75f -> 射速提高之后需要重新计算一个补偿值
 */
float GIMBAL_AUTO_PITCH_COMPENSATION = (0.f/360.0f*8192.0f);
float auto_yaw_ramp = 260;
float auto_pitch_ramp = 80;
void GIMBAL_VISION_AUTO_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	/* 计算自瞄帧率 */
	gimbal->Auto.Time[NOW] = xTaskGetTickCount();
	gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

//	/* >60ms 则认为丢失目标后再识别到 */
//	if(gimbal->Auto.Time[DELTA] > TIME_STAMP_60MS) {
//		pid[GYRO][YAW_205].Speed.integrate = 0;
//		pid[GYRO][PITCH_206].Speed.integrate = 0;
//	}
	
	if(gimbal->Auto.FLAG_first_into_auto == true) {
		gimbal->Auto.FLAG_first_into_auto = false;
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;	
	}
	
	/* 数据更新 */
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw 误差计算 */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_getYawFeedback();
			//gimbal->Auto.Yaw.erro = KalmanFilter(&Gimbal_Auto_kalmanError[YAW_205], gimbal->Auto.Yaw.erro);
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
			/* Pitch 误差计算 */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_getPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
			//gimbal->Auto.Pitch.erro = KalmanFilter(&Gimbal_Auto_kalmanError[PITCH_206], gimbal->Auto.Pitch.erro);
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
		} else {
			/* 可能存在问题 */
			gimbal->Auto.Yaw.erro = 0;
			gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback;
			gimbal->Auto.Pitch.erro = 0;
			gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback;
		}
	}
	
	/*----期望修改----*/
	pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
	gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
}

/**
 *	@brief	自瞄模式电控预测版
 *	@note	云台右摆(目标右移) - 目标速度为正
 *					   			目标角度为正
 */
Speed_Calculate_t	yaw_angle_speed_struct, pitch_angle_speed_struct;
float yaw_angle_raw, pitch_angle_raw;
float *yaw_kf_result, *pitch_kf_result;	// 二阶卡尔曼滤波结果,0角度 1速度
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
	
	/*----数据更新----*/
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	

		/* 计算自瞄帧率 */
		gimbal->Auto.Time[NOW] = xTaskGetTickCount();
		gimbal->Auto.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];
		gimbal->Auto.Time[PREV] = gimbal->Auto.Time[NOW];
		
		/* 识别到目标 */
		if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {
			/* Yaw 误差放大 */
			gimbal->Auto.Yaw.erro = gimbal->Auto.Yaw.kp * VISION_GYRO_getYawFeedback();
			/* Pitch 误差放大 */
			gimbal->Auto.Pitch.erro = (gimbal->Auto.Pitch.kp * VISION_MECH_getPitchFeedback()) - GIMBAL_AUTO_PITCH_COMPENSATION;
		} 
		/* 未识别到目标 */
		else {
			/* Yaw 误差为0 */
			gimbal->Auto.Yaw.erro = 0;
			/* Pitch 误差为0 */
			gimbal->Auto.Pitch.erro = 0;
		}
		/* Yaw 目标计算 */
		gimbal->Auto.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Auto.Yaw.erro;
		/* Pitch 目标计算 */
		gimbal->Auto.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;
	}
	
	/*----更新二阶卡尔曼预测值----*/
	/* 识别到目标 */
	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		/* 更新二阶卡尔曼速度先验估计值 */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Yaw.target);
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, gimbal->Auto.Time[NOW], gimbal->Auto.Pitch.target);

		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, yaw_angle_speed);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, pitch_angle_speed);
	} 
	/* 未识别到目标 */
	else {
		/* 更新二阶卡尔曼速度先验估计值 */
		yaw_angle_speed = speed_calculate(&yaw_angle_speed_struct, xTaskGetTickCount(), gimbal->Auto.Yaw.target);	
		pitch_angle_speed = speed_calculate(&pitch_angle_speed_struct, xTaskGetTickCount(), gimbal->Auto.Pitch.target);

		/* 对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度 */
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, 0);		// 识别不到时认为目标速度为0
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, 0);	// 识别不到时认为目标速度为0
	}
	
	js_yaw_speed = yaw_kf_result[KF_SPEED]*100;
	js_yaw_angle = yaw_kf_result[KF_ANGLE]*100;
	
	
	/*----预测量计算----*/
	/* 识别到目标 */
	if(VISION_getFlagStatus(VISION_FLAG_LOCK_TARGET) == true) {	
		auto_predict_start_delay++;	// 滤波延时开启
		
		if(auto_predict_start_delay > auto_predict_start_delay_boundary 
				&& abs(gimbal->Auto.Yaw.erro) < auto_yaw_predict_max 
					&& abs(yaw_kf_result[KF_SPEED]) >=  auto_yaw_speed_predict_low 
						&& abs(yaw_kf_result[KF_SPEED]) <=  auto_yaw_speed_predict_high )
		{
			/* 目标左移 -> 预测量为负(云台左移) */
			if(yaw_kf_result[KF_SPEED] < 0) {
				yaw_angle_predict = yaw_predict_k * yaw_kf_result[KF_SPEED]; // 时间常数2ms归入预测系数中
			}
		}
	}
	/* 未识别到目标 */
	else {
	
	}
	
	/*----期望修改----*/
	pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Auto.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, auto_yaw_ramp);
	pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Auto.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, auto_pitch_ramp);	
//	pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
//	pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&pid[GYRO][YAW_205], gimbal->Auto.Yaw.erro);
//	pid[GYRO][PITCH_206].Angle.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Auto.Pitch.erro;	
	
}

/**
 *	@brief	打符模式PID计算
 *	@note	红方打红符、蓝方打蓝符
 *			两个阶段：
 *			①小符
 *				简介：5:59~4:00允许打符，打符成功则获得1.5倍攻击，持续1分钟
 *				运动策略：10RPM
 *			②大符
 *				简介：2:59~0:00允许打符，打符成功则获得2倍攻击，50%防御加成，持续1分钟
 *			
 *			地形信息：
 *			①桥面离地1米2,大风车中心离地2米283,内径70cm,外径80cm
 *			②能量机关激活点
 *				能量机关可激活状态时，己方机器人占领并停留3s，枪口热量每秒冷却值变为5倍
 *				2.5s未击中/击中其它装甲板 -》激活失败
 *
 *			控制思路：
 *			进入打符模式的时候，云台斜坡归中(以视觉反馈的坐标原点为中心)
 *
 *			# 在家测试条件：
 *			投影直线距离：7.40m
 *			大符中心距离地面：1.55m
 *			枪管距离大符中心：7.56m(勾股定理)
 *	
 *			# 摄像头在云台（摄像头在底盘的可参考往年代码）
 */

void GIMBAL_VISION_BUFF_pidCalculate(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal)
{
	static float yaw_gyro_mid, yaw_mech_mid;
	static float pitch_gyro_mid, pitch_mech_mid;
	static uint16_t into_buff_time = 0;
	static uint16_t lost_cnt = 0;
	/* 计算打符帧率 */
	gimbal->Buff.Time[NOW] = xTaskGetTickCount();
	gimbal->Buff.Time[DELTA] = gimbal->Auto.Time[NOW] - gimbal->Auto.Time[PREV];

	/* 切换pitch补偿角 */
	if((GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT - pid[GYRO][PITCH_206].Angle.feedback) > 0)
		GIMBAL_BUFF_PITCH_COMPENSATION = BUFF_PITCH_MODIFY_TABLE[(uint8_t)((GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT - pid[GYRO][PITCH_206].Angle.feedback)/100)];
	else
		GIMBAL_BUFF_PITCH_COMPENSATION = 0;
	
	if(gimbal->Buff.FLAG_first_into_buff == true) {
		gimbal->Buff.FLAG_first_into_buff = false;
		into_buff_time = 0;
		/* 记录刚进入打符模式时的陀螺仪角度 */
		yaw_gyro_mid = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		pitch_gyro_mid = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;
		/* 记录刚进入打符模式时的机械角度 */
		yaw_mech_mid = Gimbal_PID[MECH][YAW_205].Angle.feedback;
		pitch_mech_mid = Gimbal_PID[MECH][PITCH_206].Angle.feedback;
	} else {
		/* 防止加得太大 */
		if(into_buff_time < 250)	
			into_buff_time++;
	}
	
	/* 数据更新 */
	if(VISION_getFlagStatus(VISION_FLAG_DATA_UPDATE) == true) {
		/* 清除数据更新标志位，防止未接收到新数据的情况下重复进入 */
		VISION_clearFlagStatus(VISION_FLAG_DATA_UPDATE);	
		if(VISION_getFlagStatus(VISION_FLAG_LOCK_BUFF) == true) {	// 识别到目标则更新误差
			lost_cnt = 0;
			if(into_buff_time > 100) {
				/* Yaw 误差计算 */
				gimbal->Buff.Yaw.erro = (gimbal->Buff.Yaw.kp * VISION_BUFF_getYawFeedback()) + GIMBAL_BUFF_YAW_COMPENSATION;
				gimbal->Buff.Yaw.target = pid[GYRO][YAW_205].Angle.feedback + gimbal->Buff.Yaw.erro;
				/* Pitch 误差计算 */
				gimbal->Buff.Pitch.erro = (gimbal->Buff.Pitch.kp * VISION_BUFF_getPitchFeedback()) + GIMBAL_BUFF_PITCH_COMPENSATION;
				gimbal->Buff.Pitch.target = pid[GYRO][PITCH_206].Angle.feedback + gimbal->Buff.Pitch.erro;
				/*----期望修改----*/
				pid[GYRO][YAW_205].Angle.target = RAMP_float(gimbal->Buff.Yaw.target, pid[GYRO][YAW_205].Angle.feedback, GIMBAL_BUFF_YAW_RAMP);
				pid[GYRO][PITCH_206].Angle.target = RAMP_float(gimbal->Buff.Pitch.target, pid[GYRO][PITCH_206].Angle.feedback, GIMBAL_BUFF_PITCH_RAMP);				
			} else {
				pid[GYRO][YAW_205].Angle.target = yaw_gyro_mid;
				pid[GYRO][PITCH_206].Angle.target = pitch_gyro_mid;
			}
		} else {	// 未识别到目标则保持原位置
			gimbal->Buff.Yaw.erro = 0;
			gimbal->Buff.Pitch.erro = 0;
			if(lost_cnt > 50) {	// 连续50帧丢失
//				pid[GYRO][YAW_205].Angle.target = yaw_gyro_mid;
//				pid[GYRO][PITCH_206].Angle.target = pitch_gyro_mid;	
				if(lost_cnt == 51) {	// 只赋值一次
					pid[GYRO][YAW_205].Angle.target = pid[GYRO][YAW_205].Angle.feedback;
					pid[GYRO][YAW_205].Angle.target = GIMBAL_GYRO_yawTargetBoundaryProcess(&Gimbal_PID[GYRO][YAW_205], 
													((yaw_mech_mid - Gimbal_PID[MECH][YAW_205].Angle.feedback)*360/8192.f));
					pid[GYRO][PITCH_206].Angle.target = pitch_mech_mid;	// Pitch采用机械模式
					lost_cnt++;
				}
			} else {
				lost_cnt++;
			}
		}
	}
	
	/*----期望修改----*/
	//..自动打弹
	
	gimbal->Buff.Time[PREV] = gimbal->Buff.Time[NOW];
}

/**
 *	@brief	云台常规控制
 */
void GIMBAL_normalControl(void)
{
	KEY_setGimbalAngle(&RC_Ctl_Info);
	KEY_setGimbalTurn(&RC_Ctl_Info);
	KEY_setQuickPickUp(&RC_Ctl_Info);	
}

/**
 *	@brief	云台自瞄控制
 */
void GIMBAL_autoControl(void)
{
	/* 视觉数据可用 && 键盘模式下 */
	if( (VISION_isDataValid()) && (Flag.remote.FLAG_mode == KEY) ) {
		/*----期望修改----*/
		/* 视觉预测版 */
		//GIMBAL_VISION_AUTO_pidCalculate(Gimbal_PID, &Gimbal);
		/* 电控预测版 */
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
 *	@brief	云台打符控制
 *	@note	
 *	1.先把一块装甲常亮放到最下面，调红外，调offsetx和offsety把红外点到装甲板中心。
 *
 *	2.旋转你的装甲板，看你的红外是不是一个完整的圆，就是说红外在你的距离给定的时候，不管装甲板旋转那边，红外点一直跟着。
 *
 *	3.调抬头补偿。
 *
 *	4.这个时候因为红外与枪管经常不平衡，所以打弹，把offsetx与offsety调到打中。
 *
 *	5.各个位置试着打。
 *
 *	6.加旋转。
 *	
 *	7.pid调试心得：pitch和yaw都要特别硬，可以加很大的积分（注释掉反向积分清零的操作会稳定一点）
 */
void GIMBAL_buffControl(void)
{
	/* 按下WSADQEV(任意方向键)则退出打符模式 */
	if((IF_KEY_PRESSED_W || IF_KEY_PRESSED_S || IF_KEY_PRESSED_A || IF_KEY_PRESSED_D
		|| IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_V)&&(!IF_KEY_PRESSED_CTRL)) 
	{
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;	// 进入正常模式
		Flag.gimbal.FLAG_pidMode = GYRO;		// 进入陀螺仪模式
		Gimbal_PID[GYRO][YAW_205].Angle.target = Gimbal_PID[GYRO][YAW_205].Angle.feedback;
		Gimbal_PID[GYRO][PITCH_206].Angle.target = Gimbal_PID[GYRO][PITCH_206].Angle.feedback;	// GIMBAL_GYRO_PITCH_ANGLE_MID_LIMIT
		Gimbal_PID[GYRO][YAW_205].Speed.integrate = 0;	// 积分清零
		Gimbal_PID[GYRO][PITCH_206].Speed.integrate = 0;// 积分清零
		REVOLVER_setAction(SHOOT_NORMAL);	// 变成常规控制模式
	}
	
	/* 视觉数据可用 && 键盘模式下 */
	if( (VISION_isDataValid()) && (Flag.remote.FLAG_mode == KEY) ) {
		/*----期望修改----*/
		GIMBAL_VISION_BUFF_pidCalculate(Gimbal_PID, &Gimbal);	

	}
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	pid控制器最终输出
 */
uint8_t test_yaw_pid = 0;
uint8_t test_pitch_pid = 0;
float   test_yaw_speed_max_target = 8000;
float   test_pitch_speed_max_target = 4000;
void GIMBAL_pidControlTask(void)
{
	/* YAW 角度环 */
	GIMBAL_Angle_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], YAW_205);
	/* YAW 速度环 */
	if(test_yaw_pid == 0) {
			Gimbal_PID[Flag.gimbal.FLAG_pidMode][YAW_205].Speed.target = Gimbal_PID[Flag.gimbal.FLAG_pidMode][YAW_205].Angle.out;
			GIMBAL_SPEED_PID_IOUT_MAX = 18000;
	} else {
		Gimbal_PID[Flag.gimbal.FLAG_pidMode][YAW_205].Speed.target = (RC_Ctl_Info.rc.ch0 - 1024)/660.f * test_yaw_speed_max_target;
		GIMBAL_SPEED_PID_IOUT_MAX = 18000;
	}
	GIMBAL_Speed_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], YAW_205);
	
	/* PITCH 角度环 */
	GIMBAL_Angle_pidCalculate(Gimbal_PID[Flag.gimbal.FLAG_pidMode], PITCH_206);
	/* PITCH 速度环 */
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
 *	@brief	遥控控制云台
 */
void GIMBAL_rcControlTask(void)
{
	REMOTE_setGimbalAngle(&RC_Ctl_Info);	
}

/**
 *	@brief	键盘控制云台
 */
void GIMBAL_keyControlTask(void)
{
	/* 设置云台的模式 */
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
 *	@brief	云台失控保护
 */
void GIMBAL_selfProtect(void)
{	
	GIMBAL_stop(Gimbal_PID[MECH]);
	GIMBAL_stop(Gimbal_PID[GYRO]);
	GIMBAL_pidParamsInit(Gimbal_PID[MECH], GIMBAL_MOTOR_COUNT);
	GIMBAL_pidParamsInit(Gimbal_PID[GYRO], GIMBAL_MOTOR_COUNT);	
}

/**
 *	@brief	云台控制
 */
void GIMBAL_control(void)
{
	/*----信息读入----*/
	//GIMBAL_IMU_recordFeedback(Gimbal_PID);

	/*----期望修改----*/
	if(BM_ifSet(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_GIMBAL)) {	// 复位状态
		Gimbal.State.mode = GIMBAL_MODE_NORMAL;
		GIMBAL_reset(); // 云台复位
	} else {
		if(Flag.remote.FLAG_mode == RC) {
			GIMBAL_rcControlTask();
		} else if(Flag.remote.FLAG_mode == KEY) {
			GIMBAL_keyControlTask();
		}
	}

	/* 根据云台模式切换PID参数 */
	GIMBAL_pidParamsSwitch(Gimbal_PID, &Gimbal);

	/*----最终输出----*/
	GIMBAL_pidControlTask();
}
