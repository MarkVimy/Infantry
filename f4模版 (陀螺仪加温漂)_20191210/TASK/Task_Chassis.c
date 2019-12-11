/**
 * @file        Task_Chassis.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        30-September-2019
 * @brief       This file includes the Chassis(底盘) external functions 
 *
 * @Verison			V1.1 (1-October-2019)
 */

/**
 *	PID调参口诀
 *	kp大 -> 响应快 -> 超调
 *	ki大 -> 消除静差 -> 
 *	参考网址：https://www.jianshu.com/p/4b0fa85cd353
 */

/* Includes ------------------------------------------------------------------*/
#include "Task_Chassis.h"

#include "arm_math.h"
#include "kalman.h"
#include "can.h"

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 卡尔曼滤波器 */
extKalman_t Chassis_Kalman_Error;

/* ## Global variables ## ----------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
float speed_target_203;
float speed_feedback_203;
float angle_target_203;
float angle_feedback_203;

/**
 *	@brief	底盘PID
 */
Chassis_PID_t Chassis_PID[CHASSIS_MOTOR_COUNT] = {
	{	// LEFT_FRON_201
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8			11.2
		.Speed.ki = 236,		// 243			238
		.Speed.kd = 0.0112,	// 0.0112		0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 0,
		.Angle.ki = 0,
		.Angle.kd = 0,
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
	{	// RIGH_FRON_202
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8
		.Speed.ki = 236,		// 243
		.Speed.kd = 0.0112,	// 0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 0,
		.Angle.ki = 0,
		.Angle.kd = 0,
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
	{	// LEFT_BACK_203
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8
		.Speed.ki = 236,		// 243
		.Speed.kd = 0.0112,	// 0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 1.172,		// 1.15					1.16				1.172		(这套参数适合大角度转动)
		.Angle.ki = 0.7,			// 1						0.82					0.7
		.Angle.kd = 0.0021,		// 0.002				0.0021			0.0021
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
	{	// RIGH_BACK_204
		/* 速度环 */
		.Speed.kp = 11.5,		// 10.8
		.Speed.ki = 236,		// 243
		.Speed.kd = 0.0112,	// 0.0112
		.Speed.target = 0,
		.Speed.feedback = 0,
		.Speed.erro = 0,
		.Speed.last_erro = 0,
		.Speed.integrate = 0,
		.Speed.pout = 0,
		.Speed.iout = 0,
		.Speed.dout = 0,
		.Speed.out = 0,
		/* 位置环 */
		.Angle.kp = 0,
		.Angle.ki = 0,
		.Angle.kd = 0,
		.Angle.target = 0,
		.Angle.feedback = 0,
		.Angle.erro = 0,
		.Angle.last_erro = 0,
		.Angle.integrate = 0,
		.Angle.pout = 0,
		.Angle.iout = 0,
		.Angle.dout = 0,
		.Angle.out = 0,		
	},
};

/**
 *	@brief	底盘Z方向Pid
 */
PID_Object_t Chassis_Z_Speed_PID = {
		.kp = 0.48,
		.ki = 0,
		.kd = 0,
		.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,
		.feedback = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT,
		.erro = 0,
		.last_erro = 0,
		.integrate = 0,
		.pout = 0,
		.iout = 0,
		.dout = 0,
		.out = 0,
};

/**
 *	@brief	底盘功率
 */
Chassis_Power_t Chassis_Power = {
	.maxLimit = CHASSIS_MAX_CURRENT_LIMIT,
	.currentLimit = 0,
};

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘电机PID参数初始化
 */
void Chassis_PID_ParamsInit(Chassis_PID_t *pid, uint8_t motor_cnt)
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

		pid[i].Angle.target = 0;
		pid[i].Angle.feedback = 0,
		pid[i].Angle.erro = 0;
		pid[i].Angle.last_erro = 0;
		pid[i].Angle.integrate = 0;
		pid[i].Angle.pout = 0;
		pid[i].Angle.dout = 0;
		pid[i].Angle.iout = 0;
		pid[i].Angle.out = 0;
	}
}

/**
 *	@brief	底盘陀螺仪模式参数重置
 */
void Chassis_Z_Speed_PID_ParamsInit(PID_Object_t *pid)
{
		pid->target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;
		pid->feedback = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;
		pid->erro = 0;
		pid->last_erro = 0;
		pid->integrate = 0;
		pid->pout = 0;
		pid->iout = 0;
		pid->dout = 0;
		pid->out = 0;
}

/**
 *	@brief	底盘陀螺仪模式卡尔曼滤波器初始化
 */
void CHASSIS_kalmanCreate(void)
{
	KalmanCreate(&Chassis_Kalman_Error, 1, 0);
}	

/**
 *	@brief	底盘电机更新实时速度
 */
void Chassis_updateMotorSpeed(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	底盘电机更新实时角度(累积转过的角度)
 */
void Chassis_updateMotorAngle(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
  //pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle;
	pid[MOTORx].Angle.feedback = g_Chassis_Motor_Info[MOTORx].angle_sum;
}

/**
 *	@brief	底盘电机更新实时电流值
 */
void Chassis_updateMotorCurrent(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	//pid[MOTORx].Speed.feedback = g_Chassis_Motor_Info[MOTORx].speed;
}

/**
 *	@brief	底盘电机紧急刹车
 */
void Chassis_stop(Chassis_PID_t *pid)
{
	static float pid_out[4] = {0, 0, 0, 0};
	
	/* 内环速度环最终输出 */
	pid[LEFT_FRON_201].Out = 0;
	pid[RIGH_FRON_202].Out = 0;
	pid[LEFT_BACK_203].Out = 0;
	pid[RIGH_BACK_204].Out = 0;
	
	CAN1_send(0x200, pid_out);	
}

/**
 *	@brief	底盘电机速度环
 */
void Chassis_Speed_PID_calculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Speed.erro = pid[MOTORx].Speed.target - pid[MOTORx].Speed.feedback;
	pid[MOTORx].Speed.integrate += pid[MOTORx].Speed.erro;
	//pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, -3000, 3000);
	// #integrate是否限幅?
	
	/* Pout */
	pid[MOTORx].Speed.pout = pid[MOTORx].Speed.kp * pid[MOTORx].Speed.erro;
	/* Iout */
	pid[MOTORx].Speed.iout = pid[MOTORx].Speed.ki * pid[MOTORx].Speed.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid[MOTORx].Speed.iout = constrain(pid[MOTORx].Speed.iout, -CHASSIS_SPEED_IOUT_MAX, CHASSIS_SPEED_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Speed.dout = pid[MOTORx].Speed.kd * (pid[MOTORx].Speed.erro - pid[MOTORx].Speed.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Speed.last_erro = pid[MOTORx].Speed.erro;
	
	/* 19步兵代码 - 积分项缓慢减小，防止误差为0时突然失力 */
	if( pid[MOTORx].Speed.pout * pid[MOTORx].Speed.iout < 0) {
		pid[MOTORx].Speed.integrate = constrain(pid[MOTORx].Speed.integrate, 
												-CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Speed.ki/5.f,
												+CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Speed.ki/5.f);
	}
	
	/* Total PID Output*/
	pid[MOTORx].Speed.out = pid[MOTORx].Speed.pout + pid[MOTORx].Speed.iout + pid[MOTORx].Speed.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Speed.out = constrain(pid[MOTORx].Speed.out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
	pid[MOTORx].Out = pid[MOTORx].Speed.out;
	
	/* test */
	speed_target_203 = pid[LEFT_BACK_203].Speed.target;
	speed_feedback_203 = pid[LEFT_BACK_203].Speed.feedback;
}

/**
 *	@brief	底盘电机位置环
 *	@note
 *			串环PID
 *			内环 期望值 期望角速度(实际上就是速度环)
 *					 输出值 期望角加速度
 *
 *			外环 期望值 期望角度
 *					 输出值 期望角速度
 */
void Chassis_Angle_PID_calculate(Chassis_PID_t *pid, Chassis_Motor_Names MOTORx)
{
	pid[MOTORx].Angle.erro = pid[MOTORx].Angle.target - pid[MOTORx].Angle.feedback;
	pid[MOTORx].Angle.integrate += pid[MOTORx].Angle.erro;
	//pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, -3000, 3000);
	// #integrate是否限幅?
	
	/* Pout */
	pid[MOTORx].Angle.pout = pid[MOTORx].Angle.kp * pid[MOTORx].Angle.erro;
	/* Iout */
	pid[MOTORx].Angle.iout = pid[MOTORx].Angle.ki * pid[MOTORx].Angle.integrate * 0.002f; // 模仿19步兵代码(0.002f是2ms的意思)
	/* Iout Limits */
	pid[MOTORx].Angle.iout = constrain(pid[MOTORx].Angle.iout, -CHASSIS_ANGLE_IOUT_MAX, CHASSIS_ANGLE_IOUT_MAX);
	/* Dout*/
	pid[MOTORx].Angle.dout = pid[MOTORx].Angle.kd * (pid[MOTORx].Angle.erro - pid[MOTORx].Angle.last_erro)/0.002f;
	/* Record Last Error */
	pid[MOTORx].Angle.last_erro = pid[MOTORx].Angle.erro;
	
	/* 19步兵代码 - 积分项缓慢减小，防止误差为0时突然失力 */
	if( pid[MOTORx].Angle.pout * pid[MOTORx].Angle.iout < 0) {
		pid[MOTORx].Angle.integrate = constrain(pid[MOTORx].Angle.integrate, 
												-CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Angle.ki/5.f,
												+CHASSIS_SPEED_IOUT_MAX/pid[MOTORx].Angle.ki/5.f);
	}
	
	/* Total PID Output*/
	pid[MOTORx].Angle.out = pid[MOTORx].Angle.pout + pid[MOTORx].Angle.iout + pid[MOTORx].Angle.dout;
	/* Total PID Output Limits */
	pid[MOTORx].Angle.out = constrain(pid[MOTORx].Angle.out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);
	
	/* test */
	angle_target_203 = pid[LEFT_BACK_203].Angle.target;
	angle_feedback_203 = pid[LEFT_BACK_203].Angle.feedback;
}

/**
 *	@brief	底盘陀螺仪模式加上Z方向的速度环
 *	@note
 *				将当前YAW机械角和中值YAW机械角的差值作为误差送进 Z_Speed PID控制器。
 *				类似于给出Z旋转速度的遥控值，
 */
float Chassis_Z_Speed_PID_calculate(PID_Object_t *pid)
{
	pid->erro = pid->target - pid->feedback;
	//pid->erro = KalmanFilter(&Chassis_Kalman_Error, pid->erro);
	
	// 死区控制算法
	if(abs(pid->erro) < 15) {
		pid->erro = 0;
	}
	
	/* Pout */
	pid->pout = pid->kp * pid->erro;
	/* Dout*/
	pid->dout = pid->kd * (pid->erro - pid->last_erro)/0.002f;
	/* Record Last Error */
	pid->last_erro = pid->erro;
	
	/* Total PID Output*/
	pid->out = pid->pout + pid->dout;
	/* Total PID Output Limits */
	pid->out = constrain(pid->out, -CHASSIS_PID_OUT_MAX, CHASSIS_PID_OUT_MAX);	
	return pid->out;
}

/**
 *	@brief	底盘电机PID的最终输出
 */
void Chassis_PID_out(Chassis_PID_t *pid)
{
	float pidOut[4];
	/* 速度环 + 位置环 */
//	pid_out[LEFT_FRON_201] = Chassis_PID[LEFT_FRON_201].Speed.out + Chassis_PID[LEFT_FRON_201].Angle.out;
//	pid_out[RIGH_FRON_202] = Chassis_PID[RIGH_FRON_202].Speed.out + Chassis_PID[RIGH_FRON_202].Angle.out;
//	pid_out[LEFT_BACK_203] = Chassis_PID[LEFT_BACK_203].Speed.out + Chassis_PID[LEFT_BACK_203].Angle.out;
//	pid_out[RIGH_BACK_204] = Chassis_PID[RIGH_BACK_204].Speed.out + Chassis_PID[RIGH_BACK_204].Angle.out;
	/* 内环速度环最终输出 */
//	pid[LEFT_FRON_201].Out = pid[LEFT_FRON_201].Speed.out;
//	pid[RIGH_FRON_202].Out = pid[RIGH_FRON_202].Speed.out;
//	pid[LEFT_BACK_203].Out = pid[LEFT_BACK_203].Speed.out;
//	pid[RIGH_BACK_204].Out = pid[RIGH_BACK_204].Speed.out;
	
	pidOut[LEFT_FRON_201] = pid[LEFT_FRON_201].Out;
	pidOut[RIGH_FRON_202] = pid[RIGH_FRON_202].Out;
	pidOut[LEFT_BACK_203] = pid[LEFT_BACK_203].Out;
	pidOut[RIGH_BACK_204] = pid[RIGH_BACK_204].Out;
	
	CAN1_send(0x200, pidOut);
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	底盘初始化
 */
void CHASSIS_init(void)
{
	CHASSIS_kalmanCreate();	// 创建卡尔曼滤波器
}

/**
 *	@brief	底盘功率限制(期望再分配)
 *	@note	祖传算法
 */
void CHASSIS_powerLimit(Chassis_Power_t *power, Chassis_PID_t *pid, Judge_Info_t *judge_info)
{
	float kLimit;
	float totalOutput;
	float remain_J;
	static uint16_t judge_err_cnt;
	
	// 读取缓冲焦耳能量
	remain_J = judge_info->PowerHeatData.chassis_power_buffer;	
	totalOutput = abs(pid[LEFT_FRON_201].Out) + 
				  abs(pid[RIGH_FRON_202].Out) + 
				  abs(pid[LEFT_BACK_203].Out) + 
				  abs(pid[RIGH_BACK_204].Out);
	
	if(judge_info->data_valid == false) {
		judge_err_cnt++;
		if(judge_err_cnt > 100) {
			power->currentLimit = 9000;	// 最大的1/4
		}
	} else {
		judge_err_cnt = 0;
		// 剩余焦耳量过小,开始限制输出,限制系数为平方关系
		if(remain_J < WARNING_REMAIN_POWER) {
			kLimit = (float)(remain_J / WARNING_REMAIN_POWER)
						* (float)(remain_J / WARNING_REMAIN_POWER);
			power->currentLimit =  kLimit * power->maxLimit;
		} else {	// 焦耳能量恢复到一定数值
			power->currentLimit = power->maxLimit;
		}
	}
	
	if(totalOutput > power->currentLimit) {
		pid[LEFT_FRON_201].Out = (int16_t)pid[LEFT_FRON_201].Out / totalOutput * power->currentLimit;
		pid[RIGH_FRON_202].Out = (int16_t)pid[RIGH_FRON_202].Out / totalOutput * power->currentLimit;
		pid[LEFT_BACK_203].Out = (int16_t)pid[LEFT_BACK_203].Out / totalOutput * power->currentLimit;
		pid[RIGH_BACK_204].Out = (int16_t)pid[RIGH_BACK_204].Out / totalOutput * power->currentLimit;	
	}
}

/**
 *	@brief	底盘扭腰粗糙版
 */
float CHASSIS_twistTargetCalculate(int16_t maxTarget, int16_t rampTarget)
{
	static uint8_t dir = 1;
	static float target_z_speed = 0;
	static portTickType tickTime_prev = 0;
	static portTickType tickTime_now = 0;	

	tickTime_now = xTaskGetTickCount();
	if(tickTime_now  - tickTime_prev > TIME_STAMP_200MS) {	// 超时未进入扭腰则重置数据
		dir = 1;
		target_z_speed = 0;
	}
	tickTime_prev = tickTime_now;
	
	if(target_z_speed >= maxTarget) {
		dir = 0;
	} else if(target_z_speed <= -maxTarget) {
		dir = 1;
	}
	
	if(dir == 1) {
		target_z_speed = RAMP_float(maxTarget, target_z_speed, rampTarget);
	} else if(dir == 0) {
		target_z_speed = RAMP_float((-maxTarget), target_z_speed, rampTarget);
	}
	return target_z_speed;
}

/**
 *	@brief	根据遥控值设置速度环的期望值
 *	@note		涉及麦克纳姆轮的运动合成
 *					target均为+时:
 *					LF201	↑	↓	RF202	
 *					
 *					LB203	↑	↓	RB204
 *					
 *					-660 ~ +660 遥控差值
 *					
 *					需要考虑摇杆的分配值，保证速度的目标值最大为CHASSIS_PID_OUT_MAX(9000)
 */

void REMOTE_setChassisSpeed(RC_Ctl_t *remoteInfo)
{
	float targetSpeedX, targetSpeedY, targetSpeedZ, targetTotalSpeed;	
	float kx, ky, kz;	
	uint16_t widthRcValue = abs(RC_CH_VALUE_MAX - RC_CH_VALUE_MIN)/2;
	float offset = abs(widthRcValue - CHASSIS_PID_OUT_MAX/RC_CHASSIS_SPEED_RATE);	// 遥控死区	
	
	targetSpeedX = (remoteInfo->rc.ch2 - RC_CH_VALUE_OFFSET);
	targetSpeedY = (remoteInfo->rc.ch3 - RC_CH_VALUE_OFFSET);
	if(Flag.gimbal.FLAG_pidMode == MECH) {
		targetSpeedZ = (remoteInfo->rc.ch0 - RC_CH_VALUE_OFFSET);
	} else {
		targetSpeedZ = -Chassis_Z_Speed_PID_calculate(&Chassis_Z_Speed_PID);	// (反向)云台为陀螺仪模式，底盘跟随
	}
	
	targetTotalSpeed = abs(targetSpeedX) + abs(targetSpeedY) + abs(targetSpeedZ); 
	if((targetTotalSpeed - offset) <= 0.f) {	// 遥控死区(18) => 遥控给定累加值>=18
		targetTotalSpeed = 0.f;
	} else { // targetTotalSpeed >= 18
		kx = targetSpeedX/targetTotalSpeed;
		ky = targetSpeedY/targetTotalSpeed;
		kz = targetSpeedZ/targetTotalSpeed;
	}
	
	if(targetTotalSpeed <= 0.f) {	// 死区
		Chassis_PID[LEFT_FRON_201].Speed.target = 0;
		Chassis_PID[RIGH_FRON_202].Speed.target = 0;
		Chassis_PID[LEFT_BACK_203].Speed.target = 0;
		Chassis_PID[RIGH_BACK_204].Speed.target = 0;
	} else if(targetTotalSpeed <= (widthRcValue-offset)) { // 正常区域
		Chassis_PID[LEFT_FRON_201].Speed.target = (+targetSpeedX + targetSpeedY + targetSpeedZ)*RC_CHASSIS_SPEED_RATE;
		Chassis_PID[RIGH_FRON_202].Speed.target = (+targetSpeedX - targetSpeedY + targetSpeedZ)*RC_CHASSIS_SPEED_RATE;
		Chassis_PID[LEFT_BACK_203].Speed.target = (-targetSpeedX + targetSpeedY + targetSpeedZ)*RC_CHASSIS_SPEED_RATE;
		Chassis_PID[RIGH_BACK_204].Speed.target = (-targetSpeedX - targetSpeedY + targetSpeedZ)*RC_CHASSIS_SPEED_RATE;			
	} else { // 遥控给定累加值 > 遥控单边最大值(660)
		Chassis_PID[LEFT_FRON_201].Speed.target = (+kx + ky + kz)*CHASSIS_PID_OUT_MAX;
		Chassis_PID[RIGH_FRON_202].Speed.target = (+kx - ky + kz)*CHASSIS_PID_OUT_MAX;
		Chassis_PID[LEFT_BACK_203].Speed.target = (-kx + ky + kz)*CHASSIS_PID_OUT_MAX;
		Chassis_PID[RIGH_BACK_204].Speed.target = (-kx - ky + kz)*CHASSIS_PID_OUT_MAX;
	}
}

/**
 *	@brief	按键斜坡函数
 *	@note	长按返回最大系数1
 *			长时未按返回最小系数0
 */
float KEY_rampChassisSpeed(int8_t key_state, int16_t *time, uint16_t inc_ramp_step, uint16_t dec_ramp_step)
{
	float fac;
	fac = 0.15 * sqrt( 0.15 * (*time) );	// time累加到296.3斜坡就完成
	if(key_state == 1) {	// 按键按下
		if(fac < 1) {
			*time += inc_ramp_step;
		}
	} else {	// 按键松开
		if(fac > 0) {
			*time -= dec_ramp_step;
			if(*time < 0) {
				*time = 0;
			}
		}
	}
	fac = constrain(fac, 0, 1);
	return fac;
}

/**
 *	@brief	根据按键值设置速度环的期望值
 *  @note	Loop Time: 2ms
 */
void KEY_setChassisSpeed(RC_Ctl_t *remoteInfo)
{
	static uint8_t keyFLockFlag = 0;
	static int16_t timeYFront, timeYBack, timeXLeft, timeXRight;
	float targetXSpeed, targetYSpeed, targetZSpeed, targetTotalSpeed;	
	float kx, ky, kz;
	
	
	if(IF_KEY_PRESSED_W) {
		timeYFront++;
		timeYBack = 0;	// 后退斜坡清零
	} 
	if(IF_KEY_PRESSED_S) {
		timeYBack++;
		timeYFront = 0;	// 前进斜坡清零
	}
	if(IF_KEY_PRESSED_A) {
		timeXLeft++;
		timeXRight = 0;	// 右移斜坡清零
	}
	if(IF_KEY_PRESSED_D) {
		timeXRight++;
		timeXLeft = 0;	// 左移斜坡清零
	}	
	if(IF_KEY_PRESSED_F) {
		if(keyFLockFlag == false) {
			if(Flag.chassis.FLAG_mode == CHAS_MODE_NORMAL) {
				Flag.chassis.FLAG_mode = CHAS_MODE_TWIST; 
			} else if(Flag.chassis.FLAG_mode == CHAS_MODE_TWIST) {
				Flag.chassis.FLAG_mode = CHAS_MODE_NORMAL;
				
			}
		}
		keyFLockFlag = true;
	} else {
		keyFLockFlag = false;
		
	}
	
	targetYSpeed = (KEY_rampChassisSpeed(IF_KEY_PRESSED_W, &timeYFront, 1, 500) - KEY_rampChassisSpeed(IF_KEY_PRESSED_S, &timeYBack, 1, 500))
					*RC_CH_VALUE_SIDE_WIDTH;	// 处理成模拟摇杆(0~660)
	targetXSpeed = (KEY_rampChassisSpeed(IF_KEY_PRESSED_D, &timeXRight, 1, 500) - KEY_rampChassisSpeed(IF_KEY_PRESSED_A, &timeXLeft, 1, 500))
					*RC_CH_VALUE_SIDE_WIDTH;	// 处理成模拟摇杆(0~660)
	// 默认为陀螺仪模式
	if(Flag.chassis.FLAG_mode == CHAS_MODE_TWIST) { 
		Chassis_Z_Speed_PID.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT + CHASSIS_twistTargetCalculate(800, 10);
	} else if(Flag.chassis.FLAG_mode == CHAS_MODE_NORMAL) {
		Chassis_Z_Speed_PID.target = GIMBAL_MECH_YAW_ANGLE_MID_LIMIT;	// 恢复地盘跟随
	}
	targetZSpeed = -Chassis_Z_Speed_PID_calculate(&Chassis_Z_Speed_PID);	// (反向)云台为陀螺仪模式，底盘跟随
	
	targetTotalSpeed = abs(targetXSpeed) + abs(targetYSpeed) + abs(targetZSpeed); 
	if((targetTotalSpeed - 18) <= 0.f) {	// 遥控死区(18) => 遥控给定累加值>=18
		targetTotalSpeed = 0.f;
	} else { // targetTotalSpeed >= 18
		kx = targetXSpeed/targetTotalSpeed;
		ky = targetYSpeed/targetTotalSpeed;
		kz = targetZSpeed/targetTotalSpeed;
	}
	
	if(targetTotalSpeed <= 0.f) {	// 死区
		Chassis_PID[LEFT_FRON_201].Speed.target = 0;
		Chassis_PID[RIGH_FRON_202].Speed.target = 0;
		Chassis_PID[LEFT_BACK_203].Speed.target = 0;
		Chassis_PID[RIGH_BACK_204].Speed.target = 0;
	} else if(targetTotalSpeed <= (RC_CH_VALUE_SIDE_WIDTH-18)) { // 正常区域
		Chassis_PID[LEFT_FRON_201].Speed.target = (+targetXSpeed + targetYSpeed + targetZSpeed)*RC_CHASSIS_SPEED_RATE;
		Chassis_PID[RIGH_FRON_202].Speed.target = (+targetXSpeed - targetYSpeed + targetZSpeed)*RC_CHASSIS_SPEED_RATE;
		Chassis_PID[LEFT_BACK_203].Speed.target = (-targetXSpeed + targetYSpeed + targetZSpeed)*RC_CHASSIS_SPEED_RATE;
		Chassis_PID[RIGH_BACK_204].Speed.target = (-targetXSpeed - targetYSpeed + targetZSpeed)*RC_CHASSIS_SPEED_RATE;			
	} else { // 遥控给定累加值 > 遥控单边最大值(660)
		Chassis_PID[LEFT_FRON_201].Speed.target = (+kx + ky + kz)*CHASSIS_PID_OUT_MAX;
		Chassis_PID[RIGH_FRON_202].Speed.target = (+kx - ky + kz)*CHASSIS_PID_OUT_MAX;
		Chassis_PID[LEFT_BACK_203].Speed.target = (-kx + ky + kz)*CHASSIS_PID_OUT_MAX;
		Chassis_PID[RIGH_BACK_204].Speed.target = (-kx - ky + kz)*CHASSIS_PID_OUT_MAX;
	}	
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控控制底盘任务
 */
void CHASSIS_rcControlTask(void)
{
	REMOTE_setChassisSpeed(&RC_Ctl_Info);
}

/**
 *	@brief	键盘控制底盘任务
 */

void CHASSIS_keyControlTask(void)
{
	if(!GIMBAL_ifBuffMode()) {
		KEY_setChassisSpeed(&RC_Ctl_Info);
	} else {
		Chassis_PID[LEFT_FRON_201].Speed.target = 0;
		Chassis_PID[RIGH_FRON_202].Speed.target = 0;
		Chassis_PID[LEFT_BACK_203].Speed.target = 0;
		Chassis_PID[RIGH_BACK_204].Speed.target = 0;
	}
}

/**
 *	@brief	底盘失控保护
 */
void CHASSIS_selfProtect(void)
{
	Chassis_stop(Chassis_PID);
	Chassis_PID_ParamsInit(Chassis_PID, CHASSIS_MOTOR_COUNT);
	Chassis_Z_Speed_PID_ParamsInit(&Chassis_Z_Speed_PID);	
}

/**
 *	@brief	底盘控制任务
 */
void CHASSIS_control(void)
{
	/*----信息读入----*/
	//..can.c中
	/*----期望修改----*/
	if(Flag.remote.FLAG_mode == RC) {
		CHASSIS_rcControlTask();
	} else if(Flag.remote.FLAG_mode == KEY) {
		CHASSIS_keyControlTask();
	}
	/*----最终输出----*/
	Chassis_Speed_PID_calculate(Chassis_PID, LEFT_FRON_201); 	// 左前 - 速度环
	Chassis_Speed_PID_calculate(Chassis_PID, RIGH_FRON_202); 	// 右前 - 速度环
	Chassis_Speed_PID_calculate(Chassis_PID, LEFT_BACK_203); 	// 左后 - 速度环
	Chassis_Speed_PID_calculate(Chassis_PID, RIGH_BACK_204); 	// 右后 - 速度环
	CHASSIS_powerLimit(&Chassis_Power, Chassis_PID, &Judge_Info);
	Chassis_PID_out(Chassis_PID);
}
