#ifndef __TASK_GIMBAL_H
#define __TASK_GIMBAL_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "my_app.h"

/* Global macro --------------------------------------------------------------*/
#define GIMBAL_MODE_COUNT		PID_MODE_COUNT

#define GIMBAL_MECH_YAW_ANGLE_LEFT_LIMIT		(1998 + 200)	// 实测最小可达1896(不发力) 1892(发力)
#define GIMBAL_MECH_YAW_ANGLE_MID_LIMIT			(4104)	// 中间值
#define GIMBAL_MECH_YAW_ANGLE_RIGHT_LIMIT		(6235 - 200)	// 实测最大可达6180(不发力)	6184(发力)

#define GIMBAL_GYRO_YAW_ANGLE_LEFT_LIMIT		(-7200.0f)	// 对应-360
#define GIMBAL_GYRO_YAW_ANGLE_RIGHT_LIMIT		(7200.0f)	// 对应+360
#define GIMBAL_GYRO_YAW_ANGLE_LIMIT				(7200.0f)

#define GIMBAL_GYRO_YAW_ANGLE_MID_LIMIT			(8147.0f)	// 陀螺模式下的中间值

#define GIMBAL_MECH_PITCH_ANGLE_UP_LIMIT		(3405 + 100)	// 实测最小可达3312(不发力)
#define GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT		(4096)			// 中间值
#define GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT		(4472 - 100)	// 实测最小可达4382(不发力)

#define GIMBAL_AUTO_LOCK_SENTRY_ANGLE			(GIMBAL_MECH_PITCH_ANGLE_UP_LIMIT + 150)

#define GIMBAL_GYRO_PITCH_ANGLE_UP_LIMIT		GIMBAL_MECH_PITCH_ANGLE_UP_LIMIT 	//((-28+2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_ANGLE_MID_LIMIT		GIMBAL_MECH_PITCH_ANGLE_MID_LIMIT 	//( +1 * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)
#define GIMBAL_GYRO_PITCH_ANGLE_DOWN_LIMIT		GIMBAL_MECH_PITCH_ANGLE_DOWN_LIMIT 	//((+13-2) * GIMBAL_GYRO_ANGLE_ZOOM_INDEX)

#define GIMBAL_GYRO_ANGLE_ZOOM_INDEX			(20.0f)	// IMU角度的缩放系数

#define GIMBAL_RAMP_BEGIN_YAW					4
#define GIMBAL_RAMP_BEGIN_PITCH					4

/* Global TypeDef ------------------------------------------------------------*/
/**
 *	@brief	云台Pid模式选择
 */
typedef enum {
	YAW_205 = 0, 	// Yaw
	PITCH_206 = 1,	// Pitch
	GIMBAL_MOTOR_COUNT = 2
}Gimbal_Motor_Names_t;

/**
 *	@brief	云台模式
 */
typedef enum {
	GIMBAL_MODE_NORMAL 		= 0, // 正常模式
	GIMBAL_MODE_TOP_GYRO	= 1, // 小陀螺模式
	GIMBAL_MODE_AUTO   		= 2, // 自瞄模式
	GIMBAL_MODE_BIG_BUFF	= 3, // 大符模式
	GIMBAL_MODE_SMALL_BUFF	= 4, // 小符模式
}Gimbal_Mode_t;	

/**
 *	@brief	云台PID
 */
typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			AngleRampTarget;
	float			AngleRampFeedback;
	float			Out;
}Gimbal_PID_t;

/**
 *	@brief	视觉反馈信息
 */
typedef struct {
	Gimbal_Mode_t mode;
}Gimbal_State_t;

/**
 *	@brief	速度解算信息
 */
typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}Speed_Calculate_t;

/**
 *	@brief	云台自瞄PID信息
 */
typedef struct {
	float kp;
	float erro;
	float target;
}Gimbal_Vision_Info_t;

/**
 *	@brief	云台自瞄PID信息
 */
typedef struct {
	PID_Object_t	Yaw;
	PID_Object_t	Pitch;
	uint32_t		Time[TIME_STATE_COUNT];
	int8_t			FLAG_first_into_auto;
}Gimbal_Vision_Auto_t;

/**
 *	@brief	云台打符PID信息
 */
typedef struct {
	PID_Object_t	Yaw;
	PID_Object_t	Pitch;
	uint32_t				Time[TIME_STATE_COUNT];
	int8_t					FLAG_first_into_buff;
}Gimbal_Vision_Buff_t;

/**
 *	@brief	云台综合信息结构体
 */
typedef struct {
	Gimbal_State_t 			State;
	Gimbal_Vision_Auto_t 	Auto;
	Gimbal_Vision_Buff_t	Buff;
}Gimbal_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Gimbal_PID_t Gimbal_PID[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT];
extern Gimbal_Info_t Gimbal;

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_pidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_pidParamsInit(Gimbal_PID_t *pid, uint8_t motor_cnt);
void GIMBAL_kalmanCreate(void);
void GIMBAL_stop(Gimbal_PID_t *pid);
void GIMBAL_IMU_recordFeedback(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT]);
void GIMBAL_GYRO_calAverageOffset(Mpu_Info_t mpuInfo);
void GIMBAL_Speed_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_Angle_pidCalculate(Gimbal_PID_t *pid, Gimbal_Motor_Names_t MOTORx);
void GIMBAL_pidOut(Gimbal_PID_t *pid);

/* #信息层# ---------------------------------------------------------------------------------------------------------------------------------------*/
Gimbal_Mode_t GIMBAL_getGimbalMode(void);
bool GIMBAL_ifNormalMode(void);
bool GIMBAL_ifAutoMode(void);
bool GIMBAL_ifBuffMode(void);
bool GIMBAL_ifAimSentry(void);
bool GIMBAL_BUFF_chaseReady(void);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_init(void);
void GIMBAL_reset(void);
float GIMBAL_GYRO_yawTargetBoundaryProcess(Gimbal_PID_t *pid, float delta_target);
//void REMOTE_setGimbalAngle(RC_Ctl_t *remoteInfo);
//void KEY_setGimbalAngle(RC_Ctl_t *remoteInfo);
//void KEY_setQuickPickUp(RC_Ctl_t *remoteInfo);
void GIMBAL_rcMech_To_rcGyro(void);
void GIMBAL_rcGyro_To_rcMech(void);
void GIMBAL_rcMech_To_keyGyro(void);
void GIMBAL_keyGyro_To_rcMech(void);
void GIMBAL_keyGyro_To_keyMech(void);
void GIMBAL_keyMech_To_keyGyro(void);
void GIMBAL_pidParamsSwitch(Gimbal_PID_t pid[GIMBAL_MODE_COUNT][GIMBAL_MOTOR_COUNT], Gimbal_Info_t *gimbal);
void GIMBAL_normalControl(void);
void GIMBAL_autoControl(void);
void GIMBAL_buffControl(void);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void GIMBAL_pidControlTask(void);
void GIMBAL_visionControlTask(void);
void GIMBAL_rcControlTask(void);
void GIMBAL_keyControlTask(void);
void GIMBAL_selfProtect(void);
void GIMBAL_control(void);

#endif
