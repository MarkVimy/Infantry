#ifndef __TASK_REVOLVER_H
#define __TASK_REVOLVER_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
#define REVOLVER_ID					0x207
#define REVOLVER_BM_RX_REPORT		BM_RX_REPORT_207

#define REVOLVER_SPEED_RATIO		2160		// 1秒转一圈(= 60rpm*36(减速比) => 1rps)
#define REVOLVER_SPEED_GRID			12.0f		// 拨盘12格
#define SPEED_AN_BULLET_PER_SECOND	(REVOLVER_SPEED_RATIO/REVOLVER_SPEED_GRID)	// 1发/s 的期望速度
#define ANGLE_AN_BULLET				24576.0f	// 8192*36/12 = 24576.0f
#define ANGLE_AN_BULLET_RAMP		(ANGLE_AN_BULLET/20)

#define HEAT_AN_BULLET				30	// 打一颗弹丸的热量增加

//#define LEVEL_COUNT				3
//uint16_t REVOLVER_HEAT_LIMIT[LEVEL_COUNT] = {240, 360, 480};

/* Global TypeDef ------------------------------------------------------------*/
typedef enum {
	REVO_STATE_OFF = 0,
	REVO_STATE_ON = 1,
	REVO_STATE_COUNT =2
}Revolver_State_Names_t;

typedef enum {
	REVO_POSI_MODE  = 0,	// 点射模式(位置环)
	REVO_SPEED_MODE = 1,	// 连发模式(速度环)
	REVO_MODE_COUNT = 2
}Revolver_Mode_Names_t;

typedef enum {
	SHOOT_NORMAL = 0,		// 常规模式
	SHOOT_SINGLE = 1,		// 单发模式
	SHOOT_TRIPLE = 2,		// 三连发
	SHOOT_HIGH_F_LOW_S = 3,	// 高射频低射速(推家)
	SHOOT_MID_F_HIGH_S = 4,	// 中射频高射速(暴击)
	SHOOT_BUFF	 = 5,		// 打符模式
	SHOOT_AUTO	 = 6,		// 自瞄模式
	REVO_ACTION_COUNT = 7,
}Revolver_Action_t;

typedef struct {
	PID_Object_t	Speed;
	PID_Object_t	Angle;
	float			AngleRampBuffer;
	float			Out;
}Revolver_PID_t;

typedef struct {
	uint32_t lost_time;
	uint32_t lost_time_boundary;
	uint32_t lock_time;
	uint32_t lock_time_boundary;
	uint16_t change_armor_delay;
	uint16_t change_armor_delay_boundary;
	uint32_t respond_interval;
	bool	 change_armor;
	bool	 fire;
}Revolver_Buff_Info_t;

typedef struct {
	Revolver_State_Names_t 	state;
	Revolver_Mode_Names_t 	pidMode; 
	Revolver_Action_t		action;
	uint16_t	freq;	// 射频
	int16_t		shootInterval;	// 发射时间间隔
	int16_t		shootNum;		// 需要发射的子弹数
	int16_t		shootNumBuffer; // 允许发射的子弹数
	int16_t		shootHeatCoolingRate; // 17mm枪口每秒冷却值
	int16_t		shootHeatReal;	// 17mm枪口热量
	int16_t		shootHeatLimit;	// 17mm枪口热量限制
	uint16_t 	stuckCount;	// 卡弹计数
	Revolver_Buff_Info_t	buff;
}Revolver_Info_t;

/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Revolver_PID_t Revolver_PID;
extern Revolver_Info_t Revolver;

/* API functions Prototypes --------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_pidParamsInit(Revolver_PID_t *pid);
void REVOLVER_stop(Revolver_PID_t *pid);
void REVOLVER_Speed_pidCalculate(Revolver_PID_t *pid);
void REVOLVER_Angle_pidCalculate(Revolver_PID_t *pid);
void REVOLVER_pidOut(Revolver_PID_t *pid);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_setAction(Revolver_Action_t action);
uint8_t REVOLVER_heatLimit(Revolver_Info_t *info, uint8_t targetNum);
void REVOLVER_normalControl(Revolver_Info_t *info);
void REVOLVER_singleShootControl(Revolver_Info_t *info);
void REVOLVER_tripleShootControl(Revolver_Info_t *info);
void REVOLVER_buffShootControl(Revolver_Info_t *info);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REVOLVER_rcControlTask(void);
void REVOLVER_keyControlTask(void);
void REVOLVER_selfProtect(void);
void REVOLVER_control(void);

#endif
