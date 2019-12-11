#include "pwm.h"

/**
 * @file        pwm.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        17-October-2019
 * @brief       This file includes the FRICTION WHEEL(摩擦轮) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 摩擦轮电机 朗宇电机
 *	  
 *	# 摩擦轮电调 小蜜蜂电调
 *
 *	# 舵机
 */

/* Includes ------------------------------------------------------------------*/
#include "friction.h"

#include "pwm.h"

#include "remote.h"
#include "laser.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
Friction_Info_t Friction = {
	.state = FRIC_STATE_OFF,
	.speedLevel = FRIC_SPEED_OFF,
	.speedTarget = 0,
	.speedFeedback = 0,
};

uint16_t Friction_Pwm_Output[FRIC_SPEED_COUNT] = {0, 460, 505, 563, 695, 685};	// 0, 460, 505, 583, 695, 685
												// 关闭  低速  中速  高速  狂暴  哨兵
uint16_t Friction_Pwm_Speed[FRIC_SPEED_COUNT] = {0, 0, 2190, 2510, 2850, 2800};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	摩擦轮参数初始化
 */
void FRICTION_ParamsInit(Friction_Info_t *fric)
{
	fric->state = FRIC_STATE_OFF;
	fric->speedLevel = FRIC_SPEED_OFF;
	fric->speedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	//fric->speedFeedback = 0;
}

/**
 *	@brief	摩擦轮斜坡减小输出至停止
 */
void FRICTION_stop(Friction_Info_t *fric)
{
//	fric->speedLevel = FRIC_SPEED_OFF;
//	fric->speedTarget = Friction_Pwm_Output[FRIC_SPEED_OFF];
	FRICTION_ParamsInit(fric);
	if(Friction.speedFeedback > 0) {
		Friction.speedFeedback -= 5;
	} else if(Friction.speedFeedback < 0) {
		Friction.speedFeedback = 0;
	}
	
	FRICTION_pwmOut(Friction.speedFeedback, Friction.speedFeedback);
}

/**
 *	@brief	摩擦轮斜坡输出
 */
void FRICTION_pwmCalculate(Friction_Info_t *fric)
{
	if(fric->speedFeedback < fric->speedTarget) {	// 加速
		fric->speedFeedback += 5;
		if(fric->speedFeedback > fric->speedTarget) {
			fric->speedFeedback = fric->speedTarget;
		}
	} else if(fric->speedFeedback > fric->speedTarget) {	// 减速
		fric ->speedFeedback -= 5;
		if(fric->speedFeedback < fric->speedTarget) {
			fric->speedFeedback = fric->speedTarget;
		}
	}
	
	if(fric->speedFeedback < 0) {
		fric->speedFeedback = 0;
	}
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	摩擦轮复位
 */
void FRICTION_reset(void)
{
	if(Friction.speedFeedback != 0) {
		FRICTION_stop(&Friction);
	} else {
		BM_reset(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_FRIC);	// 复位完成
	}	
}

/**
 *	@brief	判断摩擦轮是否加速/减速到期望速度
 */
bool FRICTION_ifFrictionReady(void)
{
	if(Friction.state == FRIC_STATE_OFF)
		return 0;
	return (abs(Friction.speedTarget - Friction.speedFeedback)<=20)?1:0;
}

void FRICTION_speedSwitch(Friction_Info_t *fric)
{
	if(FRICTION_ifFrictionReady()) {
		/* 判断在击打哨兵(抬头pitch减小) */
		if(GIMBAL_ifAimSentry()) {
			Friction.speedLevel = FRIC_SPEED_SENTRY;
		}
		/* 打符的时候采用超高射速 */
		else if(GIMBAL_ifBuffMode()) {
			Friction.speedLevel = FRIC_SPEED_VERYHIGH;
		}
		/* 正常模式下采用高射速 */
		else if(GIMBAL_ifNormalMode()) {
			Friction.speedLevel = FRIC_SPEED_HIGH;
		}
		Friction.speedTarget =  Friction_Pwm_Output[Friction.speedLevel];
	}
}

/**
 *	@brief	遥控设置摩擦轮开启/关闭
 *	@note	SW2_DOWN + SW1_UP
 */
void REMOTE_setFrictionState(RC_Ctl_t *remoteInfo)
{
	static uint8_t prev_sw1 = RC_SW_UP;
	uint8_t	sw1, sw2;
	sw1 = remoteInfo->rc.s1;
	sw2 = remoteInfo->rc.s2;
	
	if(BM_ifReset(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_FRIC)) {	// 摩擦轮复位完成
		if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_DOWN)) {
			if(Friction.state == FRIC_STATE_OFF) {
				Friction.state = FRIC_STATE_ON;			// 打开摩擦轮
				Friction.speedLevel = FRIC_SPEED_HIGH;	// 中等射速
				Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
				LASER_ON();
			} else if(Friction.state == FRIC_STATE_ON){
				Friction.state = FRIC_STATE_OFF;		// 关闭摩擦轮
				Friction.speedLevel = FRIC_SPEED_OFF;	// 零射速
				Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
				LASER_OFF();
			}
		}
	}
	prev_sw1 = sw1;
}

/**
 *	@brief	遥控设置摩擦轮开启/关闭
 *	@note	键盘模式下摩擦轮开启后不关闭(由鼠标左键点击触发)
 */
void KEY_setFrictionState(RC_Ctl_t *remoteInfo)
{
	if(BM_ifReset(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_FRIC)) {	// 摩擦轮复位完成
		if(Friction.state == FRIC_STATE_OFF && IF_MOUSE_PRESSED_LEFT) {
			Friction.state = FRIC_STATE_ON;			// 打开摩擦轮
			Friction.speedLevel = FRIC_SPEED_HIGH;	// 中等射速
			Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
			LASER_ON();
		}
//		/* #测试需要(先用鼠标右键触发关闭) */
//		if(IF_MOUSE_PRESSED_RIGH) {
//			Friction.state = FRIC_STATE_OFF;		// 关闭摩擦轮
//			Friction.speedLevel = FRIC_SPEED_OFF;	// 零射速
//			Friction.speedTarget = Friction_Pwm_Output[Friction.speedLevel];
//			LASER_OFF();
//		}
	}
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置摩擦轮开启/关闭
 *	@note	Loop Time: 10ms
 */
void FRICTION_rcControlTask(void)
{
	REMOTE_setFrictionState(&RC_Ctl_Info);
}

/**
 *	@brief	按键设置摩擦轮开启/关闭
 *	@note	Loop Time: 10ms
 */
void FRICTION_keyControlTask(void)
{
	KEY_setFrictionState(&RC_Ctl_Info);
}

/**
 *	@brief	摩擦轮失控保护
 */
void FRICTION_selfProtect(void)
{
	FRICTION_stop(&Friction);
	LASER_OFF();
}

/**
 *	@brief	摩擦轮控制
 */
void FRICTION_control(void)
{
	/*----信息读入----*/
	
	/*----期望修改----*/
	if(BM_ifSet(BitMask.system.BM_systemReset, BM_SYSTEM_RESET_FRIC)) {	// 复位状态
		FRICTION_reset(); // 摩擦轮复位
	} else {
		if(Flag.remote.FLAG_mode == RC) {
			FRICTION_rcControlTask();
		} else if(Flag.remote.FLAG_mode == KEY) {
			FRICTION_keyControlTask();
		}	
	}
	
	/* 根据云台模式切换摩擦轮参数 */
	FRICTION_speedSwitch(&Friction);	
	
	/*----最终输出----*/
	FRICTION_pwmCalculate(&Friction);	// 斜坡计算
	FRICTION_pwmOut(Friction.speedFeedback, Friction.speedFeedback);
}
