#include "pwm.h"

/**
 * @file        magzine.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        18-October-2019
 * @brief       This file includes the SERVO(舵机) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 舵机
 */

/* Includes ------------------------------------------------------------------*/
#include "magzine.h"

#include "pwm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
Magzine_Info_t Magzine = {
	.state = MAGZ_STATE_CLOSE,
	.angleTarget = MAGZ_ANGLE_CLOSE,
	.angleFeedback = MAGZ_ANGLE_CLOSE
};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	弹仓舵机参数初始化
 */
void MAGZINE_ParamsInit(Magzine_Info_t *magz)
{
	magz->state = MAGZ_STATE_CLOSE;
	magz->angleTarget = MAGZ_ANGLE_CLOSE;
	//magz->angleFeedback = MAGZ_ANGLE_CLOSE;
}

/**
 *	@brief	弹仓舵机停止转动(此时可手动打开弹仓)
 */
void MAGZINE_stop(Magzine_Info_t *magz)
{
	MAGZINE_ParamsInit(magz);
	MAGZINE_pwmOut(0);
}

/**
 *	@brief	弹仓斜坡输出
 */
void MAGZINE_output(Magzine_Info_t *magz)
{
	if(magz->angleFeedback < magz->angleTarget) {	// 加速
		magz->angleFeedback += 5;
		if(magz->angleFeedback > magz->angleTarget) {
			magz->angleFeedback = magz->angleTarget;
		}
	} else if(magz->angleFeedback > magz->angleTarget) {	// 减速
		magz ->angleFeedback -= 5;
		if(magz->angleFeedback < magz->angleTarget) {
			magz->angleFeedback = magz->angleTarget;
		}
	}
	
	if(magz->angleFeedback < 0) {
		magz->angleFeedback = 0;
	}

	MAGZINE_pwmOut(Magzine.angleFeedback);
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置弹仓开启/关闭
 *	@note	SW2_MID + SW1_UP
 */
void REMOTE_setMagzineState(RC_Ctl_t *remoteInfo)
{
	static uint8_t prev_sw1 = RC_SW_UP;
	uint8_t	sw1, sw2;
	sw1 = remoteInfo->rc.s1;
	sw2 = remoteInfo->rc.s2;
	
	if((sw1 == RC_SW_UP && prev_sw1 != RC_SW_UP) && (sw2 == RC_SW_MID)) {
		if(Magzine.state == MAGZ_STATE_CLOSE) {
			Magzine.state = MAGZ_STATE_OPEN;
			Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// 打开弹仓
		} else if(Magzine.state == MAGZ_STATE_OPEN){
			Magzine.state = MAGZ_STATE_CLOSE;
			Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// 关闭弹仓
		}
	}
	prev_sw1 = sw1;
}

/**
 *	@brief	遥控设置弹仓开启/关闭
 *	@note	按键 R
 */
void KEY_setMagzineState(RC_Ctl_t *remoteInfo)
{
//	static uint16_t timeToClose = 0;
//	
//		
//	if(IF_KEY_PRESSED_R) {	// 按下R
//		Magzine.state = MAGZ_STATE_OPEN;
//		Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// 打开弹仓
//		timeToClose = 500;	// 500*20ms = 10s自动关闭
//	} else {	// 松开R
//		if(timeToClose) {	// 开始倒计时
//			timeToClose--;
//		} else {
//			Magzine.state = MAGZ_STATE_CLOSE;
//			Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// 关闭弹仓
//		}
//	}
	
	static uint8_t keyRLockFlag = false;
	if(IF_KEY_PRESSED_R) {	// 按下R
		if(keyRLockFlag == false) {
			if((Magzine.state == MAGZ_STATE_CLOSE) && (Magzine.angleFeedback == Magzine.angleTarget)) {
				Magzine.state = MAGZ_STATE_OPEN;
				Magzine.angleTarget = MAGZ_ANGLE_OPEN;	// 打开弹仓
			} else if((Magzine.state == MAGZ_STATE_OPEN) && (Magzine.angleFeedback == Magzine.angleTarget)) {
				Magzine.state = MAGZ_STATE_CLOSE;
				Magzine.angleTarget = MAGZ_ANGLE_CLOSE;	// 关闭弹仓
			}
		}
		keyRLockFlag = true;
	} else {	// 松开R
		keyRLockFlag = false;
	}
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	遥控设置弹仓开启/关闭
 *	@note	Loop Time: 20ms
 */
void MAGZINE_rcControlTask(void)
{
	REMOTE_setMagzineState(&RC_Ctl_Info);
}

/**
 *	@brief	按键设置弹仓开启/关闭
 *	@note	Loop Time: 20ms
 */
void MAGZINE_keyControlTask(void)
{
	KEY_setMagzineState(&RC_Ctl_Info);
}

/**
 *	@brief	弹仓失控保护
 */
void MAGZINE_selfProtect(void)
{
	MAGZINE_stop(&Magzine);
}

/**
 *	@brief	弹仓控制
 */
void MAGZINE_control(void)
{
	/*----信息读入----*/
	
	/*----期望修改----*/
	if(Flag.remote.FLAG_mode == RC) {
		MAGZINE_rcControlTask();
	} else if(Flag.remote.FLAG_mode == KEY) {
		MAGZINE_keyControlTask();
	}	
	/*----最终输出----*/
	MAGZINE_output(&Magzine);
}
