#ifndef __REMOTE_H
#define __REMOTE_H

#include "sys.h"
#include "friction.h"
#include "magzine.h"
#include "judge.h"
#include "my_task.h"

/* ----------------------- RC Setting Definition---------------------------- */

#define		RC_DBUS_FRAME_LEN		18u			// 18字节一帧
#define 	RC_BAUDRATE				100000		// 100Kbps

/* ----------------------- RC & KEY Operation Definition-------------------------- */
#define    RC_GIMBAL_MECH_YAW_SENSITIVY			0.0092f
#define    RC_GIMBAL_MECH_PITCH_SENSITIVY		0.0175f
#define	   RC_GIMBAL_GYRO_YAW_SENSITIVY			0.0285f
#define    RC_GIMBAL_GYRO_PITCH_SENSITIVY		RC_GIMBAL_MECH_PITCH_SENSITIVY	//0.0050f
#define    RC_CHASSIS_SPEED_RATE				14
#define	   RC_CHASSIS_Z_SPEED_SENSITIVY			1.0f

#define	   KEY_GIMBAL_GYRO_YAW_SENSITIVY		0.05f
#define    KEY_GIMBAL_GYRO_PITCH_SENSITIVY		0.20f


/* ----------------------- RC Channel Definition---------------------------- */

#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define	   RC_CH_VALUE_MAX       ((uint16_t)1684)
#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2)

/* ----------------------- RC Switch Definition----------------------------- */

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* ----------------------- Function Definition-------------------------------- */
#define		RC_LEFT_CH_LR_VALUE			(RC_Ctl_Info.rc.ch2 - RC_CH_VALUE_OFFSET)
#define		RC_LEFT_CH_UD_VALUE			(RC_Ctl_Info.rc.ch3 - RC_CH_VALUE_OFFSET)
#define		RC_RIGH_CH_LR_VALUE			(RC_Ctl_Info.rc.ch0 - RC_CH_VALUE_OFFSET)
#define		RC_RIGH_CH_UD_VALUE			(RC_Ctl_Info.rc.ch1 - RC_CH_VALUE_OFFSET)

/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (RC_Ctl_Info.rc.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (RC_Ctl_Info.rc.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (RC_Ctl_Info.rc.s1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (RC_Ctl_Info.rc.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (RC_Ctl_Info.rc.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (RC_Ctl_Info.rc.s2 == RC_SW_DOWN)


/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (RC_Ctl_Info.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (RC_Ctl_Info.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (RC_Ctl_Info.mouse.z)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (RC_Ctl_Info.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (RC_Ctl_Info.mouse.press_r == 1)


/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  RC_Ctl_Info.key.v  )
#define    IF_KEY_PRESSED_W       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (RC_Ctl_Info.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )


typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t  s1;
		uint8_t  s2;
		uint16_t sw;
		uint8_t	 isDataValid;
	}rc;
	
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	
	struct
	{
		uint16_t v;
	}key;
	
} RC_Ctl_t;

extern RC_Ctl_t RC_Ctl_Info;

/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REMOTE_reenableRC(RC_Ctl_t *remoteInfo);
void REMOTE_resetRCData(RC_Ctl_t *remoteInfo);
bool REMOTE_isRCChannelReset(RC_Ctl_t *remoteInfo);
void REMOTE_init(void);

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REMOTE_processData(RC_Ctl_t *remoteInfo, char *rxBuf);
void REMOTE_rcLostProcess(void);
void REMOTE_rcLostReport(void);
void REMOTE_rcErrReport(void);
void REMOTE_rcCorrectProcess(void);

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
void REMOTE_control(void);

#endif
