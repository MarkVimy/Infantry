/**
 * @file        vision.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        27-October-2019
 * @brief       This file includes the VISION Communication Protocols(视觉通信协议) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 与视觉的通信使用USART4作为通讯接口
 *	通信协议的格式与裁判系统协议的格式类似
 *	通信协议格式:
 *				帧头		数据段			帧尾
 *	域		frame_header 	data  			frame_tailer(CRC16,整包校验)
 *	字节		3			17				2
 *	
 *	帧头格式:
 *		  起始字节(0xA5)	数据命令长度	帧头CRC8校验
 *	域			SOF 	 	cmd_id    		CRC8
 *	字节		0			1				1
 */

/* Includes ------------------------------------------------------------------*/
#include "vision.h"

#include "uart4.h"
#include "crc.h"
#include "can.h"
#include "kalman.h"

/* Private typedef -----------------------------------------------------------*/
typedef enum {
	NONE = 0,
	RED = 1,
	BLUE = 2,
} Vision_Color_t;

/**
 *	电控->视觉
 */
typedef enum {
	CMD_AIM_OFF 		= 	0x00,	// 不识别
	CMD_AIM_RED 		= 	0x01,	// 识别红色
	CMD_AIM_BLUE 		= 	0x02,	// 识别蓝色
	CMD_AIM_RED_BUFF	=	0x03,	// 识别红符
	CMD_AIM_BLUE_BUFF	=	0x04,	// 识别蓝符
} Vision_Cmd_Id_t;

/* 帧头字节偏移 */
typedef enum {
	SOF			= 0,
	CMD_ID		= 1,
	CRC8		= 2,
	DATA		= 3,
	CRC16		= 20,
} Vision_Frame_Header_Offset_t;

/* 数据长度 */
typedef enum {
	/* Std */
	LEN_FRAME_HEADER 		= 3,	// 帧头长度
	LEN_RX_DATA 			= 25,	// 接收数据段长度
	LEN_TX_DATA 			= 17,	// 发送数据段长度
	LEN_FRAME_TAILER 		= 2,	// 帧尾CRC16
	LEN_VISION_RX_PACKET	= 30,	// 接收包整包长度
	LEN_VISION_TX_PACKET	= 22,	// 发送包整包长度
} Vision_Data_Length_t;

/* 帧头格式 */
typedef __packed struct
{
	uint8_t  			sof;		// 同步头
	Vision_Cmd_Id_t  	cmd_id;		// 命令码
	uint8_t  			crc8;		// CRC8校验码
} Vision_Frame_Header_t;

/* 帧尾格式 */
typedef __packed struct 
{
	uint16_t crc16;					// CRC16校验码
} Vision_Frame_Tailer_t;

/* 接收数据段格式 */
typedef __packed struct 
{
	float 	pitch_angle;	// pitch偏差角度/像素点	单位：角度/像素点
	float 	yaw_angle;		// yaw偏差角度/像素点	单位：角度/像素点
	float 	distance;		// 距离				单位：mm
	uint8_t aim_at_center;	// 是否瞄准			单位：0/1
	uint8_t identify_target;// 是否识别到目标	单位：0/1
	uint8_t identify_buff;	// 是否识别到Buff	单位：0/1
	uint8_t identify_too_close;	// 目标距离过近	单位：0/1
	uint8_t open_anti_gyro;	// 是否识别到
	float	predict_yaw_angle;	// 预测角度值
	float	predict_yaw_speed;	// 预测角速度
} Vision_Rx_Data_t;

/* 发送数据段格式 */
typedef __packed struct
{
	uint16_t fric_speed;	// 射速乘以100发送给视觉那边
	int16_t rate_yaw;		// 以整形发送给视觉
	int16_t yaw_angle;		// Yaw电机机械角度
	int16_t pitch_angle;	// Pitch电机机械角度
} Vision_Tx_Data_t;

/* 接收包格式 */
typedef __packed struct 
{
	Vision_Frame_Header_t FrameHeader;	// 帧头
	Vision_Rx_Data_t	  RxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾	
} Vision_Rx_Packet_t;

/* 发送包格式 */
typedef __packed struct
{
	Vision_Frame_Header_t FrameHeader;	// 帧头
	Vision_Tx_Data_t	  TxData;		// 数据
	Vision_Frame_Tailer_t FrameTailer;	// 帧尾		
} Vision_Tx_Packet_t;

/* 辅助标识变量 */
typedef struct
{
	Vision_Color_t 	my_color;		// 机器人自己的颜色
	Vision_Color_t  attack_color;	// 识别和击打的敌军颜色
	Vision_Color_t  buff_color;		// 识别和击打的大小符颜色
	Vision_Mode_t	mode;			// 视觉模式
	uint16_t		unconnected_cnt;// 视觉失联计数
	uint8_t  		rx_data_valid;	// 接收数据的正确性
	uint16_t 		rx_err_cnt;		// 接收数据的错误统计
	uint32_t		rx_cnt;			// 接收数据包的统计
	uint8_t 		rx_data_update;	// 接收数据是否更新
	uint32_t 		rx_time_prev;	// 接收数据的前一时刻
	uint32_t 		rx_time_now;	// 接收数据的当前时刻
	uint16_t 		rx_time_ping;	// 帧率
} Vision_State_t;

/* 视觉通信数据包格式 */
typedef struct {
	Vision_Rx_Packet_t RxPacket;
	Vision_Tx_Packet_t TxPacket;
	Vision_State_t     State;
} Vision_Info_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define VISION_TX_BUFFER_LEN	50
#define VISION_FRAME_HEADER		(0xA5)

/*-------视觉分辨率预编译--------*/
#define	VISION_1280P	0	// 1280(H)*720(V)
#define	VISION_640P		1	// 640(H)*480(V)

#define VISION_DPI		VISION_1280P
#if VISION_DPI == VISION_1280P
	#define VISION_MID_YAW		480
	#define VISION_MID_PITCH	512
#elif VISION_DPI == VISION_640P
	#define	VISION_MID_YAW		320
	#define	VISION_MID_PITCH	240
#endif


/* Private variables ---------------------------------------------------------*/
/* ## Global variables ## ----------------------------------------------------*/
uint8_t Vision_Tx_Buffer[VISION_TX_BUFFER_LEN];
Vision_Info_t Vision = {
	.TxPacket.TxData.fric_speed = 0,
	.TxPacket.TxData.rate_yaw = 0,
	.TxPacket.TxData.pitch_angle = 1.f,
	.TxPacket.TxData.yaw_angle = 2.f,
	.State.my_color = BLUE,
};

extKalman_t Gimbal_Gyro_Auto_kalmanError[2];

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/* #驱动层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	读取视觉通信数据
 *	@return false - 数据错误
 *			true  - 数据正确	
 *	@note	uart4.c中IRQ调用
 */
extern float yaw;
float predict_yaw = 0;
float last_predict_yaw = 0;
float predict_yaw_speed = 0;
float bias=0.f;

float js_predict_yaw_speed = 0;
float last_js_predict_yaw_speed = 0;
float js_kLPF = 0.1;
bool VISION_readData(uint8_t *rxBuf)
{
	uint8_t res = false;
	Vision.State.rx_cnt++;
	Vision.State.unconnected_cnt = 0;
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == VISION_FRAME_HEADER) 
	{	
		res = Verify_CRC8_Check_Sum( rxBuf, LEN_FRAME_HEADER );
		/* 帧头CRC8校验*/
		if(res == true)
		{
			res = Verify_CRC16_Check_Sum( rxBuf, LEN_VISION_RX_PACKET );
			/* 帧尾CRC16校验 */
			if(res == true) 
			{
				/* 数据正确则拷贝接收包 */
				memcpy(&Vision.RxPacket, rxBuf, LEN_VISION_RX_PACKET);
				Vision.State.rx_data_update = true;	// 视觉数据更新
				
				/* 帧率计算 */
				Vision.State.rx_time_now = xTaskGetTickCount();
				Vision.State.rx_time_ping = Vision.State.rx_time_now - Vision.State.rx_time_prev;
				Vision.State.rx_time_prev = Vision.State.rx_time_now;
				
				js_predict_yaw_speed = js_kLPF*Vision.RxPacket.RxData.predict_yaw_speed + (1-js_kLPF)*last_js_predict_yaw_speed;
				last_js_predict_yaw_speed = js_predict_yaw_speed;
				
				//temp = Vision.State.rx_time_ping;
				predict_yaw = yaw + Vision.RxPacket.RxData.yaw_angle;
				bias = predict_yaw - last_predict_yaw;
				if(abs(bias)>179.f)
				{
					if(bias>0)
					{
						bias = bias - 360;
					
					}
					else if(bias<0)
					{
						bias = bias + 360;
					
					}
							
				}
				predict_yaw_speed = bias*1000;
				last_predict_yaw = predict_yaw;
				
			}
		}
	}
	
	/* 数据有效性判断 */
	if(res == true) {
		Vision.State.rx_data_valid = true;
	} else if(res == false) {
		Vision.State.rx_data_valid = false;
		Vision.State.rx_err_cnt++;
	}
	
	return res;
}

/**
 *	@brief	发送视觉通信数据
 *	@note	uart4发送
 */
void VISION_sendData(Vision_Tx_Packet_t *txPacket, Vision_Cmd_Id_t cmd_id)
{
	uint8_t i;
	/* 帧头格式 */
	txPacket->FrameHeader.sof = VISION_FRAME_HEADER;
	txPacket->FrameHeader.cmd_id = cmd_id;
	/* 写入帧头 */
	memcpy( Vision_Tx_Buffer, &txPacket->FrameHeader, LEN_FRAME_HEADER );
	/* 写入帧头CRC8 */
	Append_CRC8_Check_Sum( Vision_Tx_Buffer, LEN_FRAME_HEADER);
	Vision.TxPacket.FrameHeader.crc8 = Vision_Tx_Buffer[CRC8];
	
	/* 数据段填充 */
	memcpy( &Vision_Tx_Buffer[DATA], &txPacket->TxData, LEN_TX_DATA );
	/* 写入帧尾CRC16 */
	Append_CRC16_Check_Sum( Vision_Tx_Buffer, LEN_VISION_TX_PACKET);
	Vision.TxPacket.FrameTailer.crc16 = Vision_Tx_Buffer[CRC16];
	
	/* 将打包后的数据包发送给视觉PC */
	for(i = 0; i < LEN_VISION_TX_PACKET; i++) {
		UART4_sendChar(Vision_Tx_Buffer[i]);
	}
	
	/* 发送数据包清零 */
	memset(Vision_Tx_Buffer, 0, VISION_TX_BUFFER_LEN);
}

/* #应用层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	卡尔曼滤波器
 */
float kalman_kYaw_Q = 1;
float kalman_kYaw_R = 15;
float kalman_kPitch_Q = 1;
float kalman_kPitch_R = 10;
void VISION_init(void)
{
	KalmanCreate(&Gimbal_Gyro_Auto_kalmanError[YAW_205], kalman_kYaw_Q, kalman_kYaw_R);
	KalmanCreate(&Gimbal_Gyro_Auto_kalmanError[PITCH_206], kalman_kPitch_Q, kalman_kPitch_R);
}

/**
 *	@brief	反馈视觉标志位
 */
bool VISION_getFlagStatus(Vision_Flag_t flag)
{
	switch(flag)
	{
		case VISION_FLAG_DATA_VALID:
			return Vision.State.rx_data_valid;
		case VISION_FLAG_DATA_UPDATE:
			return Vision.State.rx_data_update;
		case VISION_FLAG_LOCK_TARGET:
			return Vision.RxPacket.RxData.identify_target;
		case VISION_FLAG_LOCK_BUFF:
			return Vision.RxPacket.RxData.identify_buff;
		case VISION_FLAG_CHANGE_ARMOR:
			return (Vision.RxPacket.RxData.identify_buff == 2)?1:0;
		default:
			break;
	}
	return false;
}

/**
 *	@brief	清除视觉标志位
 */
void VISION_clearFlagStatus(Vision_Flag_t flag)
{
	switch(flag)
	{
		case VISION_FLAG_DATA_VALID:
			Vision.State.rx_data_valid = false;
		case VISION_FLAG_DATA_UPDATE:
			Vision.State.rx_data_update = false;
			break;
		default:
			break;
	}
}

/**
 *	@brief	设置视觉模式
 */
void VISION_setMode(Vision_Mode_t mode)
{
	Vision.State.mode = mode;
}

/**
 *	@brief	利用裁判系统的反馈得到自己的颜色
 */
Vision_Color_t VISION_getMyColor(Judge_Info_t *info)
{
	uint8_t my_id;
	my_id = info->GameRobotStatus.robot_id;
	if(my_id > 10) {
		return BLUE;
	} else {
		return RED;
	}
}

/**
 *	@brief	反馈视觉数据是否可信
 */
bool VISION_isDataValid(void)
{
	return VISION_getFlagStatus(VISION_FLAG_DATA_VALID);
}

/**
 *	@brief	遥控设置视觉模式
 *			S2  上	打符
 *				中	手动
 *				下	自瞄
 */
void REMOTE_setVisionMode(RC_Ctl_t *remoteInfo)
{
	uint8_t	sw2;
	sw2 = remoteInfo->rc.s2;
	
	if(sw2 == RC_SW_UP) {
		Vision.State.mode = VISION_MODE_BIG_BUFF;
	} else if(sw2 == RC_SW_MID) {
		Vision.State.mode = VISION_MODE_MANUAL;
	} else if(sw2 == RC_SW_DOWN) {
		Vision.State.mode = VISION_MODE_AUTO;
	}
}

/**
 *	@brief	遥控设置自瞄对象颜色
 *	@note	前提是进入自瞄模式
 *			S1  上	红色
 *				中	不识别
 *				下	蓝色
 */
void REMOTE_setAttackColor(RC_Ctl_t *remoteInfo)
{
	uint8_t	sw1;
	sw1 = remoteInfo->rc.s1;
	
	if(Vision.State.mode == VISION_MODE_AUTO) {
		if(sw1 == RC_SW_UP) {
			Vision.State.attack_color = RED;
		} else if(sw1 == RC_SW_MID) {
			Vision.State.attack_color = NONE;
		} else if(sw1 == RC_SW_DOWN) {
			Vision.State.attack_color = BLUE;
		}
	}
}

/**
 *	@brief	键盘设置视觉模式
 *	@note	鼠标右键进入自瞄模式
 */
void KEY_setVisionMode(RC_Ctl_t *remoteInfo)
{
	static uint8_t mouseLockRFlag = false;	// 鼠标右键锁定
	if(IF_MOUSE_PRESSED_RIGH) {
		if(mouseLockRFlag == false) {	// 进入自瞄模式
			Vision.State.mode = VISION_MODE_AUTO;
		}
		mouseLockRFlag = true;
	} else {
		if(mouseLockRFlag == true) {	// 退出自瞄模式
			//GIMBAL_VISION_AUTO_pidParamsInit(&Gimbal);
		}
		Vision.State.mode = VISION_MODE_MANUAL;
		mouseLockRFlag = false;
	}
}

/**
 *	@brief	获取yaw误差角度( #机械角度 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - yaw_angle为负
 *				位于镜头右边 - yaw_angle为正
 *					云台左摆 - yaw减小
 *					云台右摆 - yaw增大
 */
float VISION_MECH_getYawFeedback(void)
{
 	float yaw_err = 0.0f;
	/* 滤波/死区处理 */
	if(abs(Vision.RxPacket.RxData.yaw_angle) > 0.05f) {	
		yaw_err = Vision.RxPacket.RxData.yaw_angle/360.0f*8192.0f;
	}
	return yaw_err; 	
}

/**
 *	@brief	获取pitch误差角度( #机械角度 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头下方 - pitch_angle为正
 *				位于镜头上方 - pitch_angle为负
 *					云台低头 - pitch增大
 *					云台抬头 - pitch减小
 */
float VISION_MECH_getPitchFeedback(void)
{
	float pitch_err = 0.0f;
	if(abs(Vision.RxPacket.RxData.pitch_angle) > 0.05f) {
		if(abs(Vision.RxPacket.RxData.pitch_angle) > 65.f) {
			pitch_err = 0.0f;
		}
		pitch_err = Vision.RxPacket.RxData.pitch_angle/360.0f*8192.0f;
	} else {
		pitch_err = 0.0f;
	}
	/* 滤波/死区处理 */
	//..
	return pitch_err;
}

/**
 *	@brief	获取yaw误差角度( #放大后的欧拉角 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - yaw_angle为负
 *				位于镜头右边 - yaw_angle为正
 *					云台左摆 - yaw减小
 *					云台右摆 - yaw增大
 */
float vision_gyro_kLPF = 0.1;
float VISION_GYRO_getYawFeedback(void)
{
//	static float last_yaw_err = 0.0f;
//	float yaw_err = 0.0f;
//	/* 滤波/死区处理 */
//	if(abs(Vision.RxPacket.RxData.yaw_angle) > 0.05f) {
//		yaw_err = Vision.RxPacket.RxData.yaw_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
//		yaw_err = vision_gyro_kLPF*yaw_err + (1-vision_gyro_kLPF)*last_yaw_err;
//		last_yaw_err = yaw_err;
//	} else {
//		yaw_err = 0.0f;
//	}
//	return yaw_err; 

	//static float last_yaw_err = 0.0f;	
	float yaw_err = 0.0f;
	
	/* 滤波/死区处理 */
	if(abs(Vision.RxPacket.RxData.yaw_angle) > 0.05f) {
		yaw_err = Vision.RxPacket.RxData.yaw_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	} else {
		yaw_err = 0.0f;
	}
	return yaw_err; 	
}

/**
 *	@brief	获取pitch误差角度( #放大后的欧拉角 )
 *	@note	镜头中心为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头下方 - pitch_angle为正
 *				位于镜头上方 - pitch_angle为负
 *					云台低头 - pitch增大
 *					云台抬头 - pitch减小
 */
float VISION_GYRO_getPitchFeedback(void)
{
	float pitch_err = 0.0f;
	/* 滤波/死区处理 */
	if(abs(Vision.RxPacket.RxData.pitch_angle) > 0.05f) {
		pitch_err = Vision.RxPacket.RxData.pitch_angle * GIMBAL_GYRO_ANGLE_ZOOM_INDEX;
	} else {
		pitch_err = 0.0f;
	}
	return pitch_err;
}

/**
 *	@brief	获取yaw误差像素点，打符专用
 *	@note	左上角为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)为负
 *				位于镜头右边 - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)为正
 *					云台左摆 - yaw减小
 *					云台右摆 - yaw增大
 */
float VISION_AUTO_getPredictError(uint16_t T)
{
//	float predict_angle;
//	float predict_target_speed, relative_speed;
//	float distance;
//	int16_t my_speed;
//	
//	my_speed = g_Gimbal_Motor_Info[YAW].speed;
//	distance = Vision.RxPacket.RxData.distance;
//	
//	predict_target_speed = my_speed + relative_speed;
//	predict_move_x = predict_target_speed * T;
//	predict_angle = atan(predict_move_x/distance);
//	
//	return predict_angle;
}

/**
 *	@brief	获取yaw误差像素点，打符专用
 *	@note	左上角为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头左边 - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)为负
 *				位于镜头右边 - (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW)为正
 *					云台左摆 - yaw减小
 *					云台右摆 - yaw增大
 */
float VISION_BUFF_getYawFeedback(void)
{
	float yaw_err = 0.0f;
	yaw_err = (Vision.RxPacket.RxData.yaw_angle - VISION_MID_YAW);
	return yaw_err;
}

/**
 *	@brief	获取pitch误差角度，打符专用
 *	@note	左上角为(0,0)，把摄像头当成自己的眼睛(单目)
 *			物体位于镜头下方 - (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH)为正
 *				位于镜头上方 - (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH)为负
 *					云台低头 - pitch增大
 *					云台抬头 - pitch减小
 */
float VISION_BUFF_getPitchFeedback(void)
{
	float pitch_err = 0.0f;
	pitch_err = (Vision.RxPacket.RxData.pitch_angle - VISION_MID_PITCH);
	return pitch_err;
}


/**
 *	@brief	视觉信息记录反馈
 *	@note	失联处理可能存在问题
 */
void VISION_recordFeedback(void)
{
	/* 失联判断 */
	if(Vision.State.unconnected_cnt >= 50) {	// 500ms没接收到视觉反馈则认为视觉失联
		Vision.State.rx_data_valid = false;
		memset(&Vision.RxPacket.RxData, 0, sizeof(Vision_Rx_Data_t));	// 接收数据包清零
	}
	/* 裁判系统数据处理 */
	if(Judge_Info.data_valid == true) {
		Vision.State.my_color = VISION_getMyColor(&Judge_Info);
	}
	/* 失联计数自增，UART4_IRQ 清零*/
	Vision.State.unconnected_cnt++;
}

/**
 *	@brief	信息反馈给视觉
 *	@note	
 */
int16_t auto_rate_yaw;
void VISION_outputInfo(void)
{
	auto_rate_yaw = (int16_t)Gimbal_PID[GYRO][YAW_205].Speed.feedback;
	if(abs(auto_rate_yaw) <= 50) {
		auto_rate_yaw = 0;
	} else {
		if(auto_rate_yaw > 0) { // >25 表明右摆
			auto_rate_yaw -= 50;
			//Vision.TxPacket.TxData.rate_yaw = 200;
		} else {	// <-25 表明左摆
			auto_rate_yaw += 50;
			//Vision.TxPacket.TxData.rate_yaw = -200;
		}
	}
	auto_rate_yaw = (int16_t)KalmanFilter(&Gimbal_Gyro_Auto_kalmanError[YAW_205], auto_rate_yaw);
	Vision.TxPacket.TxData.rate_yaw = auto_rate_yaw;

//	Vision.TxPacket.TxData.rate_yaw = g_Gimbal_Motor_Info[YAW_205].speed;
	Vision.TxPacket.TxData.fric_speed = Friction_Pwm_Speed[Friction.speedLevel];
	Vision.TxPacket.TxData.yaw_angle = g_Gimbal_Motor_Info[YAW_205].angle;
	Vision.TxPacket.TxData.pitch_angle = g_Gimbal_Motor_Info[PITCH_206].angle;
}

/* #任务层# ---------------------------------------------------------------------------------------------------------------------------------------*/
/**
 *	@brief	视觉遥控控制
 */
void VISION_rcControlTask(void)
{
//	REMOTE_setVisionMode(&RC_Ctl_Info);
//	REMOTE_setAttackColor(&RC_Ctl_Info);
}

/**
 *	@brief	视觉键盘控制
 */
void VISION_keyControlTask(void)
{
	//KEY_setVisionMode(&RC_Ctl_Info);
}

/**
 *	@brief	手动控制
 */
void VISION_manualControl(void)
{
	/* 关闭自瞄 */
	VISION_sendData(&Vision.TxPacket, CMD_AIM_OFF);
}

/**
 *	@brief	自动控制
 */
void VISION_autoControl(void)
{
	/* 根据自身颜色判断敌军颜色 */
	if(Vision.State.my_color == RED) {
		Vision.State.attack_color = BLUE;
	} else if(Vision.State.my_color == BLUE) {
		Vision.State.attack_color = RED;
	} else if(Vision.State.my_color == NONE) {
		Vision.State.attack_color = NONE;
	}
	
	switch(Vision.State.attack_color)
	{
		case RED:
			VISION_sendData(&Vision.TxPacket, CMD_AIM_RED);
			break;
		case BLUE:
			VISION_sendData(&Vision.TxPacket, CMD_AIM_BLUE);
			break;
		case NONE:
			VISION_sendData(&Vision.TxPacket, CMD_AIM_OFF);
			break;
	}
}

/**
 *	@brief	打符控制
 *	@note	Ctrl+V - 击打小符
 *			Ctrl+F - 击打大符
 *			
 */
void VISION_buffControl(void)
{
	/* 根据自身颜色判断击打的能量机关颜色 */
	if(Vision.State.mode == VISION_MODE_BIG_BUFF) {	// 大符模式
		switch(Vision.State.my_color)
		{
			case RED:
				VISION_sendData(&Vision.TxPacket, CMD_AIM_RED_BUFF);
				break;
			case BLUE:
				VISION_sendData(&Vision.TxPacket, CMD_AIM_BLUE_BUFF);
				break;
			case NONE:
				VISION_sendData(&Vision.TxPacket, CMD_AIM_OFF);
				break;
		}		
	} else if(Vision.State.mode == VISION_MODE_SMALL_BUFF) {	// 小符模式
		switch(Vision.State.my_color)
		{
			case RED:
				VISION_sendData(&Vision.TxPacket, CMD_AIM_RED_BUFF);
				break;
			case BLUE:
				VISION_sendData(&Vision.TxPacket, CMD_AIM_BLUE_BUFF);
				break;
			case NONE:
				VISION_sendData(&Vision.TxPacket, CMD_AIM_OFF);
				break;
		}			
	}
}

/**
 *	@brief	视觉控制
 */
void VISION_control(void)
{
	/*----信息读入----*/
	VISION_recordFeedback();
	/*----期望修改----*/
	if(Flag.remote.FLAG_mode == RC) {
		VISION_rcControlTask();
	} else if(Flag.remote.FLAG_mode == KEY) {
		VISION_keyControlTask();
	}
	/*----信息输出----*/
	VISION_outputInfo();
	/*----最终输出----*/
	switch(Vision.State.mode)
	{
		case VISION_MODE_MANUAL:
			VISION_manualControl();	// 手动控制
			break;
		case VISION_MODE_AUTO:
			VISION_autoControl();	// 自动控制
			break;
		case VISION_MODE_BIG_BUFF:
		case VISION_MODE_SMALL_BUFF:
			VISION_buffControl();	// 打符控制
			break;
		default:
			break;
	}
}

