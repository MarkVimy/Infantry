/**
 * @file        judge.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        19-October-2019
 * @brief       This file includes the JUDGE Protocols(裁判系统协议) external functions 
 * 				(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		
 */

/**
 *	# 裁判系统使用USART5作为通讯接口
 *	通信协议格式:
 *				帧头		命令段			数据段		帧尾
 *	域		frame_header 	cmd_id 			data  		frame_tail(CRC16,整包校验)
 *	字节		5			  2		  		n			2
 *	
 *	帧头格式:
 *		  起始字节(0xA5)	数据帧长度		包序号		帧头CRC8校验
 *	域			SOF 	 	data_length    	seq 	 	CRC8
 *	字节		0				2			1			1
 */

/* Includes ------------------------------------------------------------------*/
#include "judge.h"

#include "uart5.h"
#include "crc.h"

/* Private typedef -----------------------------------------------------------*/
/*------Cmd ID-------*/
// (0x0001)比赛状态				发送频率: 1Hz
// (0x0002)比赛结果 			发送频率: 比赛结束后发送
// (0x0003)机器人血量数据  		发送频率: 1Hz
// (0x0101)场地事件数据			发送频率: 事件改变后发送
// (0x0102)补给站动作标识		发送频率: 动作改变后发送
// (0x0103)请求补给站补弹子弹	发送频率: 上限10Hz(RM对抗赛尚未开放)
// (0x0104)裁判警告信息			发送频率: 警告发生后发送
// (0x0201)比赛机器人状态		发送频率: 10Hz
// (0x0202)实时功率热量			发送频率: 50Hz
// (0x0203)机器人位置			发送频率: 10Hz
// (0x0204)机器人增益			发送频率: 状态改变后发送
// (0x0205)空中机器人能量状态	发送频率: 10Hz
// (0x0206)伤害状态				发送频率: 伤害发生后发送
// (0x0207)实时射击信息			发送频率: 射击后发送
// (0x0208)子弹剩余发射数		发送频率: 1Hz发送(空中机器人和哨兵机器人主控发送)
// (0x0301)机器人间交互数据		发送频率: 发送方触发发送(上线10Hz)
typedef enum {
	ID_GAME_STATUS 					= 0x0001,	// 比赛状态			
	ID_GAME_RESULT 					= 0x0002,	// 比赛结果 		
	ID_GAME_ROBOT_HP 				= 0x0003,	// 机器人血量数据  	
	ID_EVENT_DATA 					= 0x0101,	// 场地事件数据		
	ID_SUPPLY_PROJECTILE_ACTION 	= 0x0102,	// 补给站动作标识	
	ID_SUPPLY_PROJECTILE_BOOKING 	= 0x0103,	// 请求补给站补弹子弹	
	ID_REFEREE_WARNING 				= 0x0104,	// 裁判警告信息
	ID_GAME_ROBOT_STATUS 			= 0x0201,	// 比赛机器人状态
	ID_POWER_HEAT_DATA 				= 0x0202,	// 实时功率热量数据
	ID_GAME_ROBOT_POS				= 0x0203,	// 机器人位置
	ID_BUFF							= 0x0204,	// 机器人增益
	ID_AERIAL_ROBOT_ENERGY			= 0x0205,	// 空中机器人能量状态
	ID_ROBOT_HURT					= 0x0206,	// 机器人伤害状态
	ID_SHOOT_DATA					= 0x0207,	// 实时射击信息
	ID_BULLET_REMAINING				= 0x0208,	// 子弹剩余发射数
	ID_COMMUNICATION				= 0x0301,	// 机器人间交互数据(发送方触发发送)
} Judge_Cmd_ID_t;

/* 帧字节偏移 */
typedef enum {
	FRAME_HEADER	= 0,
	CMD_ID			= 5,
	DATA_SEG		= 7
} Judge_Frame_Offset_t;

/* 帧头字节偏移 */
typedef enum {
	SOF			= 0,
	DATA_LENGTH	= 1,
	SEQ			= 3,
	CRC8		= 4
} Judge_Frame_Header_Offset_t;

typedef enum {
	/* Std */
	LEN_FRAME_HEAD 	= 5,	// 帧头长度
	LEN_CMD_ID 		= 2,	// 命令码长度
	LEN_FRAME_TAIL 	= 2,	// 帧尾CRC16
	/* Ext */
	LEN_GAME_STATUS 				= 3,
	LEN_GAME_RESULT 				= 1,
	LEN_GAME_ROBOT_HP 				= 28,
	LEN_EVENT_DATA					= 4,
	LEN_SUPPLY_PROJECTILE_ACTION	= 3,
	LEN_SUPPLY_PROJECTILE_BOOKING	= 2,
	LEN_REFEREE_WARNING				= 2,
	LEN_GAME_ROBOT_STATUS			= 15,
	LEN_POWER_HEAT_DATA 			= 14,
	LEN_GAME_ROBOT_POS				= 16,
	LEN_BUFF		 				= 1,
	LEN_AERIAL_ROBOT_ENERGY 		= 3,
	LEN_ROBOT_HURT					= 1,
	LEN_SHOOT_DATA					= 6,
	LEN_BULLET_REMAINING	 		= 2,
} Judge_Data_Length_t;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define JUDGE_FRAME_HEADER		(0xA5)

/* Private variables ---------------------------------------------------------*/
//ext_game_status_t 				GameStatus;					// 0x0001
//ext_game_result_t 				GameResult;					// 0x0002
//ext_game_robot_HP_t 			GameRobotHP;				// 0x0003
//ext_event_data_t				EventData;					// 0x0101
//ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
//ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
//ext_referee_warning_t			RefereeWarning;				// 0x0104
//ext_game_robot_status_t			GameRobotStatus;			// 0x0201
//ext_power_heat_data_t			PowerHeatData;				// 0x0202
//ext_game_robot_pos_t			GameRobotPos;				// 0x0203
//ext_buff_t						Buff;						// 0x0204
//ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
//ext_robot_hurt_t				RobotHurt;					// 0x0206
//ext_shoot_data_t				ShootData;					// 0x0207
//ext_bullet_remaining_t			BulletRemaining;			// 0x0208

//std_frame_header_t				FrameHeader;	// 帧头信息

/* ## Global variables ## ----------------------------------------------------*/
Judge_Info_t Judge_Info = 
{
	.frame_length = 0,
	.cmd_id = 0,
	.err_cnt = 0,
	.data_valid = false,
};	// 方便调试的时候用

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* API functions -------------------------------------------------------------*/
/**
 *	@brief	裁判系统读取实时数据
 *	@note
 *			# 将递归那一部分(判断+调用)改成数组的方式就不会进入HardFault
 *				至于具体原因还没想清楚
 *			# 增大DMA的回环数据大小(Buffer Size)可以使得错误增加较缓慢
 */
bool JUDGE_readData(uint8_t *rxBuf)
{
	uint8_t  res = false;
	uint16_t frame_length;
	uint16_t cmd_id;	

	if( rxBuf == NULL )
	{
		return -1;
	}
	
	memcpy(&Judge_Info.FrameHeader, rxBuf, LEN_FRAME_HEAD);
	
	/* 帧首字节是否为0xA5 */
	if(rxBuf[SOF] == JUDGE_FRAME_HEADER) 
	{
		/* 帧头CRC8校验 */
		if(Verify_CRC8_Check_Sum(rxBuf, LEN_FRAME_HEAD) == true) 
		{
			/* 统计一帧的总数据长度，用于CRC16校验 */
			frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + Judge_Info.FrameHeader.data_length + LEN_FRAME_TAIL;
			Judge_Info.frame_length = frame_length;
			
			if(Verify_CRC16_Check_Sum(rxBuf, frame_length) == true)
			{
				res = true;
				
				cmd_id = (rxBuf[CMD_ID+1] << 8 | rxBuf[CMD_ID]);
				Judge_Info.cmd_id = cmd_id;
				
				switch(cmd_id)
				{
					case ID_GAME_STATUS:
						memcpy(&Judge_Info.GameStatus, (rxBuf+DATA_SEG), LEN_GAME_STATUS);
						break;
					case ID_GAME_RESULT:
						memcpy(&Judge_Info.GameResult,  (rxBuf+DATA_SEG), LEN_GAME_RESULT);
						break;
					case ID_GAME_ROBOT_HP:
						memcpy(&Judge_Info.GameRobotHP, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_HP);
						break;
					case ID_EVENT_DATA:
						memcpy(&Judge_Info.EventData, (rxBuf+DATA_SEG), LEN_EVENT_DATA);
						break;
					case ID_SUPPLY_PROJECTILE_ACTION:
						memcpy(&Judge_Info.SupplyProjectileAction, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_ACTION);
						break;
					case ID_SUPPLY_PROJECTILE_BOOKING:
						memcpy(&Judge_Info.SupplyProjectileBooking, (rxBuf+DATA_SEG), LEN_SUPPLY_PROJECTILE_BOOKING);
						break;
					case ID_REFEREE_WARNING:
						memcpy(&Judge_Info.RefereeWarning, (rxBuf+DATA_SEG), LEN_REFEREE_WARNING);
						break;
					case ID_GAME_ROBOT_STATUS:
						memcpy(&Judge_Info.GameRobotStatus, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_STATUS);
						break;
					case ID_POWER_HEAT_DATA:
						memcpy(&Judge_Info.PowerHeatData, (rxBuf+DATA_SEG), LEN_POWER_HEAT_DATA);
						break;
					case ID_GAME_ROBOT_POS:
						memcpy(&Judge_Info.GameRobotPos, (rxBuf+DATA_SEG), LEN_GAME_ROBOT_POS);
						break;
					case ID_BUFF:
						memcpy(&Judge_Info.Buff, (rxBuf+DATA_SEG), LEN_BUFF);
						break;
					case ID_AERIAL_ROBOT_ENERGY:
						memcpy(&Judge_Info.AerialRobotEnergy, (rxBuf+DATA_SEG), LEN_AERIAL_ROBOT_ENERGY);
						break;
					case ID_ROBOT_HURT:
						memcpy(&Judge_Info.RobotHurt, (rxBuf+DATA_SEG), LEN_ROBOT_HURT);
						break;
					case ID_SHOOT_DATA:
						memcpy(&Judge_Info.ShootData, (rxBuf+DATA_SEG), LEN_SHOOT_DATA);
						break;
					case ID_BULLET_REMAINING:
						memcpy(&Judge_Info.BulletRemaining, (rxBuf+DATA_SEG), LEN_BULLET_REMAINING);
						break;
					case ID_COMMUNICATION:
						break;
				}
			}
		}

		/* 帧尾CRC16下一字节是否为0xA5 */
		if(rxBuf[ frame_length ] == JUDGE_FRAME_HEADER)
		{
			/* 如果一个数据包出现了多帧数据就再次读取 */
			JUDGE_readData( &rxBuf[frame_length] );
		}
	}
	
	Judge_Info.data_valid = res;
	if(Judge_Info.data_valid != true)
	{
		Judge_Info.err_cnt++;
		Judge_Info.data_valid = false;
	}
	else
	{
		Judge_Info.data_valid = true;
	}
		
	return res;
}

/**
 *	@brief	用来判断此时裁判系统数据的正确性
 */
bool JUDEG_ifDataValid(void)
{
	return Judge_Info.data_valid;
}


