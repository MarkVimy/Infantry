#ifndef __JUDGE_H
#define __JUDGE_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
#define JUDGE_VERSION	19	// 2019/07/11(V2.0)

/* Global TypeDef ------------------------------------------------------------*/
/* 自定义帧头 Byte:	  5	*/
typedef __packed struct
{
	uint8_t  sof;
	uint16_t data_length;
	uint8_t  seq;
	uint8_t  crc8;
} std_frame_header_t;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;			// 比赛类型
	uint8_t game_progress : 4;		// 比赛阶段
	uint16_t stage_remain_time;		// 当前阶段剩余时间
} ext_game_status_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte: 28   机器人血量数据数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
	uint16_t red_2_robot_HP;	// 红2工程机器人血量
	uint16_t red_3_robot_HP;	// 红3步兵机器人血量
	uint16_t red_4_robot_HP;	// 红4步兵机器人血量
	uint16_t red_5_robot_HP;	// 红5步兵机器人血量
	uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
	uint16_t red_base_HP;		// 红方基地血量
	uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
	uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
	uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
	uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
	uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
	uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
	uint16_t blue_base_HP;		// 蓝方基地血量
} ext_game_robot_HP_t; 


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  3    请求补给站补弹子弹数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 

/* ID: 0X0104  Byte:  2	   裁判警告信息 */
typedef __packed struct
{
	uint8_t level;
	uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;   //机器人ID，可用来校验发送
	uint8_t robot_level;  //1一级，2二级，3三级
	uint16_t remain_HP;  //机器人剩余血量
	uint16_t max_HP; //机器人满血量
	uint16_t shooter_heat0_cooling_rate;  //机器人 17mm 子弹热量冷却速度 单位 /s
	uint16_t shooter_heat0_cooling_limit;   // 机器人 17mm 子弹热量上限
	uint16_t shooter_heat1_cooling_rate;   
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t mains_power_gimbal_output : 1;  
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_status_t; 


/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   
	uint16_t chassis_current;    
	float chassis_power;   //瞬时功率 
	uint16_t chassis_power_buffer;//60焦耳缓冲能量(飞坡根据规则增加至250J)
	uint16_t shooter_heat0;//17mm
	uint16_t shooter_heat1;  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint16_t attack_time; // 手册里面定义成uint8_t
} ext_aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;   
	float bullet_speed;  
} ext_shoot_data_t; 


/* ID: 0x0208  Byte:  2    子弹剩余发射数数据 */
typedef __packed struct
{
	uint16_t bullet_remaining_num;
} ext_bullet_remaining_t;


/**
 *	-----------------------------------
 *	# 机器人间交互数据暂时不看！
 *	-----------------------------------
 */

/* 
	交互数据，包括一个统一的数据段头结构，
	包含了内容 ID，发送者以及接受者的 ID 和内容数据段，
	整个交互数据的包总共长最大为 128 个字节，
	减去 frame_header,cmd_id,frame_tail 以及数据段头结构的 6 个字节，
	故而发送的内容数据段最大为 113。
	整个交互数据 0x0301 的包上行频率为 10Hz。

	机器人 ID：
	1，英雄(红)；
	2，工程(红)；
	3/4/5，步兵(红)；
	6，空中(红)；
	7，哨兵(红)；
	11，英雄(蓝)；
	12，工程(蓝)；
	13/14/15，步兵(蓝)；
	16，空中(蓝)；
	17，哨兵(蓝)。 
	客户端 ID： 
	0x0101 为英雄操作手客户端( 红) ；
	0x0102 ，工程操作手客户端 ((红 )；
	0x0103/0x0104/0x0105，步兵操作手客户端(红)；
	0x0106，空中操作手客户端((红)； 
	0x0111，英雄操作手客户端(蓝)；
	0x0112，工程操作手客户端(蓝)；
	0x0113/0x0114/0x0115，操作手客户端步兵(蓝)；
	0x0116，空中操作手客户端(蓝)。 
*/
/* 交互数据接收信息：0x0301  */
typedef __packed struct 
{ 
	uint16_t data_cmd_id;    
	uint16_t send_ID;    
	uint16_t receiver_ID; 
} ext_student_interactive_header_data_t; 


/* 
	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180
	发送频率：上限 10Hz


	1.	客户端 客户端自定义数据：cmd_id:0x0301。内容 ID:0xD180。发送频率：上限 10Hz 
	字节偏移量 	大小 	说明 				备注 
	0 			2 		数据的内容 ID 		0xD180 
	2 			2 		送者的 ID 			需要校验发送者机器人的 ID 正确性 
	4 			2 		客户端的 ID 		只能为发送者机器人对应的客户端 
	6 			4 		自定义浮点数据 1 	 
	10 			4 		自定义浮点数据 2 	 
	14 			4 		自定义浮点数据 3 	 
	18 			1 		自定义 8 位数据 4 	 

*/
typedef __packed struct 
{ 
	float data1; 
	float data2; 
	float data3; 
	uint8_t masks; 
} client_custom_data_t;


/* 
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz  

	字节偏移量 	大小 	说明 			备注 
	0 			2 		数据的内容 ID 	0x0200~0x02FF 
										可以在以上 ID 段选取，具体 ID 含义由参赛队自定义 
	
	2 			2 		发送者的 ID 	需要校验发送者的 ID 正确性， 
	
	4 			2 		接收者的 ID 	需要校验接收者的 ID 正确性，
										例如不能发送到敌对机器人的ID 
	
	6 			n 		数据段 			n 需要小于 113 

*/
typedef __packed struct 
{ 
	uint8_t data[10]; //数据段,n需要小于113
} robot_interactive_data_t;

typedef struct 
{
	uint16_t frame_length;	// 整帧长度(调试时使用)
	uint16_t cmd_id;		// 命令码(调试时使用)
	uint16_t err_cnt;		// 错帧数(调试时使用)
	bool	 data_valid;	// 数据有效性
	std_frame_header_t				FrameHeader;	// 帧头信息
	ext_game_status_t 				GameStatus;					// 0x0001
	ext_game_result_t 				GameResult;					// 0x0002
	ext_game_robot_HP_t 			GameRobotHP;				// 0x0003
	ext_event_data_t				EventData;					// 0x0101
	ext_supply_projectile_action_t	SupplyProjectileAction;		// 0x0102
	ext_supply_projectile_booking_t SupplyProjectileBooking;	// 0x0103
	ext_referee_warning_t			RefereeWarning;				// 0x0104
	ext_game_robot_status_t			GameRobotStatus;			// 0x0201
	ext_power_heat_data_t			PowerHeatData;				// 0x0202
	ext_game_robot_pos_t			GameRobotPos;				// 0x0203
	ext_buff_t						Buff;						// 0x0204
	ext_aerial_robot_energy_t		AerialRobotEnergy;			// 0x0205
	ext_robot_hurt_t				RobotHurt;					// 0x0206
	ext_shoot_data_t				ShootData;					// 0x0207
	ext_bullet_remaining_t			BulletRemaining;			// 0x0208	
}Judge_Info_t;


/* ## Global Variables Prototypes ## -----------------------------------------*/
extern Judge_Info_t Judge_Info;

/* API functions Prototypes --------------------------------------------------*/
bool JUDGE_readData(uint8_t *rxBuf);
bool JUDEG_ifDataValid(void);

#endif
