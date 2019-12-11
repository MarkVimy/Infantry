/**
 * @file        can.c
 * @author      MaWeiming
 * @Version     V1.0
 * @date        22-September-2019
 * @brief       This file includes the CAN driver external functions 
 * 							(based on ST Peripherals Libaray Keil.STM32F4xx_DFP.2.13.0)
 * @Version		V1.1(10-October-2019)
 *				1. 加入CAN2
 */

/**
 *	# CAN
 *	@brief
 *		显性电平 - 逻辑0
 *		隐性电平 - 逻辑1
 *
 *	@note
 *	①数据帧
 *					帧起始 仲裁段 	控制段 		数据段 CRC段 ACK段 帧结束
 *	标准帧		1			11			1+1+4			0-64		15+1		4			7
 *	扩展帧		1			29			1+1+4	
 *		# 帧起始
 *			1个位的显性电平
 *
 *		# 仲裁段
 *			1. 表示数据优先级的段
 *			2. ID(高位在前，低位在后)
 *				基本ID禁止高7位为隐性(不能 ID = 1111111xxxxb
 *			3. RTR
 *				远程请求位	
 *				0 - 数据帧
 *				1 - 远程帧
 *			4. SRR
 *				替代远程请求帧(设置为1(隐性电平))
 *			5. IDE
 *				标识符选择位
 *				0 - 标准标识符
 *				1 - 扩展标识符
 *			6. 格式
 *				标准 - 11位基本ID + RTR
 *				扩展 - 11位基本ID + SRR + IDE
 *	
 *		# 控制段
 *			1. r0, r1
 *				保留位
 *				发送必须是显性电平，接收可以是隐性电平
 *			2. DLC
 *				数据长度码
 *				0 - 8(表示收发数据字节长度)
 *
 *		# 数据段
 *			0 - 8 字节，MSB(最高位)开始输出
 *		
 *		# CRC段
 *			检查帧传输错误
 *			15位CRC顺序 + 1位CRC界定符(分隔位)
 *			计算范围：帧起始、仲裁段、控制段、数据段
 *	
 *		# ACK段
 *			4位ACK槽(ACK Slot) + 1位ACK界定符
 *			1. 发送单元ACK段
 *				发送2个隐性位
 *			2. 接收单元ACK段
 *				接收到正确消息的单元，
 *				在ACK槽发送显性位，通知发送单元，正常接收结束。称之为发送ACK/返回ACK。
 *		
 *		# 帧结束
 *			7个位的隐性电平
 *
 *	②总线仲裁
 *		1. 先发送显性电平者抢占发送权
 *		2. 同时发送显性电平则在仲裁段中选出优先级最高者
 *			 逐位比较，显性电平者胜出、隐性电平者淘汰(进入下一轮竞争)
 *
 *	③ 位时序
 *		1. 位速率
 *			发送单元在非同步情况下每秒钟发送的位数
 *		2. 位分段
 *			1个位 = 同步段(SS) + 传播时间段(PTS) + 相位缓冲段1(PBS1) + 相位缓冲段2(PBS2)
 *			上述段又可用 Time Quantum(tq) - 最小时间单位构成
 *			位<段<<tq>>> 称为位时序
 *		3. 位时间
 *			位时间 = 1/波特率
 *			通过设定位时序，多个单元可同时采样，也可任意设定采样点
 *		4. 波特率
 *			波特率 = 1/正常的位时间
 *			正常的位时间 = 1*tq + tBS1 + tBS2
 *				tBS1 = tq * (TS1[3:0] + 1)
 *				tBS2 = tq * (TS2[2:0] + 1)
 *				tq = (BRP[9:0] + 1) * tPCLK
 *				tPCLK = APB时钟的时间周期
 *				BRP[9:0]、TS1[3:0]、TS2[2:0] 在 CAN_BTR寄存器中定义
 *	
 *				BRP[9:0] = 2, APB1 - 42Mhz
 *					tq = 3 / 42 (us) = 1/14(us)
 *				tBS1 = 9tq, tBS2 = 4tq,
 *					正常位时间 = tq + 9tq + 4tq = 14tq
 *				  波特率 = 1 / 14tq = 14/14(us) = 1MHz (刚好匹配调试设备的总线比特率)
 *				
 */

/**
 *	# 调试设备
 *	直流无刷电机M3508 + 电调C620
 *	1. CAN总线比特率为1Mbps
 *	2. C620支持两种控制方式(二选一 + 断电切换)
 *		 CAN / PWM(可进行固件升级)
 *	
 *		# 电调C620
 *		① SET按键操作		
 *			1. 独立设置
 *			2. 快速设置(<=8个)
 *			3. 电机校准
 *		② CAN通信协议
 *			1. 电调接收报文格式(标准帧)
 *				- 控制指令(主控 -> 电调)
 *				0x200 - 电调ID 1 - 4
 *				0x1FF	- 电调ID 5 - 8
 *				电调电流值范围： -16384 ~ 0 ~ 16384
 *													-20A  ~ 0 ~ 20A
 *						发现特征值： -8192	~ 0 ~ 8192
 *													-10A		0		10A
 *				- 反馈格式
 *				0x200 + 电调ID
 *				eg. ID1 -> 0x201
 *				DATA[0] - 转子机械角度高8位
 *				DATA[1] - 转子机械角度低8位
 *				DATA[2] - 转子转速高8位
 *				DATA[3] - 转子转速低8位
 *				DATA[4] - 转子转矩电流高8位
 *				DATA[5] - 转子转矩电流低8位
 *				DATA[6] - 电机温度
 *				DATA[7] - Null
 *	
 *				发送频率 - 1Khz(RoboMaster Assistant软件中修改)	
 *				转子机械角度范围: 0 ~ 8191(0°~ 360°) 
 *										精度: 0.0439453125° = 0.044
 *				转子转速值单位: RPM
 *				电机温度单位：℃
 *			2. 
 *	
 *		# 直流无刷电机M3508
 *			搭配C620电调实现正弦驱动
 *			减速箱减速比19.2:1
 *			位置反馈 + 温度检测
 *	
 *			1. 特征参数
 *			- 搭配C620
 *			额定电压	24V
 *			额定电流 10A
 */

/**
 *	# 底盘设备
 *	直流无刷电机M3510 + 电调820R
 *	1. CAN总线比特率为1Mbps
 *
 *		# 电调820R
 *		① 固件区分		
 *			- SYS系统指示灯
 *			1. 绿灯常亮						仅可驱动RM3510电机
 *			2. 绿灯每2秒闪烁1次		仅可驱动RM2310电机
 *			3. 绿灯每2秒闪烁1次		仅可驱动RM2006电机
 *			- 提示音
 *			1. 1234567						RM3510减速电机固件
 *			2. 7654321						RM2310减速电机固件
 *			3. 1313221						RM2006减速电机固件 
 *
 *		② CAN通信协议
 *			1. 电调接收报文格式(标准帧)
 *				- 控制指令(主控 -> 电调)
 *				0x200 - 电调ID 1 - 4
 *				电调电流值范围： -32768 ~ 0 ~ 32767
 *								-  A  ~ 0 ~   A
 *				电调内部做了电流限制，实际有效电流输入范围:
 *				RM3510_V1_2_5_0.bin.enc: [ -16384  : +16384]
 *				RM2310_V1_2_5_1.bin.enc: [ -8000   :  +8000]
 *				RM2006_V1_2_5_2.bin.enc: [ -8000   :  +8000]
 *	
 *				- 反馈格式
 *				0x200 + 电调ID
 *				eg. ID1 -> 0x201
 *				DATA[0] - 转子机械角度高8位
 *				DATA[1] - 转子机械角度低8位
 *				DATA[2] - 转子转速高8位
 *				DATA[3] - 转子转速低8位
 *				DATA[4] - Null
 *				DATA[5] - Null
 *				DATA[6] - Null
 *				DATA[7] - Null
 *
 *				发送频率 - 1Khz(RoboMaster Assistant软件中修改)	
 *				转子机械角度范围: 0 ~ 8191(0°~ 360°) 
 *				精度: 0.0439453125° = 0.044
 *				转子转速值单位: RPM
 *
 *		③电调校准
 *			- 拨码开关
 * 			ON
 *			□□□□
 *			■■■■
 *			1	2	3	4
 *			1	- Bit0	标定设备ID
 *			2 - Bit1	标定设备ID
 *			3 - Bit2	标定设备ID
 *			4 - Bit3	是否接入CAN总线120Ω终端电阻
 *
 *			[Bit2:Bit0]
 *			000b - 电调进入校准模式		
 *			001b - ID1
 *			010b - ID2
 *			011b - ID3
 *			100b - ID4
 *
 *		# 直流无刷电机M3510
 *			减速箱减速比19:1
 *			位置反馈
 *	
 *			1. 特征参数
 *			- 搭配820R
 *			额定电压	24V
 *			额定电流 	
 */

/**
 *	# 云台设备
 *	直流无刷电机GM6020
 *	1. 内部集成驱动器的高可靠性直流无刷电机
 *	2. 低转速、大扭矩。
 *	3. 驱动器采用磁场定向控制(FOC)算法，配合高精度的角度传感器，
 *		 能实现精确的力矩和位置控制。
 *	4. 电机具备异常提示和保护功能，支持多种通信方式，方便控制和省升级。
 *	5. CAN总线比特率为1Mbps
 *
 *		# 内部集成电调
 *		① 系统状态区分		
 *			- 电机指示灯
 *			1. 绿灯每隔1秒闪N次		正常工作，当前驱动器ID为N
 *			2. 绿灯慢闪						PWM通信正常
 *			3. 绿灯常亮						PWM行程校准中
 *
 *		② CAN通信协议
 *			1. 电调接收报文格式(标准帧)
 *				- 控制指令(主控 -> 电调) - 电压驱动
 *				0x1FF - 电调ID 1 - 4
 *				0x2FF	- 电调ID 5 - 7
 *				电调电压值范围： -30000 ~ 0 ~ 30000
 *													 -24V  ~ 0 ~  24V
 *	
 *																			0x1FF	0x2FF
 *				DATA[0] - 电压给定值高8位 - 	ID1		ID5
 *				DATA[1] - 电压给定值低8位 - 	ID1		ID5
 *				DATA[2] - 电压给定值高8位 - 	ID2		ID6
 *				DATA[3] - 电压给定值低8位 - 	ID2		ID6
 *				DATA[4] - 电压给定值高8位 - 	ID3		ID7
 *				DATA[5] - 电压给定值低8位 - 	ID3		ID7
 *				DATA[6] - 电压给定值高8位 - 	ID4		Null
 *				DATA[7] - 电压给定值低8位 - 	ID4		Null
 *				
 *				- 反馈格式
 *				0x204 + 电调ID
 *				eg. ID1 -> 0x205
 *				DATA[0] - 转子机械角度高8位
 *				DATA[1] - 转子机械角度低8位
 *				DATA[2] - 转子转速高8位
 *				DATA[3] - 转子转速低8位
 *				DATA[4] - 实际转矩电流高8位
 *				DATA[5] - 实际转矩电流低8位
 *				DATA[6] - 电机温度
 *				DATA[7] - Null
 *
 *				发送频率 - 1Khz(RoboMaster Assistant软件中修改)	
 *				转子机械角度范围: 0 ~ 8191(0°~ 360°) 
 *										精度: 0.0439453125° = 0.044
 *				转子转速值单位: RPM
 *
 *		③电调校准
 *			- 拨码开关
 * 			ON
 *			□□□□
 *			■■■■
 *			1	2	3	4
 *			1	- Bit0	标定设备ID
 *			2 - Bit1	标定设备ID
 *			3 - Bit2	标定设备ID
 *			4 - Bit3	是否接入CAN总线120Ω终端电阻
 *
 *			[Bit2:Bit0]
 *			000b - 无效	
 *			001b - ID1
 *			010b - ID2
 *			011b - ID3
 *			100b - ID4
 *			101b - ID5
 *			110b - ID6
 *			111b - ID7	
 *
 */
 
/**
 *	# 拨弹电机设备
 *	直流无刷电机M2006 + 电调C610
 *	1. 最高支持10A的持续电流
 *	2. 驱动器采用磁场定向控制(FOC)算法，实现对电机转矩的精确控制。
 *	3. 电机具备异常提示和保护功能，CAN总线指令控制。
 *	4. CAN总线比特率为1Mbps
 *
 *	# 电调C610
 *		① 系统状态区分		
 *			- 电机指示灯
 *			1. 绿灯每隔1秒闪N次		正常工作，当前驱动器ID为N
 *			2. 橙灯常亮						当前电调处于快速设置ID状态
 *			3. 绿灯快亮						当前电调处于校准模式
 *			4. ...
 *
 *		② CAN通信协议
 *			1. 电调接收报文格式(标准帧)
 *				- 控制指令(主控 -> 电调) - 电压驱动
 *				0x200 - 电调ID 1 - 4
 *				0x1FF	- 电调ID 5 - 7
 *				电调电流值范围： -10000 ~ 0 ~ 10000
 *								-10A ~ 0 ~  	10A
 *	
 *																			0x200	0x1FF
 *				DATA[0] - 控制电流值高8位 - 	ID1		ID5
 *				DATA[1] - 控制电流值低8位 - 	ID1		ID5
 *				DATA[2] - 控制电流值高8位 - 	ID2		ID6
 *				DATA[3] - 控制电流值低8位 - 	ID2		ID6
 *				DATA[4] - 控制电流值高8位 - 	ID3		ID7
 *				DATA[5] - 控制电流值低8位 - 	ID3		ID7
 *				DATA[6] - 控制电流值高8位 - 	ID4		ID8
 *				DATA[7] - 控制电流值低8位 - 	ID4		ID8
 *				
 *				- 反馈格式
 *				0x200 + 电调ID
 *				eg. ID1 -> 0x201
 *				DATA[0] - 转子机械角度高8位
 *				DATA[1] - 转子机械角度低8位
 *				DATA[2] - 转子转速高8位
 *				DATA[3] - 转子转速低8位
 *				DATA[4] - 实际输出转矩高8位
 *				DATA[5] - 实际输出转矩低8位
 *				DATA[6] - Null
 *				DATA[7] - Null
 *
 *				发送频率 - 1Khz(RoboMaster Assistant软件中修改)	
 *				转子机械角度范围: 0 ~ 8191(0°~ 360°) 
 *										精度: 0.0439453125° = 0.044
 *				转子转速值单位: RPM
 *
 *		③电调校准
 *			- SET按键
 *			
 */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "exti.h"
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
 *	@var	CAN_DefaultParams 
 *	@brief
 */
static CAN_InitTypeDef CAN_DefaultParams = 
{
	.CAN_TTCM = DISABLE,	// 非时间触发通信模式
	.CAN_ABOM = DISABLE,	// 软件自动离线管理
	.CAN_AWUM = DISABLE,	// 睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	.CAN_NART = DISABLE,	// 禁止报文自动传送 若七个电机接一个CAN 会影响发送 此时可改为ENABLE
	.CAN_RFLM = DISABLE,	// 报文不锁定，新的覆盖旧的
	.CAN_TXFP = ENABLE,		// 优先级由报文标识符决定
	.CAN_BS1	= CAN_BS1_9tq,
	.CAN_BS2	= CAN_BS2_4tq,
	.CAN_Mode = CAN_Mode_Normal,
	.CAN_Prescaler = 3,
	.CAN_SJW	= CAN_SJW_1tq,
};

static CAN_FilterInitTypeDef CAN_Filter_DefaultParams =
{
	.CAN_FilterNumber = 0,  					// 过滤器0
	.CAN_FilterMode = CAN_FilterMode_IdMask,   	// 屏蔽模式
	.CAN_FilterScale = CAN_FilterScale_32bit,   // 32位宽
	.CAN_FilterFIFOAssignment = 0,              // 过滤器0关联到FIFO0
	.CAN_FilterActivation = ENABLE,   			// 激活过滤器
	.CAN_FilterIdHigh = 0x0000,                 // 32位ID
	.CAN_FilterIdLow = 0x0000,
	.CAN_FilterMaskIdHigh = 0x0000,             // 32位Mask
	.CAN_FilterMaskIdLow = 0x0000,	
};

/* ## Global variables ## ----------------------------------------------------*/
MOTOR_Info_t g_Chassis_Motor_Info[CHASSIS_MOTOR_COUNT];
MOTOR_Info_t g_Gimbal_Motor_Info[GIMBAL_MOTOR_COUNT];
MOTOR_Info_t g_Revolver_Motor_Info;

/* Private function prototypes -----------------------------------------------*/
static void CAN1_GPIO_init(void);
static void CAN2_GPIO_init(void);
static int16_t getMotorAngle(CanRxMsg *rxMsg);
static int16_t getMotorSpeed(CanRxMsg *rxMsg);
//static int16_t getMotorCurrent(CanRxMsg *rxMsg);
static uint8_t getMotorTemperature(CanRxMsg *rxMsg);
static void calMotorAngleSum(int16_t angle_now, int16_t angle_pre, int32_t *angle_sum);

/* Private functions ---------------------------------------------------------*/
/**
 *	@brief	CAN1 GPIO初始化
 *	@note	PA11 - CAN1_RX
 *			PA12 - CAN1_TX
 */
static void CAN1_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// 初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);// 初始化PA11,PA12	
	
	// 引脚复用映射配置
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // GPIOA12复用为CAN1	
}

/**
 *	@brief	CAN2 GPIO初始化
 *	@note	PB12 - CAN2_RX
 *			PB13 - CAN2_TX
 */
static void CAN2_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	// 初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);// 初始化PB12,PB13	
	
	// 引脚复用映射配置
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); // GPIOB12复用为CAN2
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); // GPIOB13复用为CAN2	
}

/**
 *	@brief	从CAN报文中反馈电机的机械角度
 */
static int16_t getMotorAngle(CanRxMsg *rxMsg)
{
	int16_t angle;
	angle = ((int16_t)rxMsg->Data[0] << 8 | rxMsg->Data[1]);
	return angle;
}

/**
 *	@brief	从CAN报文中反馈电机的转速
 */
static int16_t getMotorSpeed(CanRxMsg *rxMsg)
{
	int16_t speed;
	speed = ((int16_t)rxMsg->Data[2] << 8 | rxMsg->Data[3]);
	return speed;
}

/**
 *	@brief	从CAN报文中反馈电机的电流
 */
//static int16_t getMotorCurrent(CanRxMsg *rxMsg)
//{
//	int16_t current;
//	current = ((int16_t)rxMsg->Data[4] << 8 | rxMsg->Data[5]);
//	return current;
//}

/**
 *	@brief	从CAN报文中反馈电机的温度
 */
static uint8_t getMotorTemperature(CanRxMsg *rxMsg)
{
	uint8_t temperature;
	temperature = (rxMsg->Data[6]);
	return temperature;
}

/**
 *	@brief	计算电机机械角度的累加值
 */
static void calMotorAngleSum(int16_t angle_now, int16_t angle_pre, int32_t *angle_sum)
{
	if(abs(angle_now - angle_pre) > 4095) {	// 转过半圈
		if(angle_now < angle_pre) {	// 转过半圈 + 转过零点(正方向)
			*angle_sum += 8191 - angle_pre + angle_now;
		} else {	// 转过半圈 + 转过零点(负方向)
			*angle_sum -= 8191 - angle_now + angle_pre;
		}
	} else {
		*angle_sum += angle_now - angle_pre;
	}
}

/* API functions -------------------------------------------------------------*/
/**
 *	@brief	CAN参数默认配置初始化
 *	@param	(CAN_InitTypeDef*)CAN_InitStructure
 */
void CAN_ParamsInit(CAN_InitTypeDef* CAN_InitStructure)
{
	CAN_InitStructure->CAN_ABOM = CAN_DefaultParams.CAN_ABOM;
	CAN_InitStructure->CAN_AWUM = CAN_DefaultParams.CAN_AWUM;
	CAN_InitStructure->CAN_BS1  = CAN_DefaultParams.CAN_BS1;
	CAN_InitStructure->CAN_BS2  = CAN_DefaultParams.CAN_BS2;
	CAN_InitStructure->CAN_Mode = CAN_DefaultParams.CAN_Mode;
	CAN_InitStructure->CAN_NART = CAN_DefaultParams.CAN_NART;
	CAN_InitStructure->CAN_Prescaler = CAN_DefaultParams.CAN_Prescaler;
	CAN_InitStructure->CAN_RFLM = CAN_DefaultParams.CAN_RFLM;
	CAN_InitStructure->CAN_SJW	= CAN_DefaultParams.CAN_SJW;
	CAN_InitStructure->CAN_TTCM	= CAN_DefaultParams.CAN_TTCM;
	CAN_InitStructure->CAN_TXFP = CAN_DefaultParams.CAN_TXFP;
}

/**
 *	@brief	CAN滤波器参数默认配置初始化
 *	@param	(CAN_FilterInitTypeDef*)CAN_FilterInitStructure
 */
void CAN_Filter_ParamsInit(CAN_FilterInitTypeDef* CAN_FilterInitStructure)
{
	CAN_FilterInitStructure->CAN_FilterActivation = CAN_Filter_DefaultParams.CAN_FilterActivation;
	CAN_FilterInitStructure->CAN_FilterFIFOAssignment = CAN_Filter_DefaultParams.CAN_FilterFIFOAssignment;
	CAN_FilterInitStructure->CAN_FilterIdHigh = CAN_Filter_DefaultParams.CAN_FilterIdHigh;
	CAN_FilterInitStructure->CAN_FilterIdLow = CAN_Filter_DefaultParams.CAN_FilterIdLow;
	CAN_FilterInitStructure->CAN_FilterMaskIdHigh = CAN_Filter_DefaultParams.CAN_FilterMaskIdHigh;
	CAN_FilterInitStructure->CAN_FilterMaskIdLow = CAN_Filter_DefaultParams.CAN_FilterMaskIdLow;
	CAN_FilterInitStructure->CAN_FilterMode = CAN_Filter_DefaultParams.CAN_FilterMode;
	CAN_FilterInitStructure->CAN_FilterNumber = CAN_Filter_DefaultParams.CAN_FilterNumber;
	CAN_FilterInitStructure->CAN_FilterScale = CAN_Filter_DefaultParams.CAN_FilterScale;
}

/**
 *	@brief	CAN1初始化
 *	@note		只配置中断接收
 */
void CAN1_init(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	// 使能CAN1时钟
	
	CAN1_GPIO_init();

	/* CAN NVIC 中断配置 */
	NVICx_init(CAN1_RX0_IRQn, CAN1_RX0_PRIO_PRE, CAN1_RX0_PRIO_SUB);
	
	/* CAN参数配置 */
	CAN_ParamsInit(&CAN_InitStructure);
	CAN_Init(CAN1, &CAN_InitStructure);
	
	/* CAN滤波器参数配置 */
	CAN_Filter_ParamsInit(&CAN_FilterInitStructure);
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* FIFO0消息挂号中断允许 */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
 *	@brief	CAN2初始化
 *	@note		配置中断收发
 */
void CAN2_init(void)
{
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);	// 使能CAN2时钟
	
	CAN2_GPIO_init();

	/* CAN NVIC 中断配置 */
	NVICx_init(CAN2_RX0_IRQn, CAN2_RX0_PRIO_PRE, CAN2_RX0_PRIO_SUB);
	NVICx_init(CAN2_RX0_IRQn, CAN2_TX_PRIO_PRE, CAN2_TX_PRIO_SUB);
	
	/* CAN参数配置 */
	CAN_ParamsInit(&CAN_InitStructure);
	CAN_Init(CAN2, &CAN_InitStructure);
	
	/* CAN滤波器参数配置 */
	CAN_Filter_ParamsInit(&CAN_FilterInitStructure);
	CAN_FilterInitStructure.CAN_FilterNumber = 14;	// 模仿19步兵代码
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* FIFO0消息挂号中断 和 发送邮箱空闲中断 允许 */
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN2, CAN_IT_TME,  ENABLE);
}

/**
 *	@brief	CAN1 发送函数
 *	@param	uint32_t stdID	- 标准标识符
 *					int16_t* dat - 数据缓冲区
 *	@note		默认8个字节的数据
 *	@debug	2019/09/22
 *						手头上的设备ID为0x203
 */
void CAN1_send(uint32_t stdID, float *dat)
{
	CanTxMsg	txMsg;
	
	txMsg.StdId = stdID;		// 使用标准标识符
	txMsg.IDE = CAN_ID_STD;	// 使用标准模式
	txMsg.RTR = CAN_RTR_DATA;	// 0 - 数据帧
	txMsg.DLC = 8;	// 数据长度

	// 先发高8位数据，再发低8位数据
	txMsg.Data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txMsg.Data[1] = (uint8_t)((int16_t)dat[0]);
	txMsg.Data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txMsg.Data[3] = (uint8_t)((int16_t)dat[1]);
	txMsg.Data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txMsg.Data[5] = (uint8_t)((int16_t)dat[2]);
	txMsg.Data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txMsg.Data[7] = (uint8_t)((int16_t)dat[3]);
	
	CAN_Transmit(CAN1, &txMsg);
	//xQueueSend(CAN1_Queue, &txMsg, 1);
}

/**
 *	@brief	CAN2 发送函数
 *	@param	uint32_t stdID	- 标准标识符
 *					int16_t* dat - 数据缓冲区
 *	@note		默认8个字节的数据
 *	@debug	
 */
void CAN2_send(uint32_t stdID, float *dat)
{
	CanTxMsg	txMsg;
	
	txMsg.StdId = stdID;		// 使用标准标识符
	txMsg.IDE = CAN_ID_STD;	// 使用标准模式
	txMsg.RTR = CAN_RTR_DATA;	// 0 - 数据帧
	txMsg.DLC = 8;	// 数据长度

	// 先发高8位数据，再发低8位数据
	txMsg.Data[0] = (uint8_t)((int16_t)dat[0] >> 8);
	txMsg.Data[1] = (uint8_t)((int16_t)dat[0]);
	txMsg.Data[2] = (uint8_t)((int16_t)dat[1] >> 8);
	txMsg.Data[3] = (uint8_t)((int16_t)dat[1]);
	txMsg.Data[4] = (uint8_t)((int16_t)dat[2] >> 8);
	txMsg.Data[5] = (uint8_t)((int16_t)dat[2]);
	txMsg.Data[6] = (uint8_t)((int16_t)dat[3] >> 8);
	txMsg.Data[7] = (uint8_t)((int16_t)dat[3]);
	
	CAN_Transmit(CAN2, &txMsg);
	//xQueueSend(CAN2_Queue, &txMsg, 1);
}

/**
 *	@brief	CAN1 接收中断(底盘与云台)
 *	@note		尽量不要在下面中断打印串口(很占资源) 
 */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg	rxMsg;
	
	if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);	// 清除中断标志位
		CAN_Receive(CAN1, CAN_FIFO0, &rxMsg);
		
		if(rxMsg.StdId == 0x201) {	// 左前
			/* 速度环实时速度记录 */
			g_Chassis_Motor_Info[LEFT_FRON_201].speed = getMotorSpeed(&rxMsg);
			Chassis_PID[LEFT_FRON_201].Speed.feedback = getMotorSpeed(&rxMsg);
			
			if(Flag.chassis.FLAG_pidStart == 0) {	// 开始计算PID
				BitMask.chassis.BM_rxReport |= BM_RX_REPORT_201;
			}			
		}
		
		if(rxMsg.StdId == 0x202) {	// 右前
			/* 速度环实时速度记录 */
			g_Chassis_Motor_Info[RIGH_FRON_202].speed = getMotorSpeed(&rxMsg);
			Chassis_PID[RIGH_FRON_202].Speed.feedback = getMotorSpeed(&rxMsg);
			
			if(Flag.chassis.FLAG_pidStart == 0) {	// 开始计算PID
				BitMask.chassis.BM_rxReport |= BM_RX_REPORT_202;
			}			
		}
		
		if(rxMsg.StdId == 0x203) {	// 左后
			/* 速度环实时速度记录 */
			g_Chassis_Motor_Info[LEFT_BACK_203].speed = getMotorSpeed(&rxMsg);
			Chassis_PID[LEFT_BACK_203].Speed.feedback = getMotorSpeed(&rxMsg);
			
			if(Flag.chassis.FLAG_pidStart == 0) {
				BitMask.chassis.BM_rxReport |= BM_RX_REPORT_203;
			}
		}
		
		if(rxMsg.StdId == 0x204) {	// 右后
			/* 速度环实时速度记录 */
			g_Chassis_Motor_Info[RIGH_BACK_204].speed = getMotorSpeed(&rxMsg);
			Chassis_PID[RIGH_BACK_204].Speed.feedback = getMotorSpeed(&rxMsg);
			
			if(Flag.chassis.FLAG_pidStart == 0) {	// 开始计算PID
				BitMask.chassis.BM_rxReport |= BM_RX_REPORT_204;
			}		
		}
		
		if(rxMsg.StdId == 0x205) {	// Yaw轴云台电机
			/* 电机温度记录 */
			g_Gimbal_Motor_Info[YAW_205].temperature = getMotorTemperature(&rxMsg);
			/* 速度环实时记录 */			
			// ..修改成用IMU的反馈值
			// Gimbal_PID[MECH][YAW_205].Speed.feedback = getMotorSpeed(&rxMsg);
			g_Gimbal_Motor_Info[YAW_205].speed = getMotorSpeed(&rxMsg);
			
			/* 位置环角度记录 */
			g_Gimbal_Motor_Info[YAW_205].angle = getMotorAngle(&rxMsg);			
			Gimbal_PID[MECH][YAW_205].Angle.feedback = getMotorAngle(&rxMsg);			
			Chassis_Z_Speed_PID.feedback = getMotorAngle(&rxMsg);
			
			if(Flag.gimbal.FLAG_pidStart == 0) {
				BitMask.gimbal.BM_rxReport |= BM_RX_REPORT_205;
			}
		}
		
		if(rxMsg.StdId == 0x206) {	// Pitch轴云台电机
			/* 电机温度记录 */
			g_Gimbal_Motor_Info[PITCH_206].temperature = getMotorTemperature(&rxMsg);
			/* 速度环实时记录 */
			// ..修改成用IMU的反馈值
			// Gimbal_PID[MECH][PITCH_206].Speed.feedback = getMotorSpeed(&rxMsg);			
			g_Gimbal_Motor_Info[PITCH_206].speed = getMotorSpeed(&rxMsg);
			
			/* 位置环角度记录 */
			g_Gimbal_Motor_Info[PITCH_206].angle = getMotorAngle(&rxMsg);
			Gimbal_PID[MECH][PITCH_206].Angle.feedback = getMotorAngle(&rxMsg);			
			
			if(Flag.gimbal.FLAG_pidStart == 0) {
				BitMask.gimbal.BM_rxReport |= BM_RX_REPORT_206;
			}
		}
		
		/* PID开始计算判断 */
		if(BitMask.chassis.BM_rxReport == (BM_RX_REPORT_201 | BM_RX_REPORT_202 | BM_RX_REPORT_203 | BM_RX_REPORT_204)) {
			Flag.chassis.FLAG_pidStart = 1;
		}
		if(BitMask.gimbal.BM_rxReport == (BM_RX_REPORT_205 | BM_RX_REPORT_206)) {
			Flag.gimbal.FLAG_pidStart = 1;
		}
	}
}

/**
 *	@brief	CAN2 接收中断(拨弹电机)
 */
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg	rxMsg;
	
	if(CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) {
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);	// 清除中断标志位
		CAN_Receive(CAN2, CAN_FIFO0, &rxMsg);
		
		if(rxMsg.StdId == REVOLVER_ID)	// 0x207
		{
			/* 拨弹电机实时转速反馈记录 */
			g_Revolver_Motor_Info.speed = getMotorSpeed(&rxMsg);
			Revolver_PID.Speed.feedback = getMotorSpeed(&rxMsg);

			/* 拨弹电机累加角度反馈记录 */			
			g_Revolver_Motor_Info.angle = getMotorAngle(&rxMsg);
			calMotorAngleSum(g_Revolver_Motor_Info.angle, g_Revolver_Motor_Info.angle_prev, &g_Revolver_Motor_Info.angle_sum);
			g_Revolver_Motor_Info.angle_prev = g_Revolver_Motor_Info.angle;
			Revolver_PID.Angle.feedback = g_Revolver_Motor_Info.angle_sum;
			
			if(Flag.revolver.FLAG_pidStart == 0) {
				BitMask.revolver.BM_rxReport |= REVOLVER_BM_RX_REPORT;
			}
		}
		
		/* PID开始计算判断 */
		if(BitMask.revolver.BM_rxReport == REVOLVER_BM_RX_REPORT) {
				Flag.revolver.FLAG_pidStart = 1;
		}
	}		
}

/**
 *	@brief	CAN2 发送中断
 */
void CAN2_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2, CAN_IT_TME) != RESET) {
		CAN_ClearITPendingBit(CAN2, CAN_IT_TME);
	}
}
