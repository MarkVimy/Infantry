#include "init.h"
#include "scheduler.h"
 
/**
 *	-2019/11/17
 *	@note
 *	1. 摄像头的位置固定需要稳定，用大弹暴力测试
 *	2. 云台归中的意义是为了偏移的时候能够回来，保证能够看到大符
 *	3. 陀螺仪数据反馈会漂移，可以试一下换主控板
 *	4. 注意关大符，关电，关遥控等等细节问题
 *	5. 中二、热血，深大总冠军！
 *	6. 组长监督组员，分配好与各个组的对接时间
 *	7. 虚位的测试方法：激光打向很远的一堵墙画十字，机械模式，掰头看一下回来之后反馈机械角度是否一致
 *	8. 调大符的流程： 1、先确保弹道的稳定。
 *					 2、画圆补偿（分析两边的问题）
 *					 3、跟踪控制
 *  9. 敢想、实现。做到别人认为做不到的事情，做到极致
 *	10. 考虑实际的上场状态。将测试环境尽量做到与实战环境一致 
 *
 *	
 *	@task
 *	# 下周日晚8点
 *	1. 大符100%
 *	2. 交接小陀螺(实现基本的控制)
 *	3. 电控预测总结、框架
 *	4. 规则疯狂阅读
 *	# 1119日晚老师过来检查基本功能
 *	
 *	-2019/11/18
 *	@note
 *	小符掉帧处理思路；
 *	陀螺仪yaw反馈采用int型，反向积分清零
 *
 *	-2019/11/19
 *	@note
 *	
 *
 *	-2019/11/21
 *	@task
 *	①调自瞄云台
 *	②思考验证新模型
 *	@note
 *	
 */

/**
 *	-2019/11/23
 *	@task
 *	①机械模式pitch反馈角度会有阶跃到0的噪声(可能是jscope接受不了这么快的频率)
 *	②思考验证新模型
 *	@note
 *	
 */
 
/**
 *	-2019/11/24
 *	1. 早上调小符跟随
 *	2. 下午调小符预测
 *	3. 晚上跳小符打弹
 */
 
/**
 *	-2019/11/26
 *	1. 看自动打符代码，发现需要修改一下拨盘任务的框架
 */
 
/**
 *	-2019/11/27
 *	1. 完善拨盘任务框架
 *	2. 思考打符策略
 *	3. 测试视觉那边的反馈效果
 *	4. 打符抬头补偿
 */
 
/**
 *	-2019/11/28
 *	1. 看卡尔曼预测
 */ 
 
/**
 *	-2019/12/07
 *	1. 203左后轮在疯狂摇动摇杆之后，有时会出现摇杆归中之后轮子还在全速转动
 *	-2019/12/08
 *	
 */
 
/**
 *	# 深大总冠军！ #
 *	# 步兵Infantry #
 */

int main(void)
{
	All_Init();
	while(1) {
	}
}
