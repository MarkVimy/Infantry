#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//系统时钟初始化	
//包括时钟设置/中断管理/GPIO设置等
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/2
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
//********************************************************************************
//修改说明
//无
//////////////////////////////////////////////////////////////////////////////////  


//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//关闭所有中断(但是不包括fault和NMI中断)
__asm void INTX_DISABLE(void)
{
	CPSID   I
	BX      LR	  
}
//开启所有中断
__asm void INTX_ENABLE(void)
{
	CPSIE   I
	BX      LR  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
	MSR MSP, r0 			//set Main Stack value
	BX r14
}

/**
  * @brief  斜坡函数,使目标输出值缓慢等于期望值
  * @param  期望最终输出,当前输出,变化速度(越大越快)
  * @retval 当前输出
  * @attention  
  */
float RAMP_float(float final, float now, float ramp )
{
	  float buffer = 0;
	
	
	  buffer = final - now;
	
		if (buffer > 0)
		{
				if (buffer > ramp)
				{  
						now += ramp;
				}   
				else
				{
						now += buffer;
				}
		}
		else
		{
				if (buffer < -ramp)
				{
						now += -ramp;
				}
				else
				{
						now += buffer;
				}
		}
		
		return now;
}

void Bubble_sort_float(uint16_t num, float *arr)
{
	uint16_t i, j;
	float tmp = 0;
	bool isSorted = true;
	uint16_t lastExchangeIndex = 0;	// 上一次交换元素所在下标
	uint16_t sortBorder = num - 1;	// 无序数列边界
	for(i = 0; i < num - 1; i++) {
		isSorted = true;
		for(j = 0; j < sortBorder; j++) {
			if(arr[j] > arr[j + 1]) {
				tmp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = tmp;
				isSorted = false;
				lastExchangeIndex = j;
			}
		}
		sortBorder = lastExchangeIndex;
		
		if(isSorted) {	// 一趟比较下来没有交换元素则说明数组已经有序
			break;
		}
	}	
}

/*************中位值平均浮点滤波***************/
float Median_filter_float(float data,int measureNum,float *Filterdata)
{
	unsigned int i = 0;
	unsigned int MAX_error_targe = 0;
	float MAX_error1;
	Filterdata[measureNum-1] = data;
	for(i=0;i<measureNum-1;i++)	//--更新数据序列
	{
	 Filterdata[i]=Filterdata[i+1];
	}
	Bubble_sort_float(measureNum, Filterdata);	// 向下冒泡排序（[0]最小）
	MAX_error1 = abs(Filterdata[1] - Filterdata[0]);	//--计算误差微分初值（斜率）
	for(i = 1 ; i < measureNum-1 ; i++)
	{
			if(MAX_error1 < abs(Filterdata[i+1] - Filterdata[i]) )
			{
					MAX_error1 =  abs(Filterdata[i+1] - Filterdata[i]);
					MAX_error_targe = i; //--最大误差微分值下标
			}
	}
	float Average_data=0;
	if(MAX_error_targe+1 > (measureNum+1)/2)	//--摒弃最大误差所在那半边的数据，然后对剩余数据求平均值
	{
			for(i = 0 ; i <= MAX_error_targe ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (MAX_error_targe+1);
	}
	else
	{
			for(i = MAX_error_targe + 1 ; i < measureNum ; i++)
			{
					Average_data += Filterdata[i];
			}
			Average_data /= (measureNum - MAX_error_targe -1);
	}
	return Average_data;	
}
