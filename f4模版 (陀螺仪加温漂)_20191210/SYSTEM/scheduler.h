#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "vision.h"
#include "init.h"

//typedef struct
//{
//	uint8_t  check_flag;  // 周期任务检测标志 在循环开始前置1 结束时置0 判断其值可以检测循环是否正常执行
//	uint16_t err_flag; 		// 错误标记
//	uint16_t cnt_1ms;
//	uint16_t cnt_2ms;
//	uint16_t cnt_5ms;
//	uint16_t cnt_10ms;
//	uint16_t cnt_20ms;
//	uint16_t cnt_50ms;
//	uint16_t cnt_100ms;
//	uint16_t cnt_120ms;
//	uint16_t cnt_500ms;
//	uint32_t time;
//}loop_t;
//	
//void Duty_1ms(void);
//void Duty_2ms(void);
//void Duty_5ms(void);
//void Duty_10ms(void);
//void Duty_20ms(void);
//void Duty_50ms(void);
//void Duty_100ms(void);
//void Duty_120ms(void);
//void Duty_500ms(void);

//void Loop_err_clear(void);
//void Duty_Loop(void);
//void Loop_check(void);

void start_task(void *pvParameters);	// 任务函数

#endif


