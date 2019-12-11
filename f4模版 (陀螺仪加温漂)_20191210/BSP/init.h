#ifndef __INIT_H
#define __INIT_H

/* Includes FREERTOS Header Files ----------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
/* Includes FREERTOS Task Files ------------------------------------- */
#include "my_task.h"
/* Includes SYSTEM Header Files ------------------------------------- */
#include "sys.h"
#include "delay.h"
#include "my_include.h"
//#include "exti.h"
//#include "wdg.h"
//#include "timer.h"
//#include "rtc.h"
/* Includes APP & APPInfo Header Files ------------------------------ */
#include "remote.h"
#include "friction.h"
#include "magzine.h"
#include "crc.h"
/* Includes HARDWARE Header Files ----------------------------------- */
#include "led.h"
#include "laser.h"
#include "can.h"
#include "pwm.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//#include "MPU_Temperature.h"
/* Includes DRIVER Header Files ------------------------------------- */
#include "uart4.h"
#include "uart5.h"
//#include "usmart.h"
//#include "iic.h"
//#include "spi.h"

void All_Init(void);

#endif

