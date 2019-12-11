#ifndef __LED_H
#define __LED_H

//#include "system.h"
#include "sys.h"

#define LED_GREEN		PCout(10)
#define LED_RED			PCout(11)
#define LED_BLUE		PCout(13)
#define LED_ORANGE	PCout(14)

#define LED_GREEN_ON()		LED_GREEN = 0
#define LED_GREEN_OFF()		LED_GREEN = 1

#define LED_RED_ON()			LED_RED = 0
#define LED_RED_OFF()			LED_RED = 1

#define LED_BLUE_ON()			LED_BLUE = 0
#define LED_BLUE_OFF()		LED_BLUE = 1

#define LED_ORANGE_ON()		LED_ORANGE = 0
#define LED_ORANGE_OFF()	LED_ORANGE = 1

#define LED_ALL_ON()			{LED_GREEN_ON();LED_RED_ON();LED_BLUE_ON();LED_ORANGE_ON();}
#define LED_ALL_OFF()			{LED_GREEN_OFF();LED_RED_OFF();LED_BLUE_OFF();LED_ORANGE_OFF();}

void LED_init(void);

#endif

