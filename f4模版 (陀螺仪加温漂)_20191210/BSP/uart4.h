#ifndef __UART4_H
#define __UART4_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/

/* Global TypeDef ------------------------------------------------------------*/
/* ## Global Variables Prototypes ## -----------------------------------------*/

/* API functions Prototypes --------------------------------------------------*/
void UART4_init( void );
void UART4_sendChar( uint8_t cData );

#endif
