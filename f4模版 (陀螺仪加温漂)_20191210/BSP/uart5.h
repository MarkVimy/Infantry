#ifndef __UART5_H
#define __UART5_H

/* Includes ------------------------------------------------------------------*/
#include "sys.h"

/* Global macro --------------------------------------------------------------*/
#define    JUDGE_BUFFER_LEN           200

/* Global TypeDef ------------------------------------------------------------*/
/* ## Global Variables Prototypes ## -----------------------------------------*/
extern uint8_t  Judge_Buffer[ JUDGE_BUFFER_LEN ];

/* API functions Prototypes --------------------------------------------------*/
void UART5_init( void );
void UART5_sendChar( uint8_t cData );

#endif
