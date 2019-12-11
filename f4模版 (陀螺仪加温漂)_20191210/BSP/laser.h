#ifndef __LASER_H
#define __LASER_H

#include "sys.h"

#define LASER	PDout(9)

#define LASER_ON()	(LASER = 1)
#define LASER_OFF()	(LASER = 0)

void LASER_init(void);

#endif
