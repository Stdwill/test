#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"


void dianjioutput(int Voltage);
void leftrightoutput(int Turn);
float distance_output(int Angle,int wheelbase);
float turning_time_output(int L,int Turn,int flag);


#endif
