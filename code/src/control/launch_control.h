#ifndef _LAUNCH_CONTROL_H_
#define _LAUNCH_CONTROL_H_

#include "Headfile.h"

#define Launch_height  100 //һ����ɶ���
#define Launch_Velocity   27

#define LaunchThrottleBase  1585 //һ����ɻ�������

extern bool Launch_Init_Complete ;

void Launch_Init(setpoint_t *setpoint);



#endif
