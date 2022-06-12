#ifndef _LAUNCH_CONTROL_H_
#define _LAUNCH_CONTROL_H_

#include "Headfile.h"

#define Launch_height  100 //一键起飞定高
#define Launch_Velocity   27

#define LaunchThrottleBase  1585 //一键起飞基础油门

extern bool Launch_Init_Complete ;

void Launch_Init(setpoint_t *setpoint);



#endif
