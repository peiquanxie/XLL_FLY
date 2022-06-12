#ifndef __ALT_CONTROL_H_
#define __ALT_CONTROL_H_

#include "Headfile.h"

#define ALT_CONTROL_RATE      RATE_20_HZ


#define altHoldThrottleMAX   1650  //定高控制油门输出限幅
#define altHoldThrottleMIN   1490

#define altHoldThrustBaseMAX     1590  //定高基础油门限幅
#define altHoldThrustBaseMIN     1540  

////Z轴速度PID积分限幅（单位cm/s）
//#define PID_VZ_INTEGRATION_LIMIT 			100.0
////Z轴速度PID输出限幅（单位油门值）
//#define PID_VZ_OUTPUT_LIMIT					           //写在state_control.c

//XYZ位置PID输出限幅（单位cm/s）
#define PID_POS_OUTPUT_LIMIT				120.0   



extern	bool Alttitude_control_setup;
extern bool Launch_Init_Complete;
extern float altThrustAdj;
extern float altHoldThrottleBase;	//定高油门基础值  待实验测量
extern bool setupAltitudeHoldFlag;		//标志定高设定完成



void Alt_Control_Init(const state_t *state, setpoint_t *setpoint);
void Alt_Control(const state_t *state,setpoint_t *setpoint);






#endif
