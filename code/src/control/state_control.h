#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "Headfile.h"
#include "pid.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ������̬���ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/
extern attitude_t attitudeDesired;
extern attitude_t rateDesired;
extern PidObject pid[PID_NUM];

void stateControlInit(void);
bool stateControlTest(void);
void stateControl(const sensorData_t *sensorData, const state_t *state, setpoint_t *setpoint, control_t *control, const u32 tick);
void stateControlResetYawHolding(void);
void stateControlSetVelocityZPIDIntegration(float integ);
#endif /*__STATE_CONTROL_H */

