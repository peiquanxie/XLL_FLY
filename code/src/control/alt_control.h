#ifndef __ALT_CONTROL_H_
#define __ALT_CONTROL_H_

#include "Headfile.h"

#define ALT_CONTROL_RATE      RATE_20_HZ


#define altHoldThrottleMAX   1650  //���߿�����������޷�
#define altHoldThrottleMIN   1490

#define altHoldThrustBaseMAX     1590  //���߻��������޷�
#define altHoldThrustBaseMIN     1540  

////Z���ٶ�PID�����޷�����λcm/s��
//#define PID_VZ_INTEGRATION_LIMIT 			100.0
////Z���ٶ�PID����޷�����λ����ֵ��
//#define PID_VZ_OUTPUT_LIMIT					           //д��state_control.c

//XYZλ��PID����޷�����λcm/s��
#define PID_POS_OUTPUT_LIMIT				120.0   



extern	bool Alttitude_control_setup;
extern bool Launch_Init_Complete;
extern float altThrustAdj;
extern float altHoldThrottleBase;	//�������Ż���ֵ  ��ʵ�����
extern bool setupAltitudeHoldFlag;		//��־�����趨���



void Alt_Control_Init(const state_t *state, setpoint_t *setpoint);
void Alt_Control(const state_t *state,setpoint_t *setpoint);






#endif
