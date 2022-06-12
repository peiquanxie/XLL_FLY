#ifndef __COMMANDER_H
#define __COMMANDER_H

#include "Headfile.h"
#include "atkp.h"
#include "config.h"
#include "fsrx_task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ATKflight�ɿع̼�
 * ��ȡң��������������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct
{
	bool autoLandActive;
	uint32_t autoLandTime;
}autoLandState_t;

extern int16_t rcCommand[4];
extern autoLandState_t autoLandState;
extern u16 Hover_count;   //��ͣʱ�����
extern bool Turn_left_Flag;

extern u8 Roll_direction ;		//Ĭ�ϲ����
extern u8 Start_command; 		//��������

void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint);
void commanderSetupAltitudeHoldMode(void);
uint16_t commanderGetALtHoldThrottle(void);
void commanderActiveFailsafe(void);
void resetFailsafe(void);







#endif /* __COMMANDER_H */
