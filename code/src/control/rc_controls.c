#include "rc_controls.h"
#include "fsrx_task.h"
#include "sensorsalignment.h"
//#include "beeper.h"
#include "commander.h"
#include "runtime_config.h"
#include "posxy_control.h"
#include "alt_control.h"
#include "launch_control.h"
#include "safe_control.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/************************** * ң�ؿ��ƴ���************************************/	 

	


#define RC_PERIOD_MS     20

stickPositions_e rcStickPositions = (stickPositions_e) 0; //ң��λ��״̬


stickPositions_e getRcStickPositions(void)
{
    return rcStickPositions;
}

//����ң��λ��
static void updateRcStickPositions(void)
{
	stickPositions_e tmp = (stickPositions_e)0;
	
	tmp |= ((rcData[ROLL] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (ROLL * 2);
	tmp |= ((rcData[ROLL] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (ROLL * 2);

	tmp |= ((rcData[PITCH] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (PITCH * 2);
	tmp |= ((rcData[PITCH] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (PITCH * 2);

	tmp |= ((rcData[THROTTLE] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (THROTTLE * 2);
	tmp |= ((rcData[THROTTLE] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (THROTTLE * 2);

	tmp |= ((rcData[YAW] > RC_COMMANDER_MINCHECK) ? 0x02 : 0x00) << (YAW * 2);
	tmp |= ((rcData[YAW] < RC_COMMANDER_MAXCHECK) ? 0x01 : 0x00) << (YAW * 2);
	
    rcStickPositions = tmp;
}

//����ң��λ�úͶ�Ӧ����
void processRcStickPositions(void)
{
	static u32 lastTickTimeMs;
	static uint8_t rcDelayCommand;
	static uint32_t rcSticks;
	portTickType currentTimeMs = xTaskGetTickCount();
	
  updateRcStickPositions();
	uint32_t stTmp = getRcStickPositions();
	if(stTmp == rcSticks)
	{
		if(rcDelayCommand<250 && (currentTimeMs - lastTickTimeMs) >= RC_PERIOD_MS)
		{	
			lastTickTimeMs = currentTimeMs;
			rcDelayCommand++;
		}
	}
	else
		rcDelayCommand = 0;
	
	rcSticks = stTmp;
	
	if (rcDelayCommand != 20) return;
	
	//����
	if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
	{
		mwDisarm();
	}
	//����
	if((rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)&&(!FLIGHT_MODE(NAV_ALTHOLD_MODE)))
	{
		mwArm();
	}
	

}

//������ͨ��λ�úͶ�Ӧ����
void processRcAUXPositions(void)
{
	auxPositions_e channelPos[CH_NUM];
	
	for (int i=AUX1; i<CH_NUM; i++)
	{
		if (rcData[i] < (RC_MID-200))
			channelPos[i] = AUX_LO;
		else if (rcData[i] > (RC_MID+200))
			channelPos[i] = AUX_HI;
		else
			channelPos[i] = AUX_CE;
	}

	//AUX1ͨ�������ο���) һ�����ģʽ
	
		if(channelPos[AUX1] == AUX_LO)
	{
//		if (FLIGHT_MODE(NAV_LAUNCH_MODE))//�ر�һ�����
//			DISABLE_FLIGHT_MODE(NAV_LAUNCH_MODE); 
//			Launch_Init_Complete = false;
	}
	else if(channelPos[AUX1] == AUX_HI)
	{
		if (!FLIGHT_MODE(NAV_LAUNCH_MODE))// һ�����
			ENABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
	}
	
	
	
	//AUX3ͨ�������ο��أ���Ӧ����ģʽ������ģʽ������ģʽ���ֶ�ģʽ��
	if(channelPos[AUX3] == AUX_LO)//
	{
		
	}
	else if(channelPos[AUX3] == AUX_CE)//����ģʽ
	{
		if (!FLIGHT_MODE(ANGLE_MODE))
			ENABLE_FLIGHT_MODE(ANGLE_MODE);
		
		if (FLIGHT_MODE(ACRO_MODE))
			DISABLE_FLIGHT_MODE(ACRO_MODE);
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	else if(channelPos[AUX3] == AUX_HI)//����ģʽ
	{
		if (!FLIGHT_MODE(NAV_POSHOLD_MODE))
		{
			ENABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
		  posXY_control_setup=false;
		}
		
		if (FLIGHT_MODE(ANGLE_MODE))
			DISABLE_FLIGHT_MODE(ANGLE_MODE);
		if (FLIGHT_MODE(ACRO_MODE))
			DISABLE_FLIGHT_MODE(ACRO_MODE);
	}
	
	//AUX4ͨ�������ο��أ���Ӧ����ģʽ������ģʽ��
	if(channelPos[AUX4] == AUX_LO)
	{
		if (FLIGHT_MODE(NAV_ALTHOLD_MODE))//�Ƕ���ģʽ
		{
//			DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
//			Alttitude_control_setup = false;  //��λ�������ñ�־λ
		}
	}
	else if(channelPos[AUX4] == AUX_HI)
	{
//		if (!FLIGHT_MODE(NAV_ALTHOLD_MODE))//����ģʽ
//			ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
	}
	
	//AUX2ͨ�������ο��أ������Զ�����
	if(channelPos[AUX2] == AUX_LO)
	{
//		if (FLIGHT_MODE(AUTO_LAND))    //��ʹ���Զ�����
//			DISABLE_FLIGHT_MODE(AUTO_LAND);
//	  Land_Init = false;
	}
	else if(channelPos[AUX2] == AUX_HI)
	{
		if (!FLIGHT_MODE(AUTO_LAND))//�����Զ�����
		ENABLE_FLIGHT_MODE(AUTO_LAND);
		DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);

	}
}

//��ȡ����ң��״̬
throttleStatus_e calculateThrottleStatus(void)
{
    if (rcData[THROTTLE] < RC_COMMANDER_MINCHECK)
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}






