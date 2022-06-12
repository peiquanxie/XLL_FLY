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

/************************** * 遥控控制代码************************************/	 

	


#define RC_PERIOD_MS     20

stickPositions_e rcStickPositions = (stickPositions_e) 0; //遥杆位置状态


stickPositions_e getRcStickPositions(void)
{
    return rcStickPositions;
}

//更新遥杆位置
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

//处理遥杆位置和对应命令
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
	
	//加锁
	if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
	{
		mwDisarm();
	}
	//解锁
	if((rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE)&&(!FLIGHT_MODE(NAV_ALTHOLD_MODE)))
	{
		mwArm();
	}
	

}

//处理辅助通道位置和对应命令
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

	//AUX1通道（两段开关) 一键起飞模式
	
		if(channelPos[AUX1] == AUX_LO)
	{
//		if (FLIGHT_MODE(NAV_LAUNCH_MODE))//关闭一键起飞
//			DISABLE_FLIGHT_MODE(NAV_LAUNCH_MODE); 
//			Launch_Init_Complete = false;
	}
	else if(channelPos[AUX1] == AUX_HI)
	{
		if (!FLIGHT_MODE(NAV_LAUNCH_MODE))// 一键起飞
			ENABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
	}
	
	
	
	//AUX3通道（三段开关）对应三个模式（自稳模式、定点模式、手动模式）
	if(channelPos[AUX3] == AUX_LO)//
	{
		
	}
	else if(channelPos[AUX3] == AUX_CE)//自稳模式
	{
		if (!FLIGHT_MODE(ANGLE_MODE))
			ENABLE_FLIGHT_MODE(ANGLE_MODE);
		
		if (FLIGHT_MODE(ACRO_MODE))
			DISABLE_FLIGHT_MODE(ACRO_MODE);
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			DISABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
	}
	else if(channelPos[AUX3] == AUX_HI)//定点模式
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
	
	//AUX4通道（两段开关）对应两个模式（定高模式）
	if(channelPos[AUX4] == AUX_LO)
	{
		if (FLIGHT_MODE(NAV_ALTHOLD_MODE))//非定高模式
		{
//			DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
//			Alttitude_control_setup = false;  //复位定高配置标志位
		}
	}
	else if(channelPos[AUX4] == AUX_HI)
	{
//		if (!FLIGHT_MODE(NAV_ALTHOLD_MODE))//定高模式
//			ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
	}
	
	//AUX2通道（两段开关）激活自动降落
	if(channelPos[AUX2] == AUX_LO)
	{
//		if (FLIGHT_MODE(AUTO_LAND))    //非使能自动降落
//			DISABLE_FLIGHT_MODE(AUTO_LAND);
//	  Land_Init = false;
	}
	else if(channelPos[AUX2] == AUX_HI)
	{
		if (!FLIGHT_MODE(AUTO_LAND))//开启自动降落
		ENABLE_FLIGHT_MODE(AUTO_LAND);
		DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);

	}
}

//获取油门遥杆状态
throttleStatus_e calculateThrottleStatus(void)
{
    if (rcData[THROTTLE] < RC_COMMANDER_MINCHECK)
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}






