#include "fsrx_task.h"
#include "config.h"
#include "commander.h"
#include "cppm.h"
#include "rc_controls.h"
#include "runtime_config.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


static uint8_t currChannel = 0;
uint16_t rcData[CH_NUM];//捕获PPM的通道信号值
rcLinkState_t rcLinkState;
failsafeState_t  failsafeState;


void ppmRxTask(void *param)
{
	uint16_t ppm;
	uint32_t currentTick;
	while(1)
	{
		currentTick = xTaskGetTickCount();
		
		if(cppmGetTimestamp(&ppm) == pdTRUE)//20ms阻塞式获取PPM脉冲值
		{
			if ( ppm < 2100)//判断PPM帧结束
			{
				if(currChannel < CH_NUM)
				{
					rcData[currChannel] = ppm;
				}
				currChannel++;
			}
			else//接收完一帧数据
			{
				currChannel = 0;
				rcLinkState.linkState = true;
				if (rcData[THROTTLE] < 950 || rcData[THROTTLE] > 2100)//无效脉冲，说明接收机输出了失控保护的值
					rcLinkState.invalidPulse = true;
				else
					rcLinkState.invalidPulse = false;
				rcLinkState.realLinkTime = currentTick;
			}
		}
		
		if (currentTick - rcLinkState.realLinkTime > 1000)//1S没接收到信号说明遥控器连接失败
		{
			rcLinkState.linkState = false;
		}	
	}
}

void fsRxTask(void *param)
{	
	u32 tick = 0;
	portTickType lastWakeTime = xTaskGetTickCount();
	uint32_t currentTick;
	
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz运行频率
		
		if (RATE_DO_EXECUTE(RATE_50_HZ, tick))
		{
			currentTick = xTaskGetTickCount();
			
			//处理遥杆命令和辅助通道模式切换
			if (rcLinkState.linkState == true)
			{
				processRcStickPositions();
				processRcAUXPositions();
			}
			
			//处理解锁状态下遥控失去连接
			if (ARMING_FLAG(ARMED))
			{
				if (rcLinkState.linkState == false || rcLinkState.invalidPulse == true)//遥控失去连接或无效脉冲
				{
					if (failsafeState.failsafeActive == false)
					{
						if (currentTick > failsafeState.throttleLowPeriod )//有一段低油门时间（如5秒），说明飞机在地上可直接关闭电机
						{
							//mwDisarm();
						}
						else 
						{
							failsafeState.failsafeActive = true;
							commanderActiveFailsafe();//激活失控保护自动降落
						}
					}
				}
				else//遥控连接正常
				{
					throttleStatus_e throttleStatus = calculateThrottleStatus();
					if (throttleStatus == THROTTLE_HIGH)
					{
						failsafeState.throttleLowPeriod = currentTick + 5000;//5000表示需要低油门的时间（5秒）
					}
				}
			}
			else
			{
				failsafeState.throttleLowPeriod = 0;
			}
			
		}
		tick++;
	}
}



