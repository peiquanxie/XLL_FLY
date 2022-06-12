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
uint16_t rcData[CH_NUM];//����PPM��ͨ���ź�ֵ
rcLinkState_t rcLinkState;
failsafeState_t  failsafeState;


void ppmRxTask(void *param)
{
	uint16_t ppm;
	uint32_t currentTick;
	while(1)
	{
		currentTick = xTaskGetTickCount();
		
		if(cppmGetTimestamp(&ppm) == pdTRUE)//20ms����ʽ��ȡPPM����ֵ
		{
			if ( ppm < 2100)//�ж�PPM֡����
			{
				if(currChannel < CH_NUM)
				{
					rcData[currChannel] = ppm;
				}
				currChannel++;
			}
			else//������һ֡����
			{
				currChannel = 0;
				rcLinkState.linkState = true;
				if (rcData[THROTTLE] < 950 || rcData[THROTTLE] > 2100)//��Ч���壬˵�����ջ������ʧ�ر�����ֵ
					rcLinkState.invalidPulse = true;
				else
					rcLinkState.invalidPulse = false;
				rcLinkState.realLinkTime = currentTick;
			}
		}
		
		if (currentTick - rcLinkState.realLinkTime > 1000)//1Sû���յ��ź�˵��ң��������ʧ��
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
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz����Ƶ��
		
		if (RATE_DO_EXECUTE(RATE_50_HZ, tick))
		{
			currentTick = xTaskGetTickCount();
			
			//����ң������͸���ͨ��ģʽ�л�
			if (rcLinkState.linkState == true)
			{
				processRcStickPositions();
				processRcAUXPositions();
			}
			
			//�������״̬��ң��ʧȥ����
			if (ARMING_FLAG(ARMED))
			{
				if (rcLinkState.linkState == false || rcLinkState.invalidPulse == true)//ң��ʧȥ���ӻ���Ч����
				{
					if (failsafeState.failsafeActive == false)
					{
						if (currentTick > failsafeState.throttleLowPeriod )//��һ�ε�����ʱ�䣨��5�룩��˵���ɻ��ڵ��Ͽ�ֱ�ӹرյ��
						{
							//mwDisarm();
						}
						else 
						{
							failsafeState.failsafeActive = true;
							commanderActiveFailsafe();//����ʧ�ر����Զ�����
						}
					}
				}
				else//ң����������
				{
					throttleStatus_e throttleStatus = calculateThrottleStatus();
					if (throttleStatus == THROTTLE_HIGH)
					{
						failsafeState.throttleLowPeriod = currentTick + 5000;//5000��ʾ��Ҫ�����ŵ�ʱ�䣨5�룩
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



