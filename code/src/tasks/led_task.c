#include "led_task.h"
#include "key.h"
#include "open_mv.h"
#include "fsrx_task.h"
 #include "safe_control.h"
 #include "posxy_control.h"
#include "alt_control.h"  
#include "runtime_config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

float CPUusage;
int time_count=0;
char AUTO_FLY = 0;
char Competition_MODE=0;

bool auto_Init=false;

void Init_RCdata(void)
{
	rcData[0]=1500;
	rcData[1]=1500;
	rcData[2]=1000;
	rcData[3]=1500;
}

//	ANGLE_MODE        = (1 << 0),   //角度模式
//	ACRO_MODE         = (1 << 1),  //速率模式
//	HEADFREE_MODE     = (1 << 2),  //无头模式	
//	NAV_ALTHOLD_MODE  = (1 << 3),  //定高模式
//	NAV_RTH_MODE      = (1 << 4),  //定高速率模式
//	NAV_POSHOLD_MODE  = (1 << 5), 	
//	NAV_LAUNCH_MODE   = (1 << 7),  //一键起飞模式
//	FAILSAFE_MODE     = (1 << 8),
//	AUTO_LAND         = (1 << 9),//自动降落

void Init_FlyMode(void)
{
	flightModeFlags = 0;
}

void Init_Flag(void)
{
	Launch_Init_Complete = false;
	posXY_control_setup=false;
	Land_Init = false;
}
	

void led1Task(void* param)
{ 
	u32 tick = 0;
	portTickType lastWakeTime = xTaskGetTickCount();
	
	while(1) 
	{
		
		//1KHz运行频率
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
		CPUusage=OSCPUusage/100.0;
		
		if(KEY3_Scan(0))
			Competition_MODE =!Competition_MODE;
		
		if(Competition_MODE==1)   //比赛模式0--巡线，比赛模式1--定点
			BLED_ON();            
		else 
			BLED_OFF();
		
		if(AUTO_FLY==0 && KEY2_Scan(0))   //按键扫描
			{	
				AUTO_FLY=1;
				auto_Init=false;
			}
		else if(AUTO_FLY && KEY2_Scan(0))
				AUTO_FLY=0;
		
		if (RATE_DO_EXECUTE(50, tick))
		{
			if(AUTO_FLY)
			{
				if(auto_Init==false)
				{
					
					if(time_count<250)  //按下KEY2后，开始计时10S
						time_count++;
					else   //等待时间到，解锁起飞
					{
						time_count=0;
					  Init_RCdata();
						
						mwArm();
						if(Competition_MODE==1)  //悬挂重物定点模式
						{
							ENABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
							ENABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
						}
						else if(Competition_MODE==0)  //巡线模式
						{
							ENABLE_FLIGHT_MODE(NAV_POSHOLD_MODE);
							ENABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
						}
					  auto_Init=true;
					 }
				 }
				
				GLED_ON();  //自动飞行指示灯开启
				
				
			}
			
			else
			{
				time_count=0;
				GLED_OFF();
			}
				
		}

		tick++;

	}

}

