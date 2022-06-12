#include "position_task.h"
#include "stabilizer_task.h"
#include "us100_height.h"
#include "optical_position.h"
#include "pmw3901.h"
#include "usart.h"
#include "gyro.h"
#include "open_mv.h"
#include "us_100.h"
#include "ANO_optical.h"
#include "tof_height.h"
#include "runtime_config.h"
#include "alt_control.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

float altitudeRaw = 0; 
		
void Position_Control_Init(void)
{
	US100_Height_Init();
	Open_MV_Init();
	TOF_Init();
//	ANO_optical_Init();
}


void positionTask(void* param)
{
	u32 tick = 0;
	portTickType lastWakeTime = xTaskGetTickCount();
	
	//opticalFlowInit();   //光流模块初始化
	
	//等待陀螺仪校准完成
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	Position_Control_Init(); 
	
	while(1) 
	{
		//100Hz运行频率
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));	
		
		//获取超声波高度数据（滤波频率100HZ,原始数据读取20HZ，经平滑处理，控制频率可适当提高)
		if (RATE_POSITION_CTR(US100_UPDATE_RATE, tick))
		{
			Position_Height_Update();  //得到高度位置，高度速度
//			ANO_Optical_Update();
		}
		
		//获取光流水平位置数据
//		if (RATE_POSITION_CTR(OPTICAL_UPDATE_RATE, tick))
//		{
////			if(FLIGHT_MODE(NAV_POSHOLD_MODE))
//				Optical_Position_Update(&state, 1.0f/OPTICAL_UPDATE_RATE);  
//		}
		
		//获取Open_MV 水平位置数据(滤波频率100HZ,原始数据读取50HZ，经平滑处理，控制频率可适当提高)
		if (RATE_POSITION_CTR(OPEN_MV_UPDATE_RATE, tick))
		{
//			if(FLIGHT_MODE(NAV_POSHOLD_MODE))
				OpenMV_Position_Update(&state, 1.0f/OPEN_MV_UPDATE_RATE);
		}
		
			
		tick++;
	}
}



