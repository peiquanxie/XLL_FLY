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

/*FreeRTOS���ͷ�ļ�*/
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
	
	//opticalFlowInit();   //����ģ���ʼ��
	
	//�ȴ�������У׼���
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	Position_Control_Init(); 
	
	while(1) 
	{
		//100Hz����Ƶ��
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));	
		
		//��ȡ�������߶����ݣ��˲�Ƶ��100HZ,ԭʼ���ݶ�ȡ20HZ����ƽ����������Ƶ�ʿ��ʵ����)
		if (RATE_POSITION_CTR(US100_UPDATE_RATE, tick))
		{
			Position_Height_Update();  //�õ��߶�λ�ã��߶��ٶ�
//			ANO_Optical_Update();
		}
		
		//��ȡ����ˮƽλ������
//		if (RATE_POSITION_CTR(OPTICAL_UPDATE_RATE, tick))
//		{
////			if(FLIGHT_MODE(NAV_POSHOLD_MODE))
//				Optical_Position_Update(&state, 1.0f/OPTICAL_UPDATE_RATE);  
//		}
		
		//��ȡOpen_MV ˮƽλ������(�˲�Ƶ��100HZ,ԭʼ���ݶ�ȡ50HZ����ƽ����������Ƶ�ʿ��ʵ����)
		if (RATE_POSITION_CTR(OPEN_MV_UPDATE_RATE, tick))
		{
//			if(FLIGHT_MODE(NAV_POSHOLD_MODE))
				OpenMV_Position_Update(&state, 1.0f/OPEN_MV_UPDATE_RATE);
		}
		
			
		tick++;
	}
}



