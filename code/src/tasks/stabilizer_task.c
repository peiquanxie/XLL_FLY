#include "stabilizer_task.h"
#include "sensors_task.h"
#include "state_control.h"
#include "power_control.h"
#include "pos_estimator.h"
#include "commander.h"
#include "gyro.h"
#include "imu.h"
#include "runtime_config.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"


static bool isInit;     
setpoint_t		setpoint;	/*����Ŀ��״̬*/  
sensorData_t 	sensorData;	/*����������*/  
state_t 		state;		/*������̬*/
control_t 		control;	/*������Ʋ���*/


void stabilizerInit(void)
{
	if(isInit) return;  //�Ѿ���ʼ���򷵻�
	stateControlInit();		/*��̬PID��ʼ��*/
	powerControlInit();		/*�����ʼ��*/
	imuInit();				/*��̬�����ʼ��*/
	isInit = true;
}

void stabilizerTask(void* param)
{
	u32 tick = 0;
	portTickType lastWakeTime = xTaskGetTickCount();
	
	//�ȴ�������У׼���
	while(!gyroIsCalibrationComplete())
	{
		vTaskDelayUntil(&lastWakeTime, M2T(1));
	}
	
	while(1) 
	{
		//1KHz����Ƶ��
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));	
		
		//��ȡ����������
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			sensorsAcquire(&sensorData, tick);
		}
		
		//��Ԫ����ŷ���Ǽ���
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdateAttitude(&sensorData, &state, ATTITUDE_ESTIMAT_DT);				
		}	
		
		//λ��Ԥ������
		if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick))
		{  	
//			updatePositionEstimator(&sensorData, &state, POSITION_ESTIMAT_DT);
			
		}
		
		//Ŀ����̬�ͷ���ģʽ�趨	
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			commanderGetSetpoint(&state, &setpoint);
			updateArmingStatus();
		}
		
		//PID���������������� 500HZ
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			stateControl(&sensorData, &state, &setpoint, &control, tick);
		}
		
		//���Ƶ�������500Hz��
		if (RATE_DO_EXECUTE(MAIN_LOOP_RATE, tick))
		{
			powerControl(&control);
		}
		
		tick++;
	}
}

