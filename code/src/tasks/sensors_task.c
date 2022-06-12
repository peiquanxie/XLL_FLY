#include "sensors_task.h"
#include "gyro.h"
#include "accelerometer.h"
#include "led.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"


sensorData_t sensors;

static bool isInit = false;

static bool isMPUPresent=false;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;

/*�Ӷ��ж�ȡ��������*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*�Ӷ��ж�ȡ���ټ�����*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}


/* ������������ʼ�� */
void sensorsInit(void)
{
	if (isInit) return;
	

	isMPUPresent = gyroInit(GYRO_UPDATE_RATE);
	isMPUPresent &= accInit(ACC_UPDATE_RATE);
//	
	/*�������������ݶ���*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));

	isInit = true;
}

void sensorsTask(void* param)
{ 
	u32 tick = 0;
	portTickType lastWakeTime = xTaskGetTickCount();
	
	sensorsInit();
	
	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz����Ƶ��

		if (isMPUPresent && RATE_DO_EXECUTE(GYRO_UPDATE_RATE, tick))
		{
			gyroUpdate(&sensors.gyro);
		}
		
		if (isMPUPresent && RATE_DO_EXECUTE(ACC_UPDATE_RATE, tick))
		{
			accUpdate(&sensors.acc);
		}
			
		vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
		xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
		xQueueOverwrite(gyroDataQueue, &sensors.gyro);
		
		xTaskResumeAll();

		tick++;
	}

}

/*��ȡ����������*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)
{
	sensorsReadGyro(&sensors->gyro);  //�����ǵ�����ֵ
	sensorsReadAcc(&sensors->acc);    //���ٶȼƵ�����ֵ
}

