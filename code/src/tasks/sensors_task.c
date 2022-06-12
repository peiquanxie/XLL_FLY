#include "sensors_task.h"
#include "gyro.h"
#include "accelerometer.h"
#include "led.h"

/*FreeRTOS相关头文件*/
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

/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}


/* 传感器器件初始化 */
void sensorsInit(void)
{
	if (isInit) return;
	

	isMPUPresent = gyroInit(GYRO_UPDATE_RATE);
	isMPUPresent &= accInit(ACC_UPDATE_RATE);
//	
	/*创建传感器数据队列*/
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
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_1000_HZ));//1KHz运行频率

		if (isMPUPresent && RATE_DO_EXECUTE(GYRO_UPDATE_RATE, tick))
		{
			gyroUpdate(&sensors.gyro);
		}
		
		if (isMPUPresent && RATE_DO_EXECUTE(ACC_UPDATE_RATE, tick))
		{
			accUpdate(&sensors.acc);
		}
			
		vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
		xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
		xQueueOverwrite(gyroDataQueue, &sensors.gyro);
		
		xTaskResumeAll();

		tick++;
	}

}

/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)
{
	sensorsReadGyro(&sensors->gyro);  //陀螺仪的三个值
	sensorsReadAcc(&sensors->acc);    //加速度计的三个值
}

