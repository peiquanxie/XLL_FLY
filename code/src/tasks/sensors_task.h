#ifndef __SENSORS_TASK_H__
#define __SENSORS_TASK_H__

#include "Headfile.h"

/*��������ȡ����Ƶ��*/
#define GYRO_UPDATE_RATE		RATE_500_HZ
#define ACC_UPDATE_RATE			RATE_500_HZ

void sensorsTask(void* param);
void sensorsAcquire(sensorData_t *sensors, const u32 tick);

#endif


