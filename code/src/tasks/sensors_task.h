#ifndef __SENSORS_TASK_H__
#define __SENSORS_TASK_H__

#include "Headfile.h"

/*传感器读取更新频率*/
#define GYRO_UPDATE_RATE		RATE_500_HZ
#define ACC_UPDATE_RATE			RATE_500_HZ

void sensorsTask(void* param);
void sensorsAcquire(sensorData_t *sensors, const u32 tick);

#endif


