#include "usart_task.h"
#include "stabilizer_task.h"
#include "state_control.h"
#include "atkp.h"
#include "imu.h"
#include "us_100.h"
#include "motor.h"
#include "power_control.h"
#include "commander.h"
#include "runtime_config.h"
#include "safe_control.h"
#include "position_task.h"
#include "alt_control.h"
#include "tof_height.h"
#include "gyro.h"
#include "ANO_optical.h"
#include "us100_height.h"
#include "accelerometer.h"
#include "open_mv.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"



//数据返回周期时间（单位ms）
#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR		  40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50

#define ATKP_RX_QUEUE_SIZE 	10 /*ATKP包接收队列消息个数*/

bool isInit = false;
static xQueueHandle rxQueue;

/*数据周期性发送给上位机，每1ms调用一次*/
static void atkpSendPeriod(void)
{
	static u16 count_ms = 1;

	if(!(count_ms % PERIOD_STATUS))
	{
		u8 fm = 0;
		bool armed = ARMING_FLAG(ARMED);  
		if (FLIGHT_MODE(ANGLE_MODE))
			fm = 1;
		if (FLIGHT_MODE(NAV_ALTHOLD_MODE))
			fm = 2;
		if (FLIGHT_MODE(NAV_POSHOLD_MODE))
			fm = 3;
		sendStatus(state.attitude.roll, -state.attitude.pitch, -imuAttitudeYaw, state.position.z, fm, armed);				
	}
	if(!(count_ms % PERIOD_SENSOR))
	{
//		sendSenser(accADC.x, accADC.y, accADC.z, gyroADC.x, gyroADC.y, gyroADC.z, 0, 0, 0);	
//		sendSenser(rcData[AUX4],setpoint.mode.z, control.thrust*10, state.position.z*100, state.velocity.z*100, 
//		setpoint.position.z*100,setpoint.velocity.z*100, altThrustAdj*10 ,altHoldThrottleBase*10);	}

//定点数据发送*/
		sendSenser( state.position.x * 100,state.position.y * 100,WORK_MODE * 1000,state.position.z * 10,
								setpoint.attitude.pitch*1000,state.attitude.pitch*1000,High_TOF*10,state.attitude.roll*100,
								Roll_direction * 100);
		
//		sendSenser( OF_VX*100,state.velocity.x*100,state.position.x*100,state.attitude.roll * 1000,
//								OF_VY*100,state.velocity.y*100,state.position.y*100,state.attitude.pitch* 1000,
//								sensorData.acc.z*100);		
//			
		
///*定高数据发送*/
//	  sendSenser(state.position.z*10,setpoint.position.z*10, state.velocity.z*100,
//		setpoint.velocity.z*100,control.thrust*10,pid[VELOCITY_Z].integ,altHoldThrottleBase*10,
//		pid[VELOCITY_Z].outP*100,pid[VELOCITY_Z].outD*100 );
		
//		sendSenser(state.position.z*10,setpoint.position.z*10, state.velocity.z*100,
//		setpoint.velocity.z*100,height_choosed*10,height_raw*10,
//		Dheight*10,rcData[AUX4]*10, rcData[AUX1]*10);

// 自动降落数据发送
//	  sendSenser(state.position.z*10,High_fusion*10,tof_height*100,
//		setpoint.velocity.z*100,control.thrust*10,LAND_ThrustAdj*100,pid[VELOCITY_LAND].outP*100,
//		pid[VELOCITY_LAND].outI*100,pid[VELOCITY_LAND].outD*100 );

 
 
 
 
/*自稳roll数据发送*/
//		sendSenser( rcData[THROTTLE],attitudeDesired.roll*100, rateDesired.roll*100, 
//		state.attitude.roll*100,sensorData.gyro.x*100,state.position.z*10, pid[ROLL].outP*10,
//		pid[ROLL].outI*100, pid[ROLL].outD*100);			


//		sendSenser( rcData[THROTTLE],attitudeDesired.yaw*10, rateDesired.yaw*10, 
//		sensorData.acc.x*1000,sensorData.acc.y*1000,state.position.z*10, pid[YAW].outP*10,
//		pid[YAW].outI*100, pid[YAW].outD*100);

 
	}
		if(!(count_ms % PERIOD_RCDATA))
	{
		sendRCData(	rcData[THROTTLE], rcData[YAW], rcData[ROLL],
					rcData[PITCH], rcData[AUX1], rcData[AUX2],
					rcData[AUX3], rcData[AUX4], rcData[AUX5], rcData[AUX6]);
	}
//	if(!(count_ms % PERIOD_POWER))
//	{
//		float bat = pmGetBatteryVoltage();
//		sendPower(bat * 100,0);
//	}
	if(!(count_ms % PERIOD_MOTOR))
	{
		sendMotorPWM(motorPWM.m1, motorPWM.m2, motorPWM.m3, motorPWM.m4, 0,0,0,0);
	}
//	if(!(count_ms % PERIOD_SENSOR2))
//	{
//		sendSenser2(baroAltitude * 100, state.position.z);
//	}
	if(++count_ms>=65535) 
		count_ms = 1;	
}



void usartTxTask(void *param)
{
	while(1)
	{
		atkpSendPeriod();
		vTaskDelay(1);
	}
}


void usartRxTask(void *param)
{
	atkp_t p;
	while(1)
	{
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
		taskENTER_CRITICAL();
		atkpReceiveAnl(&p);
		taskEXIT_CRITICAL();
	}
}

void atkpInit(void)
{
	if(isInit) return;
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));
	ASSERT(rxQueue);
	isInit = true;
}

bool atkpReceivePacketBlocking(atkp_t *p)
{
	ASSERT(p);
	ASSERT(p->dataLen <= ATKP_MAX_DATA_SIZE);
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}
