#ifndef __PMW3901_H
#define __PMW3901_H

#include "Headfile.h"
#include "spi_master.h"


typedef struct opFlow_s 
{
	float pixSum[2];		/*累积像素*/
	float pixComp[2];		/*像素补偿*/
	float pixValid[2];		/*有效像素*/
	float pixValidLast[2];	/*上一次有效像素*/
	
	float deltaPos[2];		/*2帧之间的位移 单位cm*/
	float deltaVel[2];		/*速度 单位cm/s*/
	float posSum[2];		/*累积位移 单位cm*/
	float velLpf[2];		/*速度低通 单位cm/s*/
	
	bool isOpFlowOk;		/*光流状态*/
	bool isDataValid;		/*数据有效*/

} opFlow_t;


typedef __packed struct motionBurst_s 
{
	__packed union 
	{
		uint8_t motion;
		__packed struct 
		{
			uint8_t frameFrom0    : 1;
			uint8_t runMode       : 2;
			uint8_t reserved1     : 1;
			uint8_t rawFrom0      : 1;
			uint8_t reserved2     : 2;
			uint8_t motionOccured : 1;
		};
	};

	uint8_t observation;
	int16_t deltaX;
	int16_t deltaY;

	uint8_t squal;

	uint8_t rawDataSum;
	uint8_t maxRawData;
	uint8_t minRawData;

	uint16_t shutter;
} motionBurst_t;




void readMotion(motionBurst_t * motion);
void opticalFlowPowerControl(bool state);	//光流电源控制
void opticalFlowInit(void);		/*初始化光流模块*/

#endif
