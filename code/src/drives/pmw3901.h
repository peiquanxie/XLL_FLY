#ifndef __PMW3901_H
#define __PMW3901_H

#include "Headfile.h"
#include "spi_master.h"


typedef struct opFlow_s 
{
	float pixSum[2];		/*�ۻ�����*/
	float pixComp[2];		/*���ز���*/
	float pixValid[2];		/*��Ч����*/
	float pixValidLast[2];	/*��һ����Ч����*/
	
	float deltaPos[2];		/*2֮֡���λ�� ��λcm*/
	float deltaVel[2];		/*�ٶ� ��λcm/s*/
	float posSum[2];		/*�ۻ�λ�� ��λcm*/
	float velLpf[2];		/*�ٶȵ�ͨ ��λcm/s*/
	
	bool isOpFlowOk;		/*����״̬*/
	bool isDataValid;		/*������Ч*/

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
void opticalFlowPowerControl(bool state);	//������Դ����
void opticalFlowInit(void);		/*��ʼ������ģ��*/

#endif
