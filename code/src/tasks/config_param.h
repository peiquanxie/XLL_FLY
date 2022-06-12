#ifndef __CONFIG_PARAM_H
#define __CONFIG_PARAM_H

#include "Headfile.h"
	

enum pidIndex
{
	RATE_ROLL = 0,
	RATE_PITCH,
	RATE_YAW,
	ANGLE_ROLL,
	ANGLE_PITCH,
	ANGLE_YAW,
	VELOCITY_Z,
	POSHOLD_Z,
	VELOCITY_XY,
	POSHOLD_XY,	
	VELOCITY_LAND,
	PID_NUM
};


typedef struct 
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	int16_t accZero[3];
	int16_t accGain[3];
} accBias_t;

typedef struct
{
	int16_t magZero[3];
} magBias_t;

typedef struct 
{
    int16_t rollDeciDegrees;
    int16_t pitchDeciDegrees;
    int16_t yawDeciDegrees;
} boardAlignment_t;

typedef struct	
{
	u8 version;						/*����汾��*/
	pidInit_t pid[PID_NUM];			/*PID����*/
	accBias_t accBias;				/*���ٶ�У׼ֵ*/
	magBias_t magBias;				/*������У׼ֵ*/
	boardAlignment_t boardAlign;	/*����΢��*/
	u16 thrustBase;					/*���Ż���ֵ*/
	u8 cksum;						/*У��*/
} configParam_t;


extern configParam_t configParam;

void configParamInit(void);	/*�������ó�ʼ��*/
void configParamTask(void* param);	/*������������*/
//bool configParamTest(void);

void configParamGiveSemaphore(void);
void resetConfigParam(void);
void saveConfigAndNotify(void);

#endif /*__CONFIG_PARAM_H */

