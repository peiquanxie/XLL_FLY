#ifndef _US_100_H_
#define _US_100_H_

#include "Headfile.h"
#include "KalmanFilter.h"

#define TIMES  20
#define User_Height_Max  150//用户使用的最大高度值，单位为cm，需要流出一定裕度给气压计/超声波切换过渡

#define US_Distance_CMD    0x55
#define US_Temperature_CMD 0x50
#define US100_UPDATE_RATE			RATE_100_HZ

#define ABS(X)  (((X)>0)?(X):-(X))


float US_DistanceRead(void);
bool US100_Init(void);
float US100_Update(void);

extern float height_raw,height_raw1,Dheight;
extern float height_choosed;  //异常数据处理后的超声波原始数据
extern float height_test,height_test1;
extern bool Data_Select; //1: height_choosed  0: height_test

extern uint8_t US_Update_Flag;
extern float distance_up;

#endif


