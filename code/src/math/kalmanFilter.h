#ifndef __KALMANFILTER_H
#define __KALMANFILTER_H

#include "Headfile.h"


//卡尔曼滤波 k时刻 数据集合
typedef struct
{
	float Xk;
	float Pk;
	float Xk_1;
	float Pk_1;
	float Kg;
	float Zk;
	
} value_k_t;


//卡尔曼滤波 k-1时刻 数据集合
typedef struct
{
	float Xk_1;
	float Pk_1;
	
} value_k_1_t;

void KalmanFilter(value_k_1_t *value_k_1 ,value_k_t *value_k, float Q , float R);

#endif



