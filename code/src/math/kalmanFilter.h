#ifndef __KALMANFILTER_H
#define __KALMANFILTER_H

#include "Headfile.h"


//�������˲� kʱ�� ���ݼ���
typedef struct
{
	float Xk;
	float Pk;
	float Xk_1;
	float Pk_1;
	float Kg;
	float Zk;
	
} value_k_t;


//�������˲� k-1ʱ�� ���ݼ���
typedef struct
{
	float Xk_1;
	float Pk_1;
	
} value_k_1_t;

void KalmanFilter(value_k_1_t *value_k_1 ,value_k_t *value_k, float Q , float R);

#endif



