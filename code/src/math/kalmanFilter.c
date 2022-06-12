#include "KalmanFilter.h"


void KalmanFilter(value_k_1_t *value_k_1 ,value_k_t *value_k, float Q, float R)
{
	value_k_1->Xk_1 = value_k->Xk;
	value_k_1->Pk_1 = value_k->Pk;    //递归
	value_k->Xk_1 = value_k_1->Xk_1;  //先验估计值
	value_k->Pk_1 = value_k_1->Pk_1+Q;  //先验估计值的协方差
	value_k->Kg = value_k->Pk_1 / ( value_k->Pk_1 + R);  //卡尔曼系数
	value_k->Xk = value_k->Xk_1 + value_k->Kg * (value_k->Zk - value_k->Xk_1); //后验估计值
	value_k->Pk = (1-value_k->Kg) * value_k->Pk_1;  //后验估计值的协方差

}


		  

