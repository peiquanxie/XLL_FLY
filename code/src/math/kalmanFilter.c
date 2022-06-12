#include "KalmanFilter.h"


void KalmanFilter(value_k_1_t *value_k_1 ,value_k_t *value_k, float Q, float R)
{
	value_k_1->Xk_1 = value_k->Xk;
	value_k_1->Pk_1 = value_k->Pk;    //�ݹ�
	value_k->Xk_1 = value_k_1->Xk_1;  //�������ֵ
	value_k->Pk_1 = value_k_1->Pk_1+Q;  //�������ֵ��Э����
	value_k->Kg = value_k->Pk_1 / ( value_k->Pk_1 + R);  //������ϵ��
	value_k->Xk = value_k->Xk_1 + value_k->Kg * (value_k->Zk - value_k->Xk_1); //�������ֵ
	value_k->Pk = (1-value_k->Kg) * value_k->Pk_1;  //�������ֵ��Э����

}


		  

