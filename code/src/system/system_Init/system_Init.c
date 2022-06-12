#include "system_init.h"
#include "config.h"
#include "delay.h"
#include "usart.h"

#include "led.h"
#include "key.h"
#include "cppm.h"
#include "usart_task.h"
#include "stabilizer_task.h"
#include "config_param.h"



void sys_Init(void)
{
	IntPriorityGroupingSet(3);   //�ж����ȼ�������3
	Sys_Clock_Init();    //TIVA����Ƶ�ʳ�ʼ����80M
	LED_Init();          //ָʾ�Ƴ�ʼ��
	Key_Init();					 //������ʼ��
	configParamInit();   //��ʼ�����ò���
	ConfigureUART1();    //��λ�����ڳ�ʼ��
	atkpInit();          //����Э���ʼ��
	cppmInit();          //ң����ppm�źŽ��ճ�ʼ��
	stabilizerInit();    //��� PID ��̬�����ʼ��
	
}


