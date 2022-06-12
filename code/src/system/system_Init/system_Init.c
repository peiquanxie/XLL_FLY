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
	IntPriorityGroupingSet(3);   //中断优先级规则组3
	Sys_Clock_Init();    //TIVA工作频率初始化，80M
	LED_Init();          //指示灯初始化
	Key_Init();					 //按键初始化
	configParamInit();   //初始化配置参数
	ConfigureUART1();    //上位机串口初始化
	atkpInit();          //传输协议初始化
	cppmInit();          //遥控器ppm信号接收初始化
	stabilizerInit();    //电机 PID 姿态解算初始化
	
}


