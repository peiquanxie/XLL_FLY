#include "Headfile.h"
#include "led.h"
#include "Time_Cnt.h"


void PPMtime_init(void)//ϵͳ���ȶ�ʱ����ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);//��ʱ��1ʹ��	
  	
  TimerConfigure(WTIMER1_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);//32λ���ڶ�ʱ��
	TimerClockSourceSet(WTIMER1_BASE,TIMER_CLOCK_SYSTEM);
	TimerPrescaleSet(WTIMER1_BASE,TIMER_A,80-1);   //��Ƶϵ��80��ÿ1us��һ��
  	
  TimerEnable(WTIMER1_BASE,TIMER_A); //��ʱ��ʹ�ܿ�ʼ����

}



