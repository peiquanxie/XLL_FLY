#include "Headfile.h"
#include "led.h"
#include "Time_Cnt.h"


void PPMtime_init(void)//系统调度定时器初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER1);//定时器1使能	
  	
  TimerConfigure(WTIMER1_BASE,TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);//32位周期定时器
	TimerClockSourceSet(WTIMER1_BASE,TIMER_CLOCK_SYSTEM);
	TimerPrescaleSet(WTIMER1_BASE,TIMER_A,80-1);   //分频系数80，每1us计一次
  	
  TimerEnable(WTIMER1_BASE,TIMER_A); //定时器使能开始计数

}



