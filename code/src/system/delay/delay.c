#include "delay.h"


static volatile uint32_t counter;

static void SycTickHandler(void) {
  counter++;
}

void Sys_Clock_Init(void)
{
	//200/2.5=80MHz,以达到最高频率
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 |SYSCTL_USE_PLL |SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);  
}

void systick_Init(void) 
{
	
  SysTickPeriodSet(SysCtlClockGet() / 1000000UL); // 1000 for milliseconds & 1000000 for microseconds
  SysTickIntRegister(SycTickHandler);
  SysTickIntEnable();
  SysTickEnable();
}

u32 read_tick(void)
{
	return counter;
}

//延时nus
//nus:要延时的us数.	
//nus:0~204522252   								   
void delay_us(u32 nus)
{	
	  ROM_SysCtlDelay(SysCtlClockGet()/3000000*nus);//延时1us
}


//延时nms
//nms:要延时的ms数.	
//nus:0~65535  								   
void delay_ms(u16 nms)
{	
	  ROM_SysCtlDelay(SysCtlClockGet()/3000*nms);//延时1ms
}

