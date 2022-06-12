#include "delay.h"


static volatile uint32_t counter;

static void SycTickHandler(void) {
  counter++;
}

void Sys_Clock_Init(void)
{
	//200/2.5=80MHz,�Դﵽ���Ƶ��
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

//��ʱnus
//nus:Ҫ��ʱ��us��.	
//nus:0~204522252   								   
void delay_us(u32 nus)
{	
	  ROM_SysCtlDelay(SysCtlClockGet()/3000000*nus);//��ʱ1us
}


//��ʱnms
//nms:Ҫ��ʱ��ms��.	
//nus:0~65535  								   
void delay_ms(u16 nms)
{	
	  ROM_SysCtlDelay(SysCtlClockGet()/3000*nms);//��ʱ1ms
}

