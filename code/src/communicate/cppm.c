#include "Headfile.h"
#include "cppm.h"
#include "time_cnt.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static xQueueHandle captureQueue;
static u32 prevCapureVal;


void PortDIntHandle(void);


void cppmInit(void)
{   

    PPMtime_init();    //初始化PPM计时时钟  
	
    // 配置GPIO_D7作下降沿捕捉脚   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
	
	  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//解锁PD6
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//确认
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//重新锁定
	
	  GPIOIntRegister(GPIO_PORTD_BASE,PortDIntHandle); //注册中断
	  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_7);
	  GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);   // 为管脚配置弱上拉模式
	  GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);  //下降沿触发
	  GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_7);    //使能中断	
	  IntEnable(INT_GPIOD); 
    IntPrioritySet(INT_GPIOD, USER_INT3);
	  IntMasterEnable();
	  
	  captureQueue = xQueueCreate(64, sizeof(uint16_t));

}



int cppmGetTimestamp(uint16_t *timestamp)
{
	ASSERT(timestamp);

	return xQueueReceive(captureQueue, timestamp, 20);
}

void cppmClearQueue(void)
{
	xQueueReset(captureQueue);
}


u32 capureVal;
u16 capureValDiff;

void PortDIntHandle(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

		capureVal = TimerValueGet(WTIMER1_BASE,TIMER_A);
		capureValDiff = prevCapureVal-capureVal;
		prevCapureVal = capureVal;

		xQueueSendFromISR(captureQueue, &capureValDiff, &xHigherPriorityTaskWoken);
 
	  GPIOIntClear(GPIO_PORTD_BASE, GPIO_PIN_7);


}



