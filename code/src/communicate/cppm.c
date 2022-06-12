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

    PPMtime_init();    //��ʼ��PPM��ʱʱ��  
	
    // ����GPIO_D7���½��ز�׽��   
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); 
	
	  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����PD6
		HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//ȷ��
		HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//��������
	
	  GPIOIntRegister(GPIO_PORTD_BASE,PortDIntHandle); //ע���ж�
	  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_7);
	  GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);   // Ϊ�ܽ�����������ģʽ
	  GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_FALLING_EDGE);  //�½��ش���
	  GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_7);    //ʹ���ж�	
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



