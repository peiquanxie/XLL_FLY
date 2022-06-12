#include "led.h"


/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
static uint32_t warningLedTimer = 0;

typedef enum
{
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_LED_FLASH
} warningLedState_e;

static warningLedState_e warningLedState = WARNING_LED_OFF;




void RLED_Change()
{
	GPIOPinWrite(LED_BASE,R_LED,~GPIOPinRead(LED_BASE,R_LED));
}

void GLED_Change()
{
	GPIOPinWrite(LED_BASE,G_LED,~GPIOPinRead(LED_BASE,G_LED));
}

void BLED_Change()
{
	GPIOPinWrite(LED_BASE,B_LED,~GPIOPinRead(LED_BASE,B_LED));
}


void RLED_ON()
{
	GPIOPinWrite(LED_BASE,R_LED,0);
}
void RLED_OFF()
{
	GPIOPinWrite(LED_BASE,R_LED,R_LED);
	
}
void GLED_ON()
{
	GPIOPinWrite(LED_BASE,G_LED,0);
}
void GLED_OFF()
{
	GPIOPinWrite(LED_BASE,G_LED,G_LED);
}
void BLED_ON()
{
	GPIOPinWrite(LED_BASE,B_LED,0);
}
void BLED_OFF()
{
	GPIOPinWrite(LED_BASE,B_LED,B_LED);
}


void LED_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_LED);//初始化外设GPIO
  GPIOPinTypeGPIOOutput(LED_BASE,R_LED|G_LED|B_LED);
	GPIOPinWrite(LED_BASE,R_LED|G_LED|B_LED,R_LED|G_LED|B_LED);

}

void warningLedON(void)
{
    warningLedState = WARNING_LED_ON;
}

void warningLedOFF(void)
{
    warningLedState = WARNING_LED_OFF;
}

void warningLedFlash(void)
{
    warningLedState = WARNING_LED_FLASH;
}

void warningLedRefresh(void)
{
    switch (warningLedState) 
	{
        case WARNING_LED_OFF:
            RLED_OFF();
            break;
        case WARNING_LED_ON:
            RLED_ON();
            break;
        case WARNING_LED_FLASH:
            RLED_Change();
            break;
    }
}

void warningLedUpdate(void)
{
	if (xTaskGetTickCount() - warningLedTimer > 500)//500ms刷新一次灯的状态
	{
		warningLedRefresh();
		warningLedTimer = xTaskGetTickCount();
	}
}


