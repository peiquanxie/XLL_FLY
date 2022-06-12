#include "key.h"

#define Key2  GPIOPinRead(GPIO_PORTF_BASE , GPIO_PIN_0)
#define Key3  GPIOPinRead(GPIO_PORTF_BASE , GPIO_PIN_4)

void Key_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//
	// Unlock PF0 so we can change it to a GPIO input
	// Once we have enabled (unlocked) the commit register then re-lock it
	// to prevent further changes.  PF0 is muxed with NMI thus a special case.
	//
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_DIR_MODE_IN);//SW2
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_DIR_MODE_IN);//SW3
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

}

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
u8 KEY2_Scan(u8 mode)
{
	static u8 key_up=1;//按键按松开标志
	if(mode)key_up=1;  //支持连按		  
	if(key_up&&Key2==0)
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(Key2==0)return 1;
		
	}else if(Key2==1)key_up=1; 	    
 	return 0;// 无按键按下
}

//按键处理函数
//返回按键值
//mode:0,不支持连续按;1,支持连续按;
u8 KEY3_Scan(u8 mode)
{
	static u8 key_upp=1;//按键按松开标志
	if(mode)key_upp=1;  //支持连按		  
	if(key_upp&&Key3==0)
	{
		delay_ms(10);//去抖动 
		key_upp=0;
		if(Key3==0)return 1;
		
	}else if(Key3==1)key_upp=1; 	    
 	return 0;// 无按键按下
}


