#include "motor.h"

static uint16_t period;

static bool isInit = false;
u32 motor_ratios[] = {0, 0, 0, 0};
const u32 MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };

/*val:0-1000*/
static u16 ratioToCCRx(u16 val)
{
	return (MOTOR_TIM_CNT_FOR_HIGH + val * MOTOR_TIM_CNT_FOR_HIGH/ 1000);//MOTOR_TIM_CNT_FOR_HIGH为1ms高电平时间值
}


void motorsInit(void)//PWM初始化
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_4); // 80M/4=20M   
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
	
  GPIOPinConfigure(GPIO_PB6_M0PWM0);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);

  GPIOPinConfigure(GPIO_PC4_M0PWM6);
  GPIOPinConfigure(GPIO_PC5_M0PWM7);
	
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1

  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);//M0PWM6
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);//M0PWM7
	
  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
  // The period is set to 2.5ms (400 Hz)
  period = MOTOR_TIM_PERIOD; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period

  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
	
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);

  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	
  // Enable the outputs
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT |PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
//  PWM_Output(50,50,50,50);
	
	isInit = true;
}



//void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
//{
//  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,width1);//PC4 对应motor1
//  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,width2);//PC5 对应motor3
//  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,width3);//PB7 对应motor4
//  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,width4);//PB6 对应motor2
//}



/*设置电机PWM占空比*/
/*ithrust:0-1000*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,ratioToCCRx(ithrust));   //PC4 对应motor1
				break;
			case 1:		/*MOTOR_M2*/
				PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,ratioToCCRx(ithrust));//PB6 对应motor2
				break;
			case 2:		/*MOTOR_M3*/
				PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,ratioToCCRx(ithrust));//PC5 对应motor3
				break;
			case 3:		/*MOTOR_M4*/	
				PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,ratioToCCRx(ithrust));//PB7 对应motor4
				break;
			default: break;
		}	
	}
}


