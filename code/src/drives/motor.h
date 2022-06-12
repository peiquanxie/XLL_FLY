#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "Headfile.h" 

#define TIM_CLOCK_HZ 				80000000
#define MOTOR_TIM_PRESCALE 		4

#define STANDARD_PWM_PERIOD      0.0025// 2.5ms = 400Hz
#define MOTOR_TIM_PERIOD 		 (uint16_t)(TIM_CLOCK_HZ * STANDARD_PWM_PERIOD / MOTOR_TIM_PRESCALE)//定时器重装载值
#define MOTOR_TIM_CNT_FOR_HIGH   (uint16_t)(TIM_CLOCK_HZ * 0.001 / MOTOR_TIM_PRESCALE)//1000us高电平所需计数值



#define NBR_OF_MOTORS 	4
#define MOTOR_M1  		0
#define MOTOR_M2  		1
#define MOTOR_M3  		2
#define MOTOR_M4  		3


void motorsInit(void);
void motorsSetRatio(u32 id, u16 ithrust);
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4);

#endif


