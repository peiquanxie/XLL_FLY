#ifndef __LED_H_
#define __LED_H_

#include "Headfile.h"

#define R_LED  				GPIO_PIN_1//LaunchPad°å
#define B_LED   			GPIO_PIN_2
#define G_LED			    GPIO_PIN_3
#define LED_BASE			GPIO_PORTF_BASE
#define SYSCTL_PERIPH_LED	SYSCTL_PERIPH_GPIOF



void GLED_Change(void);
void BLED_Change(void);
void RLED_Change(void);
void WLED_Change(void);

void RLED_OFF(void);
void RLED_ON(void);

void GLED_OFF(void);
void GLED_ON(void);

void BLED_OFF(void);
void BLED_ON(void);

void WLED_OFF(void);
void WLED_ON(void);

void LED_Init(void);
void warningLedUpdate(void);
void warningLedFlash(void);
void warningLedOFF(void);
void warningLedON(void);




#endif


