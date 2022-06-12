#ifndef __delay_h__
#define __delay_h__

#include "Headfile.h"

void Sys_Clock_Init(void);
void systick_Init(void);
void delay_us(u32 nus);
void delay_ms(u16 nms);
u32 read_tick(void);

#endif
