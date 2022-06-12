#ifndef __LED_TASK_H__
#define __LED_TASK_H__

#include "Headfile.h"
#include "led.h"

extern char AUTO_FLY ;
extern char Competition_MODE;

void led1Task(void* param);
void Init_FlyMode(void);
void Init_Flag(void);

#endif


