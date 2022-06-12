#ifndef __USART_TASK_H
#define __USART_TASK_H

#include "Headfile.h"
#include "atkp.h"



void usartTxTask(void *param);
void usartRxTask(void *param);

void atkpInit(void);
bool atkpReceivePacketBlocking(atkp_t *p);


#endif


