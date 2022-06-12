#ifndef __TOF_HEIGHT_H
#define __TOF_HEIGHT_H

#include "Headfile.h"

extern u16 tof_height;
void TOF_Init(void);
void GetOneByte(uint8_t data);
float TOF_Update(void);

#endif

