#ifndef __ANO_OPTICAL_H
#define __ANO_OPTICAL_H

#include "Headfile.h"



extern int16_t OF_VX, OF_VY;
extern int16_t OF_DIST_X,OF_DIST_Y;
extern uint16_t		OF_ALT;

void AnoOF_GetOneByte(uint8_t data);
void ANO_optical_Init(void);
void ANO_Optical_Update(void);
void check_optical(void);

#endif

