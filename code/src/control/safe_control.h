#ifndef __SAFE_CONTROL_H
#define __SAFE_CONTROL_H

#include "Headfile.h"

#define LAND_VELOCITY   -18.0   //自动降落速度

extern float LAND_ThrustAdj;
extern bool Land_Init;

extern char land_flag;
void Auto_Land(const state_t *state, setpoint_t *setpoint);
void AutoLand_Init(void);
	
#endif

