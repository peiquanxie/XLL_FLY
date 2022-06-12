#ifndef __POSXY_CONTROL_H
#define __POSXY_CONTROL_H

#include "Headfile.h"
#include "pid.h"

extern PidObject pidX, pidY, pidVX, pidVY;
extern bool posXY_control_setup;

void positionResetAllPID(void);
void PosXY_Control(const state_t *state, setpoint_t *setpoint);
void Track_line_Control(const state_t *state, setpoint_t *setpoint);

#endif

