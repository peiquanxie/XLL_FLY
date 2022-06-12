#ifndef __ACCELEROMETER_H
#define __ACCELEROMETER_H

#include "Headfile.h"
#include "mpu6500.h"
 
#define GRAVITY_CMSS    980.665f 
#define GRAVITY_MSS     9.80665f

#define CALIBRATING_ACC_CYCLES          400

extern Axis3i16 accADC;

bool accInit(float accUpdateRate);
void accUpdate(Axis3f *acc);
bool accIsCalibrationComplete(void);
void accSetCalibrationCycles(uint16_t calibrationCyclesRequired);


#endif

