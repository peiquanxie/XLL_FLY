#ifndef __OPEN_MV_H
#define __OPEN_MV_H

#include "Headfile.h"


#define NO_LINE     0
#define START_POINT 1
#define GO_AHEAD    2
#define END_POINT   3
#define LANDED      4

extern char WORK_MODE;
extern int8_t MV_Angle,MV_Distance, MV_line_flag;
extern u16 MV_DotX, MV_DotY;
extern float DOT_X,DOT_Y;
extern float positionx ;
extern float positiony ;
void Open_MV_Init(void);
void OpenMV_Position_Update(state_t *state, float dt);



#endif

