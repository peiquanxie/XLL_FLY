#ifndef __OPTICAL_POSITION_H__
#define __OPTICAL_POSITION_H__

#include "Headfile.h"

#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */

void Get_OpticalData(void);
void reset_Position(void);
bool Optical_Position_Update(state_t *state, float dt);

#endif


