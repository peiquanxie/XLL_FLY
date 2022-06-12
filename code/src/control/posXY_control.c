#include "posxy_control.h"
#include "commander.h"
#include "optical_position.h"
#include "config_param.h"
#include "ANO_optical.h"
#include "stabilizer_task.h"
#include <math.h>
#include "pid.h"
#include "maths.h"
#include "open_mv.h"

#define  Track_line_velocity   2.5f
bool posXY_control_setup= false;
bool Track_line_setup = false;  //循线模式配置标志位


PidObject pidX, pidY, pidVX, pidVY;

void PosXY_Control(const state_t *state, setpoint_t *setpoint)
{
//	check_optical();
	  if(posXY_control_setup==false)
		{
			positionResetAllPID();
//			reset_Position();
			setpoint->position.x =	0;  //设定红点为目标位置
			setpoint->position.y = 	0;
			posXY_control_setup = true;
		}
		
		setpoint->velocity.x = pidUpdate(&pidX, setpoint->position.x - state->position.x);
		setpoint->velocity.y = pidUpdate(&pidY, setpoint->position.y - state->position.y);
		

 //Roll and Pitch （单级）
//		setpoint->attitude.roll   = pidUpdate(&pidX, setpoint->position.x - state->position.x);
//		setpoint->attitude.pitch  = pidUpdate(&pidY, setpoint->position.y - state->position.y);
		
 //Roll and Pitch  （串级） (光流速度)
//		setpoint->attitude.roll 	= pidUpdate(&pidVX, setpoint->velocity.x - OF_VX);
//		setpoint->attitude.pitch  = pidUpdate(&pidVY, setpoint->velocity.y - OF_VY);

 //Roll and Pitch  （串级） (openMV速度)
		setpoint->attitude.roll 	= pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
		setpoint->attitude.pitch  = pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);

}


void positionResetAllPID(void)
{
	
	pidReset(&pidX);
	pidReset(&pidVX);
	pidReset(&pidY);
	pidReset(&pidVY);
	
}




void Track_line_Control(const state_t *state, setpoint_t *setpoint)
	
{
			  if(Track_line_setup==false)
		{
			
//			reset_Position();
			setpoint->position.x =	0;    //  设置机身相对线的位置
			Track_line_setup = true;
			
		}
		
//		setpoint->velocity.x = pidUpdate(&pidX, setpoint->position.x - state->position.x);
			setpoint->velocity.x = pidUpdate(&pidX, setpoint->position.x - state->position.x);
//		if(Turn_left_Flag == 0)
//		{
			setpoint->attitude.roll 	=  pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
//		}
//		else if(Turn_left_Flag == 1)
//		{
//			setpoint->attitude.roll 	= 1.2f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x) - 0.25f;
//		}
			setpoint->attitude.pitch  =  pidUpdate(&pidVY, Track_line_velocity - OF_VY);
			setpoint->attitude.pitch = constrainf(setpoint->attitude.pitch, -2.3f, 0.4f);
		

}


