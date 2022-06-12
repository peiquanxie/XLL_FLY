#include <stdbool.h>
#include "alt_control.h"  
#include "data_type.h"
#include "state_control.h"
#include "maths.h"
#include "launch_control.h"
#include "commander.h"
float altThrustAdj;
float altHoldThrust;
bool Alttitude_control_setup = false;
bool Launch_Init_Complete = false;

float altHoldThrottleBase = 1575;	//定高油门基础值  待实验测量


bool setupAltitudeHoldFlag = false;		//标志定高设定完成

void Alt_Control(const state_t *state,setpoint_t *setpoint)
{
//	if(Alttitude_control_setup==false)
//	{
//		Alt_Control_Init(state,	setpoint);
//		Alttitude_control_setup = true;
//	} 

	if(setpoint->mode.z == modeAbs)
		{
			setpoint->velocity.z = pidUpdate(&pid[POSHOLD_Z], setpoint->position.z - state->position.z);
		}

	if(setpoint->mode.z != modeDisable)
		{			
			altThrustAdj = pidUpdate(&pid[VELOCITY_Z], setpoint->velocity.z - state->velocity.z);
			
			altHoldThrust = altThrustAdj + altHoldThrottleBase;
		}
		
		setpoint->thrust = constrainf (altHoldThrust , altHoldThrottleMIN ,altHoldThrottleMAX);	
}


void Alt_Control_Init(const state_t *state, setpoint_t *setpoint)
{
	setpoint->position.z = Launch_height;//设定目标高度为当前估测的高度
//	setpoint->velocity.z = 0.0f;
//	altHoldThrottleBase =	setpoint->thrust;         //当前油门为定高基础油门
	altHoldThrottleBase = constrainf (altHoldThrottleBase , altHoldThrustBaseMIN , altHoldThrustBaseMAX);
//	stateControlSetVelocityZPIDIntegration(0);
}


//设置Z轴速度PID的积分项
void stateControlSetVelocityZPIDIntegration(float integ)
{
	pidSetIntegral(&pid[VELOCITY_Z], integ);
}


void Launch_Init(setpoint_t *setpoint)
{
		if(setpoint->thrust>=1180&&setpoint->thrust< LaunchThrottleBase)
			setpoint->thrust+=1;
		else if(setpoint->thrust<1180)
			setpoint->thrust = 1180;
		else 
		{
			Launch_Init_Complete = true;
//			setpoint->position.z = Launch_height;  //起飞 定高高度
			setpoint->velocity.z = Launch_Velocity;
			Hover_count = 0;
			altHoldThrottleBase = LaunchThrottleBase;
			stateControlSetVelocityZPIDIntegration(0);
		}
}



