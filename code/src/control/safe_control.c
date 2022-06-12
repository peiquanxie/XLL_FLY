 #include "safe_control.h"
#include "stabilizer_task.h"
#include "runtime_config.h"
#include "commander.h"
#include "state_control.h"
#include "maths.h"
#include "pid.h"
#include "alt_control.h"


//������ͣ����1580  �����½�1560  ��������1590  �������1570  ����������ͣ1570
//#define LAND_BaseTHROTTLE		1565		//�Զ���������ֵ  

char land_flag=0;
float LAND_ThrustAdj=0;
float LAND_BaseTHROTTLE = 1555;
bool Land_Init=false;
void Slow_Down(float VZ,  float *thrust)
{
	
	LAND_ThrustAdj = pidUpdate(&pid[VELOCITY_LAND], LAND_VELOCITY - VZ);
	*thrust = LAND_BaseTHROTTLE + LAND_ThrustAdj;
	*thrust = constrainf(*thrust, 1480, 1590);	
	
//	*thrust += LAND_ThrustAdj;
//	*thrust = constrainf(*thrust, 1495, 1590);	

}

void AutoLand_Init(void)  //�Զ������ʼ��
{
	LAND_BaseTHROTTLE = setpoint.thrust;
		LAND_BaseTHROTTLE = constrainf (LAND_BaseTHROTTLE , altHoldThrustBaseMIN , altHoldThrustBaseMAX);
		setpoint.velocity.z = LAND_VELOCITY;
//		pidSetIntegral(&pid[VELOCITY_LAND], 0);
//	pidReset(&pid[VELOCITY_LAND]);
}

void Auto_Land(const state_t *state, setpoint_t *setpoint)
{ 
	  ENABLE_FLIGHT_MODE(ANGLE_MODE);//��������ģʽ
		setpoint->attitude.roll = 0;
 		setpoint->attitude.pitch = 0;
		setpoint->attitude.yaw = 0;
	  if(state->position.z<10.0f||land_flag==1)
		{
			setpoint->thrust -= 1;
			land_flag=1;
			if(setpoint->thrust<1050)
			{
				mwDisarm();   //����
				setpoint->thrust = 1000;
				land_flag=0;
			}
		}
		else if(ARMING_FLAG(ARMED))
		{
      Slow_Down(state->velocity.z, &setpoint->thrust);
		}
		
}

