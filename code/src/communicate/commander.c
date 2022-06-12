#include "commander.h"
#include "rc_controls.h"
#include "runtime_config.h"
#include "alt_control.h"
#include "pos_estimator.h"
#include "state_control.h"
#include "safe_control.h"
#include "posxy_control.h"
#include "maths.h"
#include "led_task.h"
#include "open_mv.h"
#include "launch_control.h"
#include "safe_control.h"
#include "open_mv.h"
#include "math.h"
/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

//	ANGLE_MODE        = (1 << 0),   //�Ƕ�ģʽ
//	ACRO_MODE         = (1 << 1),  //����ģʽ
//	HEADFREE_MODE     = (1 << 2),  //ǰ�� ģʽ
//	NAV_ALTHOLD_MODE  = (1 << 3),  //����ģʽ
//	NAV_RTH_MODE      = (1 << 4),  //��������ģʽ
//	NAV_POSHOLD_MODE  = (1 << 5), 	
//	NAV_LAUNCH_MODE   = (1 << 7),  //һ�����ģʽ
//	FAILSAFE_MODE     = (1 << 8),
//	AUTO_LAND         = (1 << 9),//�Զ�����

/*commonder���ò���*/
#define DEAD_BAND					5			//����roll\pitch�е�����ֵ
#define DEAD_BAND_YAW	   	10			//yaw�е�����ֵ

#define MAX_ANGLE_ROLL		(30.0f)		//�������Ƕ�
#define MAX_ANGLE_PITCH  	(30.0f)

#define MAX_ANGLE_PITCH_TRACK  	(8.0f)

#define MAX_RATE_ROLL		(200.0f)	//�ֶ�ģʽ�������
#define MAX_RATE_PITCH		(200.0f)
#define MAX_RATE_YAW     	(200.0f)


#define ALT_HOLD_DEADBAND	50			//��������ң������ֵ
#define POS_HOLD_DEADBAND	30			//���㷽��ң������ֵ

#define RATE_POSITION(RATE_HZ, TICK) ((TICK % (RATE_500_HZ / RATE_HZ)) == 0)

#define GO_AHEAD_Count		200

//bool setupAltitudeHoldFlag = false;		//��־�����趨���
int16_t rcCommand[4];									//[1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

u8 Roll_direction = 0;		//Ĭ�ϲ����
u8 Start_command = 0; 		//��������


u16 Hover_count = 0;   //��ͣʱ�����
u16 roll_count = 0;     //
u16 pitch_count = 0;    //
u32 command_count = 0;  


autoLandState_t autoLandState = //�Զ�����
{
	.autoLandActive = 0,
	.autoLandTime = 0,
};	


//ң������ת��Ϊ����
static float rcCommandToRate(int16_t stick, float rate)
{
    return scaleRangef((float) stick, -500, 500, -rate, rate);
}

//ң������ת��Ϊ�Ƕ�
static float rcCommandToAngle(int16_t stick, float maxInclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float) stick, -500, 500, -maxInclination, maxInclination);
}

//ң�������������̺�ȥ������
static int16_t getAxisRcCommand(int16_t rawData, int16_t deadband)
{
    int16_t stickDeflection;
	
    stickDeflection = constrain(rawData - RC_MID, -500, 500);
    stickDeflection = applyDeadband(stickDeflection, deadband);
    return stickDeflection;
}


//static float rcCommandTo_Z_Rate(int16_t stick, float max_rate )   //ң������ת��ΪZ��Ŀ���ٶ�
//{
//		return scaleRangef((float) stick, -500, 500, -max_rate, max_rate);
//}

//��λ����ģʽ
static void resetSetpointMode(setpoint_t *setpoint)
{
	setpoint->mode.roll 	= 	modeDisable;
	setpoint->mode.pitch 	= 	modeDisable;
	setpoint->mode.yaw 		= 	modeDisable;
	
	setpoint->mode.x = modeDisable;
	setpoint->mode.y = modeDisable;
	setpoint->mode.z = modeDisable;
}

	u32 ratetick = 0;
//�趨��������ͷ���ģʽ
void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint)
{
		//��ȡң����ҡ��ֵ
		rcCommand[ROLL] = getAxisRcCommand(rcData[ROLL], DEAD_BAND);
		rcCommand[PITCH] = getAxisRcCommand(rcData[PITCH], DEAD_BAND);
		rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], DEAD_BAND_YAW);//������Z�����ֶ�����ʱ��Ϊ��������Ϊ����
		rcCommand[THROTTLE] = constrain(rcData[THROTTLE], RC_MIN, RC_MAX);

  resetSetpointMode(setpoint);//��λ����ģʽ

	//�ֶ�ģʽ������ģʽ�Ͷ���ģʽ
	if (FLIGHT_MODE(ACRO_MODE))
	{
		setpoint->mode.roll = modeVelocity;
		setpoint->mode.pitch = modeVelocity;
		setpoint->mode.yaw = modeVelocity;
		
		setpoint->attitudeRate.roll = 	rcCommandToRate(rcCommand[ROLL], MAX_RATE_ROLL);
		setpoint->attitudeRate.pitch = 	rcCommandToRate(rcCommand[PITCH], MAX_RATE_PITCH);
		setpoint->attitudeRate.yaw = 		rcCommandToRate(rcCommand[YAW], MAX_RATE_YAW);
	}


	else if (FLIGHT_MODE(ANGLE_MODE))
	{
		setpoint->mode.yaw = modeVelocity;
		setpoint->mode.roll = modeAbs;
		setpoint->mode.pitch = modeAbs;
		
		setpoint->attitude.roll = rcCommandToAngle(rcCommand[ROLL], MAX_ANGLE_ROLL);
		setpoint->attitude.pitch = rcCommandToAngle(rcCommand[PITCH], MAX_ANGLE_PITCH);
		
		setpoint->attitudeRate.yaw = rcCommandToRate(rcCommand[YAW], MAX_RATE_YAW);

		
	}

	
	else if(FLIGHT_MODE(NAV_POSHOLD_MODE)) //�������㹦�ܣ�Ŀ��roll,pitch �Ƕ��������趨
	{
		setpoint->mode.yaw = modeVelocity;
		setpoint->mode.roll = modeAbs;
		setpoint->mode.pitch = modeAbs;
		

		if(RATE_POSITION(50, ratetick))  //20HZ
		{

			setpoint->attitudeRate.yaw = 0;
		
			PosXY_Control(state, setpoint);

		}

  }

	else if (FLIGHT_MODE(HEADFREE_MODE))  // ѭ��ģʽ  roll  pid ����  pitch  ����
	{
		setpoint->mode.yaw = modeVelocity;
		setpoint->mode.roll = modeAbs;
		setpoint->mode.pitch = modeAbs;
		
		if(RATE_POSITION(50, ratetick))  //50HZ
		{
//				roll_count++;
			if(Start_command == 0)
				setpoint->attitude.pitch = 0.7;  //��ǰ����
			else if(Start_command == 1)
				setpoint->attitude.pitch = 0.1;     //�������
			else if(Start_command == 2)         //pitch ����
			{
					pitch_count++;
				
					if(pitch_count == 60)
						setpoint->attitude.pitch = 0.45;
					else if(pitch_count == 90)
					{
						setpoint->attitude.pitch = -0.12;
						pitch_count = 0;
					}
			
			}
								
			if (Roll_direction == 0)
				setpoint->attitude.roll = -0.6;
			else if (Roll_direction == 1)   //  ������
				setpoint->attitude.roll = -0.85;
			else if (Roll_direction == 2)   //  ���Ҵ��
				setpoint->attitude.roll = -0.5;
		}
		
		
//		if(RATE_POSITION(50, ratetick))  //50HZ
//		{
//					setpoint->attitudeRate.yaw = 0;
//				
//				setpoint->attitude.pitch = 0.3f;
//			Track_line_Control(state, setpoint);
//		}
	}



	////����ֵ����
	if (FLIGHT_MODE(AUTO_LAND))  //�����Զ�����
	{
			if(Land_Init==false)
					{
						AutoLand_Init();
						Land_Init = true;
					}
	//		Auto_Land(state, setpoint);
			if(state->position.z<15.0f||land_flag==1)
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
					if(RATE_POSITION(50, ratetick))  //20HZ	
						{
							setpoint->mode.z = modeVelocity;
							setpoint->velocity.z = LAND_VELOCITY;
							Alt_Control(state, setpoint);
						}	
			}
		
	}
	
	else if(FLIGHT_MODE(NAV_ALTHOLD_MODE)) //�������߹���
	{
				setpoint->mode.z = modeAbs;

		if(RATE_POSITION(50, ratetick))  //20HZ
		{
			
				if(Alttitude_control_setup==false)
				{
					Alt_Control_Init(state,	setpoint);
					Alttitude_control_setup = true;
				}
				Alt_Control(state, setpoint);	
		}
	}
	
	
	else if(FLIGHT_MODE(NAV_LAUNCH_MODE))  //һ�����ģʽ   ��غ���д��alt_control.c
	{
			if(!Launch_Init_Complete)  
			{
				if(ratetick%2==0)
				{
						Launch_Init(setpoint);	
				}
			}
			else if(Launch_Init_Complete)
			{
					if(RATE_POSITION(50, ratetick))  //20HZ	
					{
						setpoint->mode.z = modeVelocity;
						setpoint->velocity.z = Launch_Velocity;
						Alt_Control(state, setpoint);
					}
						
					if(state->position.z > (Launch_height - 10))
					{
							ENABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
							DISABLE_FLIGHT_MODE(NAV_LAUNCH_MODE);
							Alttitude_control_setup = false;
					}
			}	
	}
	else
	setpoint->thrust = rcCommand[THROTTLE];  //Ĭ�����������Ϊң��ֵ
	
	
	//�������ֵС�ڻ����MINCHECKʱ��yaw���Ʋ�������
	//Ŀ�����ڼ����ͽ�������ʱ��ֹ������ת
	if(setpoint->thrust <= RC_COMMANDER_MINCHECK)
	{
		setpoint->attitudeRate.yaw = 0;
	}
	

	
// ģʽ�л� 	
	if(RATE_POSITION(50, ratetick))
		
	{		
			if(Competition_MODE==1)  //�����ﶨ���
			{
				if((FLIGHT_MODE(NAV_ALTHOLD_MODE)))
				{
					Hover_count += 1;
					if(Hover_count >= 750)
							{
									Hover_count = 0;
									DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
									ENABLE_FLIGHT_MODE(AUTO_LAND);
							}
					}
			}
			
			else if(Competition_MODE==0)  //Ѳ��ģʽ
			{
				
				if((FLIGHT_MODE(NAV_ALTHOLD_MODE)))
				{
						if(WORK_MODE==START_POINT)
						{
								Hover_count += 1;
								if(Hover_count >= 300)   
								{
										WORK_MODE=END_POINT;
										Hover_count = 0;
//										Roll_direction = 1;  //������
//									  Start_command  = 0;
								}
							
						}
						
//						else if(WORK_MODE==GO_AHEAD)   //GO_AHEAD_Count 300
//						{
//								Hover_count += 1;
//							if(Hover_count >= GO_AHEAD_Count)
//							{
//									WORK_MODE=END_POINT;
//									Hover_count = 0;
//							}
//							else if(Hover_count == 50)
//							{
//									Start_command = 2;
//						setpoint->attitude.pitch = 0;								
//							}
//							else if(Hover_count == 100)
//							{
//									Roll_direction = 2;  //roll ���Ҵ��
//							}
//							else if(Hover_count == 150)
//							{
//									Roll_direction = 0;    //  roll �����
//							}
//							else if(Hover_count == 150) 
//							{
//									Start_command = 1;      //pitch �������
//							}
//						}
//						
						else if(WORK_MODE==END_POINT)
						{
							DISABLE_FLIGHT_MODE(NAV_ALTHOLD_MODE);
							ENABLE_FLIGHT_MODE(AUTO_LAND);
							Hover_count += 1;
							if(Hover_count >= 300 )
							{
									mwDisarm();   //����
									setpoint->thrust = 1000;
									land_flag=0;
									Hover_count = 0;
							}
						}	
				}
			}				
	}
	
	ratetick++;
}


//�����Զ�����ģʽ
void commanderActiveFailsafe(void)
{
	autoLandState.autoLandActive = true;
	autoLandState.autoLandTime = xTaskGetTickCount();
}

//��λ�Զ�����ģʽ
void resetFailsafe(void)
{
	autoLandState.autoLandActive = false;
	autoLandState.autoLandTime = 0;
}



