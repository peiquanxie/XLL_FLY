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

//	ANGLE_MODE        = (1 << 0),   //角度模式
//	ACRO_MODE         = (1 << 1),  //速率模式
//	HEADFREE_MODE     = (1 << 2),  //前进 模式
//	NAV_ALTHOLD_MODE  = (1 << 3),  //定高模式
//	NAV_RTH_MODE      = (1 << 4),  //定高速率模式
//	NAV_POSHOLD_MODE  = (1 << 5), 	
//	NAV_LAUNCH_MODE   = (1 << 7),  //一键起飞模式
//	FAILSAFE_MODE     = (1 << 8),
//	AUTO_LAND         = (1 << 9),//自动降落

/*commonder配置参数*/
#define DEAD_BAND					5			//设置roll\pitch中点死区值
#define DEAD_BAND_YAW	   	10			//yaw中点死区值

#define MAX_ANGLE_ROLL		(30.0f)		//自稳最大角度
#define MAX_ANGLE_PITCH  	(30.0f)

#define MAX_ANGLE_PITCH_TRACK  	(8.0f)

#define MAX_RATE_ROLL		(200.0f)	//手动模式最大速率
#define MAX_RATE_PITCH		(200.0f)
#define MAX_RATE_YAW     	(200.0f)


#define ALT_HOLD_DEADBAND	50			//定高油门遥杆死区值
#define POS_HOLD_DEADBAND	30			//定点方向遥杆死区值

#define RATE_POSITION(RATE_HZ, TICK) ((TICK % (RATE_500_HZ / RATE_HZ)) == 0)

#define GO_AHEAD_Count		200

//bool setupAltitudeHoldFlag = false;		//标志定高设定完成
int16_t rcCommand[4];									//[1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

u8 Roll_direction = 0;		//默认不打杆
u8 Start_command = 0; 		//启动命令


u16 Hover_count = 0;   //悬停时间计数
u16 roll_count = 0;     //
u16 pitch_count = 0;    //
u32 command_count = 0;  


autoLandState_t autoLandState = //自动降落
{
	.autoLandActive = 0,
	.autoLandTime = 0,
};	


//遥控数据转换为速率
static float rcCommandToRate(int16_t stick, float rate)
{
    return scaleRangef((float) stick, -500, 500, -rate, rate);
}

//遥控数据转换为角度
static float rcCommandToAngle(int16_t stick, float maxInclination)
{
    stick = constrain(stick, -500, 500);
    return scaleRangef((float) stick, -500, 500, -maxInclination, maxInclination);
}

//遥控数据限制量程和去除死区
static int16_t getAxisRcCommand(int16_t rawData, int16_t deadband)
{
    int16_t stickDeflection;
	
    stickDeflection = constrain(rawData - RC_MID, -500, 500);
    stickDeflection = applyDeadband(stickDeflection, deadband);
    return stickDeflection;
}


//static float rcCommandTo_Z_Rate(int16_t stick, float max_rate )   //遥控数据转换为Z轴目标速度
//{
//		return scaleRangef((float) stick, -500, 500, -max_rate, max_rate);
//}

//复位控制模式
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
//设定控制命令和飞行模式
void commanderGetSetpoint(const state_t *state, setpoint_t *setpoint)
{
		//获取遥控四摇杆值
		rcCommand[ROLL] = getAxisRcCommand(rcData[ROLL], DEAD_BAND);
		rcCommand[PITCH] = getAxisRcCommand(rcData[PITCH], DEAD_BAND);
		rcCommand[YAW] = -getAxisRcCommand(rcData[YAW], DEAD_BAND_YAW);//传感器Z轴右手定则逆时针为正，所以为负号
		rcCommand[THROTTLE] = constrain(rcData[THROTTLE], RC_MIN, RC_MAX);

  resetSetpointMode(setpoint);//复位所有模式

	//手动模式和自稳模式和定点模式
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

	
	else if(FLIGHT_MODE(NAV_POSHOLD_MODE)) //开启定点功能，目标roll,pitch 角度由以下设定
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

	else if (FLIGHT_MODE(HEADFREE_MODE))  // 循线模式  roll  pid 控制  pitch  开环
	{
		setpoint->mode.yaw = modeVelocity;
		setpoint->mode.roll = modeAbs;
		setpoint->mode.pitch = modeAbs;
		
		if(RATE_POSITION(50, ratetick))  //50HZ
		{
//				roll_count++;
			if(Start_command == 0)
				setpoint->attitude.pitch = 0.7;  //向前启动
			else if(Start_command == 1)
				setpoint->attitude.pitch = 0.1;     //结束打舵
			else if(Start_command == 2)         //pitch 调整
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
			else if (Roll_direction == 1)   //  向左打舵
				setpoint->attitude.roll = -0.85;
			else if (Roll_direction == 2)   //  向右打舵
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



	////油门值处理
	if (FLIGHT_MODE(AUTO_LAND))  //开启自动降落
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
					mwDisarm();   //锁定
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
	
	else if(FLIGHT_MODE(NAV_ALTHOLD_MODE)) //开启定高功能
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
	
	
	else if(FLIGHT_MODE(NAV_LAUNCH_MODE))  //一键起飞模式   相关函数写在alt_control.c
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
	setpoint->thrust = rcCommand[THROTTLE];  //默认情况下油门为遥控值
	
	
	//如果油门值小于或等于MINCHECK时，yaw控制不起作用
	//目的是在加锁和解锁动作时防止四轴自转
	if(setpoint->thrust <= RC_COMMANDER_MINCHECK)
	{
		setpoint->attitudeRate.yaw = 0;
	}
	

	
// 模式切换 	
	if(RATE_POSITION(50, ratetick))
		
	{		
			if(Competition_MODE==1)  //挂重物定红点
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
			
			else if(Competition_MODE==0)  //巡线模式
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
//										Roll_direction = 1;  //向左打舵
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
//									Roll_direction = 2;  //roll 向右打舵
//							}
//							else if(Hover_count == 150)
//							{
//									Roll_direction = 0;    //  roll 不打舵
//							}
//							else if(Hover_count == 150) 
//							{
//									Start_command = 1;      //pitch 结束打舵
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
									mwDisarm();   //锁定
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


//激活自动降落模式
void commanderActiveFailsafe(void)
{
	autoLandState.autoLandActive = true;
	autoLandState.autoLandTime = xTaskGetTickCount();
}

//复位自动降落模式
void resetFailsafe(void)
{
	autoLandState.autoLandActive = false;
	autoLandState.autoLandTime = 0;
}



