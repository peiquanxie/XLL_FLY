#include "stabilizer_task.h"
#include "position_task.h"
#include "open_mv.h"
#include "usart.h"
#include "maths.h"
#include "filter.h"
#include "led_task.h"
#include "KalmanFilter.h"
#include "optical_position.h"



#define MV_Q 0.004f   //过程方差
#define MV_R 0.015f  //测量方差

//参数设定：
#define PIXELPDEG_X    2.4f  		//每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。//3.2f->(160*120,3.6mm)  2.4f->(160*120,2.8mm)
#define PIXELPDEG_Y    2.4f 		//每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。
#define CMPPIXEL_X     0.01f    //每像素对应的地面距离，与焦距和高度有关，需要调试标定。//目前粗略标定
#define CMPPIXEL_Y     0.01f    //每像素对应的地面距离，与焦距和高度有关，需要调试标定。

/*openMV原始数据低通滤波参数*/
#define MV_RAW_LPF_CUTOFF_FREQ  	7.0f    

#define MV_Vel_LPF_CUTOFF_FREQ    6.0f

float positionx = 0.0f;
float positiony = 0.0f;
char WORK_MODE = START_POINT;



biquadFilter_t MV_RAW_FilterLPFX;//openmv原始数据二阶低通滤波器
biquadFilter_t MV_RAW_FilterLPFY;//openmv原始数据二阶低通滤波器

biquadFilter_t MV_Vel_FilterLPFX;//openmvX方向速度二阶低通滤波器
biquadFilter_t MV_Vel_FilterLPFY;//openmvY方向速度二阶低通滤波器 

int8_t MV_Angle=0,MV_Distance=0, MV_line_flag=0;
u16 MV_DotX=0, MV_DotY=0;
value_k_t MV_X_value_k,MV_Y_value_k;
value_k_1_t	MV_X_value_k_1,MV_Y_value_k_1;

void Open_MV_Init(void)
{
		ConfigureUART0();
		ConfigureUART6();	
		biquadFilterInitLPF(&MV_RAW_FilterLPFX, OPEN_MV_UPDATE_RATE, MV_RAW_LPF_CUTOFF_FREQ);
		biquadFilterInitLPF(&MV_RAW_FilterLPFY, OPEN_MV_UPDATE_RATE, MV_RAW_LPF_CUTOFF_FREQ);
		biquadFilterInitLPF(&MV_Vel_FilterLPFX, OPEN_MV_UPDATE_RATE, MV_Vel_LPF_CUTOFF_FREQ);
		biquadFilterInitLPF(&MV_Vel_FilterLPFY, OPEN_MV_UPDATE_RATE, MV_Vel_LPF_CUTOFF_FREQ);
}

void MV_Start_Read(void)
{
	if(position_control_flag==DOWN)
	{
		if(Competition_MODE==1)
			UARTCharPut(UART6_BASE, 0x54);
		else if(Competition_MODE==0)
			UARTCharPut(UART6_BASE, 0x32);
	}
	else if(position_control_flag==LEFT)
	{
		if(WORK_MODE==START_POINT)
				UARTCharPut(UART0_BASE, 0x32);
		else if(WORK_MODE==GO_AHEAD)
				UARTCharPut(UART0_BASE, 0x33);
		else if(WORK_MODE==LANDED)
				UARTCharPut(UART0_BASE, 0x34);
  }
}

void Open_MV_Read_Data(float *DOT_X, float *DOT_Y,state_t *state)
{
	float incline_pixel_x,incline_pixel_y;
	MV_Start_Read();
	if(position_control_flag==DOWN)
	{
		incline_pixel_x = constrain(state->attitude.roll * PIXELPDEG_X,-80,80);
		incline_pixel_y = constrain(state->attitude.pitch * PIXELPDEG_Y,-60,60);
		*DOT_X= (80- (MV_DotX-incline_pixel_x) ) * state->position.z *CMPPIXEL_X;   
		*DOT_Y=-(60- (MV_DotY+incline_pixel_y) ) * state->position.z *CMPPIXEL_Y; 
	}
  else if(position_control_flag==LEFT)
	{
		*DOT_X= (160- (MV_DotX) ) * 10 *CMPPIXEL_X;   
		*DOT_Y=-(120- (MV_DotY) ) * 10 *CMPPIXEL_Y; 
	}

}

float DOT_X,DOT_Y;    //采集图像160*120,中心点为（0,0）
void OpenMV_Position_Update(state_t *state, float dt)
{
	static u32 tick=0;
	
	if((tick%(OPEN_MV_UPDATE_RATE/50))==0)  
		Open_MV_Read_Data(&DOT_X,&DOT_Y,state);
	
	MV_X_value_k.Zk=biquadFilterApply(&MV_RAW_FilterLPFX, DOT_X);
	MV_Y_value_k.Zk=biquadFilterApply(&MV_RAW_FilterLPFY, DOT_Y); //原始数据滤波

	
	KalmanFilter(&MV_X_value_k_1 , &MV_X_value_k, MV_Q, MV_R);
	KalmanFilter(&MV_Y_value_k_1 , &MV_Y_value_k, MV_Q, MV_R);//卡尔曼滤波
	 
	state->position.x=MV_X_value_k.Xk;
	state->position.y=MV_Y_value_k.Xk;//水平位置
	state->velocity.x=biquadFilterApply(&MV_Vel_FilterLPFX, (MV_X_value_k.Xk-MV_X_value_k.Xk_1)*OPEN_MV_UPDATE_RATE);
	state->velocity.y=biquadFilterApply(&MV_Vel_FilterLPFY, (MV_Y_value_k.Xk-MV_Y_value_k.Xk_1)*OPEN_MV_UPDATE_RATE);

	tick++;
}





