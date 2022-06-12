#include "us100_height.h"
#include "optical_position.h"
#include "stabilizer_task.h"
#include "US_100.h"
#include "tof_height.h"
#include "usart.h"
#include "math.h"
#include "filter.h"
#include "KalmanFilter.h"
#include "runtime_config.h"

#define Height_Q 0.002f   //过程方差
#define Height_R 0.004f  //测量方差

value_k_t 	height_value_k;
value_k_1_t	height_value_k_1;

/*高度原始数据低通滤波参数*/
#define HEIGHT_RAW_LPF_CUTOFF_FREQ  	6.0f    

biquadFilter_t HEIGHT_RAW_FilterLPF;//US100原始数据二阶低通滤波器

/*高度速度低通滤波参数*/
#define US100_LPF_CUTOFF_FREQ  	6.0f    

biquadFilter_t US100FilterLPF;//高度速度二阶低通滤波器
//static bool isUS100Present=false;
//float velocity;

void US100_Height_Init(void)
{
  US100_Init();
	//初始化高度原始数据低通滤波器
	biquadFilterInitLPF(&HEIGHT_RAW_FilterLPF, US100_UPDATE_RATE, HEIGHT_RAW_LPF_CUTOFF_FREQ);
	biquadFilterInitLPF(&US100FilterLPF, US100_UPDATE_RATE, US100_LPF_CUTOFF_FREQ);
}

float velocity_Z_biquadFilter(void)
{
	return (biquadFilterApply(&US100FilterLPF, (sensorData.height.distance-sensorData.height.distance_pre)*US100_UPDATE_RATE));
}


char height_first_flag=0;
float High_US,High_TOF,High_fusion;

void Position_Height_Update(void)
{ 
		
	High_US=US100_Update();   //超声波高度
	//High_TOF=TOF_Update()*cosf(state.attitude.roll*DEG2RAD)*cosf(state.attitude.pitch*DEG2RAD);
	High_TOF=TOF_Update()*cosf(state.attitude.roll*DEG2RAD)*cosf(state.attitude.pitch*DEG2RAD);

		if (High_TOF > 0)
		ENABLE_STATE(CALIBRATE_MAG);
	else 
		DISABLE_STATE(CALIBRATE_MAG);
	
	if(height_first_flag==0)
		{
			High_fusion = High_US;
			height_first_flag=1;			
		}
		
	else if(High_TOF < 40)
		High_fusion = High_US;

	
	else if(High_TOF >=40 )
		High_fusion = High_TOF;

	
	height_value_k.Zk=biquadFilterApply(&HEIGHT_RAW_FilterLPF, High_fusion);  //高度原始数据低通滤波
	KalmanFilter(&height_value_k_1 , &height_value_k, Height_Q, Height_R);
	sensorData.height.distance_pre= height_value_k_1.Xk_1;
	sensorData.height.distance= height_value_k.Xk;
	state.position.z=sensorData.height.distance; 
	state.velocity.z=velocity_Z_biquadFilter();   //微分算出高度速度值（经二阶低通滤波）

}




