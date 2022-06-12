#include "ANO_optical.h"
#include "filter.h"
 #include "safe_control.h"
 #include "stabilizer_task.h"
#include "position_task.h"

///*ANO光流速度低通滤波参数*/
//#define ANO_LPF_CUTOFF_FREQ  	10.0f

//QUALITY：光流数据质量，数值越大，表示光流数据质量越好  LIGHT：光线强度
uint8_t		OF_QUA , OF_LIGHT;

//OF_DX2 , OF_DY2 ：单位为厘米/秒，噪声小，有零偏，适合做速度环控制。
//OF_DX2FIX , OF_DY2FIX: 积分修正后的速度数据，噪声较大，积分后偏移幅度小，适合用于积分计算；
int16_t		OF_DX2 , OF_DY2 , OF_DX2FIX , OF_DY2FIX;

//OF_DIS_X , OF_DIS_Y：距离积分，单位 cm
int16_t		OF_DIS_X , OF_DIS_Y;

//OF_ALT:原始高度  OF_ALT2：修正高度
uint16_t	OF_ALT ;



//biquadFilter_t ANO_OP_FilterLPF_X;
//biquadFilter_t ANO_OP_FilterLPF_Y;

//biquadFilter_t ANO_OP_FilterLPF_VX;
//biquadFilter_t ANO_OP_FilterLPF_VY;

void ANO_optical_Init(void)
{
	ConfigureUART7();
//	biquadFilterInitLPF(&ANO_OP_FilterLPF_VX, OPTICAL_UPDATE_RATE, ANO_LPF_CUTOFF_FREQ);
//	biquadFilterInitLPF(&ANO_OP_FilterLPF_VY, OPTICAL_UPDATE_RATE, ANO_LPF_CUTOFF_FREQ);
//	biquadFilterInitLPF(&ANO_OP_FilterLPF_X, OPTICAL_UPDATE_RATE, ANO_LPF_CUTOFF_FREQ);
//	biquadFilterInitLPF(&ANO_OP_FilterLPF_Y, OPTICAL_UPDATE_RATE, ANO_LPF_CUTOFF_FREQ);
}

void AnoOF_DataAnl(uint8_t *data_buf,uint8_t num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		
	
	if(*(data_buf+3)==0X51)//光流信息
	{
		if(*(data_buf+5)==1)//融合后光流信息
		{
			OF_QUA 		= *(data_buf+6);
			OF_DX2		= (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) ;
			OF_DY2		= (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) ;
			OF_DX2FIX	= (int16_t)(*(data_buf+11)<<8)|*(data_buf+12) ;
			OF_DY2FIX	= (int16_t)(*(data_buf+13)<<8)|*(data_buf+14) ;
			OF_DIS_X  = (int16_t)(*(data_buf+15)<<8)|*(data_buf+16) ;
			OF_DIS_Y  = (int16_t)(*(data_buf+17)<<8)|*(data_buf+18) ;
			OF_LIGHT  	= *(data_buf+19);
				
			
		}
	}
	if(*(data_buf+3)==0X52)//高度信息
	{
		if(*(data_buf+5)==0)//原始高度信息
		{
			OF_ALT = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
		}
//		else if(*(data_buf+5)==1)//融合后高度信息
//		{
//			OF_ALT2 = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
//		}
	}
}	
	
static uint8_t _datatemp[30];
static u8 _data_cnt = 0;

//AnoOF_GetOneByte是初级数据解析函数，串口每接收到一字节光流数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数AnoOF_DataAnl
void AnoOF_GetOneByte(uint8_t data)
{

	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0x22)	//源地址
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2)			//目的地址
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3)			//功能字
	{
		state = 4;
		_datatemp[3]=data;
	}
	else if(state==4)			//长度
	{
		state = 5;
		_datatemp[4]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==5&&_data_len>0)
	{
		_data_len--;
		_datatemp[5+_data_cnt++]=data;
		if(_data_len==0)
			state = 6;
	}
	else if(state==6)
	{
		state = 0;
		_datatemp[5+_data_cnt]=data;
		AnoOF_DataAnl(_datatemp,_data_cnt+6);//anoof_data_ok = 1 ;//
	}
	else
		state = 0;
}


int16_t OF_VX, OF_VY;
int16_t OF_DIST_X,OF_DIST_Y;

void ANO_Optical_Update(void)
{
	
		OF_VX = -OF_DY2FIX;
		OF_VY = OF_DX2FIX;
		OF_DIST_X = -OF_DIS_Y;
		OF_DIST_Y = OF_DIS_X;	
//	state->position.x = biquadFilterApply(&ANO_OP_FilterLPF_X, OF_DIS_X);
//	state->position.y = biquadFilterApply(&ANO_OP_FilterLPF_Y, OF_DIS_Y);
//	state->velocity.x = biquadFilterApply(&ANO_OP_FilterLPF_VX, OF_DX2FIX);
//	state->velocity.y = biquadFilterApply(&ANO_OP_FilterLPF_VY, OF_DY2FIX);
}

void check_optical(void)
{
	if(OF_ALT==0 || OF_QUA==0)
			Auto_Land(&state,&setpoint);
}





