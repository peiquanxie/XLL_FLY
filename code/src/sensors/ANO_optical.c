#include "ANO_optical.h"
#include "filter.h"
 #include "safe_control.h"
 #include "stabilizer_task.h"
#include "position_task.h"

///*ANO�����ٶȵ�ͨ�˲�����*/
//#define ANO_LPF_CUTOFF_FREQ  	10.0f

//QUALITY������������������ֵԽ�󣬱�ʾ������������Խ��  LIGHT������ǿ��
uint8_t		OF_QUA , OF_LIGHT;

//OF_DX2 , OF_DY2 ����λΪ����/�룬����С������ƫ���ʺ����ٶȻ����ơ�
//OF_DX2FIX , OF_DY2FIX: ������������ٶ����ݣ������ϴ󣬻��ֺ�ƫ�Ʒ���С���ʺ����ڻ��ּ��㣻
int16_t		OF_DX2 , OF_DY2 , OF_DX2FIX , OF_DY2FIX;

//OF_DIS_X , OF_DIS_Y��������֣���λ cm
int16_t		OF_DIS_X , OF_DIS_Y;

//OF_ALT:ԭʼ�߶�  OF_ALT2�������߶�
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
	
	if(*(data_buf+3)==0X51)//������Ϣ
	{
		if(*(data_buf+5)==1)//�ںϺ������Ϣ
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
	if(*(data_buf+3)==0X52)//�߶���Ϣ
	{
		if(*(data_buf+5)==0)//ԭʼ�߶���Ϣ
		{
			OF_ALT = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
		}
//		else if(*(data_buf+5)==1)//�ںϺ�߶���Ϣ
//		{
//			OF_ALT2 = (uint16_t)(*(data_buf+6)<<8)|*(data_buf+7) ;
//		}
	}
}	
	
static uint8_t _datatemp[30];
static u8 _data_cnt = 0;

//AnoOF_GetOneByte�ǳ������ݽ�������������ÿ���յ�һ�ֽڹ������ݣ����ñ�����һ�Σ������������Ǵ����յ�������
//����������α����ã����ս��յ�������һ֡���ݺ󣬻��Զ��������ݽ�������AnoOF_DataAnl
void AnoOF_GetOneByte(uint8_t data)
{

	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		_datatemp[0]=data;
	}
	else if(state==1&&data==0x22)	//Դ��ַ
	{
		state=2;
		_datatemp[1]=data;
	}
	else if(state==2)			//Ŀ�ĵ�ַ
	{
		state=3;
		_datatemp[2]=data;
	}
	else if(state==3)			//������
	{
		state = 4;
		_datatemp[3]=data;
	}
	else if(state==4)			//����
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





