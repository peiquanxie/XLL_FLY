#include "tof_height.h"
#include "ANO_optical.h"

uint8_t tof_data[10];
u8 _data_cnt;

u16 tof_height;
u16 tof_trength;
u8 Mode;

void TOF_Init(void)
{
	ConfigureUART7();
}

void TOF_DataAnl(uint8_t *data_buf,uint8_t num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+8)))		return;		

	tof_height = (*(data_buf+3)<<8)|*(data_buf+2) ;
	tof_trength = (*(data_buf+5)<<8)|*(data_buf+4) ;
	Mode = *(data_buf+6);	
	
}	
	

//AnoOF_GetOneByte是初级数据解析函数，串口每接收到一字节光流数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数AnoOF_DataAnl
void GetOneByte(uint8_t data)
{

	static u8 _data_len = 0;
	static u8 state = 0;
	
	if(state==0&&data==0x59)
	{
		state=1;
		tof_data[0]=data;
	}
	else if(state==1&&data==0x59)	
	{
		state=2;
		tof_data[1]=data;
		_data_len = 6;
		_data_cnt = 0;
	}

	else if(state==2&&_data_len>0)
	{
		_data_len--;
		tof_data[2+_data_cnt++]=data;
		if(_data_len==0)
			state = 3;
	}
	else if(state==3)
	{
		state = 0;
		tof_data[8]=data;
		TOF_DataAnl(tof_data,9);
	}
	else
		state = 0;
}

float TOF_Update(void)
{
	return tof_height;
//	return OF_ALT;
}

