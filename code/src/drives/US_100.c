#include "stabilizer_task.h"
#include "US_100.h"
#include "usart.h"
#include "delay.h"
#include "math.h"
#include "filter.h"
#include "runtime_config.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"



char SR04_error=0;
u16 Start_Tail=0,Last_Start_Tail;


float distance_up;


float DistanceCombine(uint8 MSB,uint8 LSB)
{
  return (256*(MSB)+(LSB))/10.0f;//��λcm
}

float US_TemperatureRead(uint8 data)
{
  return (data-45)/1.0;
}


void US_Start(void)
{	
  UARTCharPut(UART3_BASE, US_Distance_CMD);
}


uint8_t US_Update_Flag=0;
float US_DistanceRead(void)
{
	  float US_Distance;
    US_Start();        //����������
    Start_Tail=COM0_Rx_Buf.Tail;
    if(ABS(Start_Tail-Last_Start_Tail)==2)  //ʹ�û��ζ��ж�ȡ���������룬��߶�ȡЧ��
    {
      if(Start_Tail==0)
      {
        US_Distance=DistanceCombine(COM0_Rx_Buf.Ring_Buff[2],
                                    COM0_Rx_Buf.Ring_Buff[3]);
        US_Update_Flag=1;
      }
      else if(Start_Tail==1)
      {
        US_Distance=DistanceCombine(COM0_Rx_Buf.Ring_Buff[3],
                                    COM0_Rx_Buf.Ring_Buff[0]);
        US_Update_Flag=1;
      }
      else if(Start_Tail==2)
      {
        US_Distance=DistanceCombine(COM0_Rx_Buf.Ring_Buff[0],
                                    COM0_Rx_Buf.Ring_Buff[1]);
        US_Update_Flag=1;
      }
      else if(Start_Tail==3)
      {
        US_Distance=DistanceCombine(COM0_Rx_Buf.Ring_Buff[1],
                                    COM0_Rx_Buf.Ring_Buff[2]);
        US_Update_Flag=1;
      }
    }
    Last_Start_Tail=Start_Tail;
//  if(US_Distance<=User_Height_Max&&US_Distance>0)  Sensor_Flag.Hcsr04_Health=1;
//  else  Sensor_Flag.Hcsr04_Health=0; 
//		US_Distance=US_Distance*cosf(state.attitude.roll)*cosf(state.attitude.pitch);  //��ǲ���
	   distance_up=US_Distance;
		return US_Distance;
}


bool US100_Init(void)
{
	
	ConfigureUART3();  //��ʼ������3
	 return true;
}


u32 tick = 0;
float height_raw;

float US100_Update(void )
{
	
	if((tick%(US100_UPDATE_RATE/20))==0)  //ȷ�����������ݸ���Ƶ��20HZ
	{ 
		height_raw= US_DistanceRead();

	}
	
	tick++;
	return height_raw;
}



