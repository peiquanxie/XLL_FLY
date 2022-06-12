#include "Usart.h"
#include "atkp.h"
#include "open_mv.h"
#include "tof_height.h"
#include "ANO_optical.h"
#include "usart_task.h"



#define USBLINK_TX_QUEUE_SIZE 	30 /*���ն��и���*/


atkp_t rxPacket;


//����ѭ�����л������ݶ���
RingBuff_t COM0_Rx_Buf;

void ANO_DT_Data_Receive_Prepare(u8 data)//ANO����վ���ݽ���
{
  
	static u8 cksum = 0;
  static u8 _data_cnt = 0;
  static u8 state = 0;
	
  if(state==0&&data==0xAA)//֡ͷ1
  {
    state=1;
		cksum = data;
  }
  else if(state==1&&data==0xAF)//֡ͷ2
  {
    state=2;
    cksum += data;
  }
  else if(state==2&&data<0XF1)//�����ֽ�
  {
    state=3;
		rxPacket.msgID = data;
    cksum += data;
  }
  else if(state==3&&data<ATKP_MAX_DATA_SIZE)//��Ч���ݳ���
  {
    state = 4;
		rxPacket.dataLen = data;
    _data_cnt = 0;
		cksum += data;
  }
  else if(state==4&&rxPacket.dataLen>0)//���ݽ���
  {
    rxPacket.data[_data_cnt++]=data;
		cksum += data;
    if(_data_cnt==rxPacket.dataLen)
        state = 5;
  }
  else if(state==5)//У���
  {
    state = 0;
    if (cksum == data)	/*����У����ȷ*/
		{
			atkpReceivePacketBlocking(&rxPacket);
		} 
		else	/*У�����*/
       state = 0;	
  }
  else state = 0;
}




u8 position_control_flag = DOWN;

u8 MV_data[5];

void OPEN_MV_Data_Receive(u8 data)
{
  static u8 state = 0;
	
  if(state==0&&data==0xAA)//֡ͷ1
    state=1;

  else if(state==1)
  {
    state=2;
    MV_data[0]=data;
  }
  else if(state==2)
  {
    state=3;
		MV_data[1]=data;
  }
  else if(state==3)
  {
    state = 0;
		MV_data[2]=data;
		if(position_control_flag==DOWN)
		{
			MV_DotX=MV_data[0];
			MV_DotY=MV_data[1];
			MV_line_flag=MV_data[2];
		}
		
  }

		
  else state = 0;
	
}

u8 MV_LEFT_DATA[5];

void OPEN_MV_LEFT_Data_Receive(u8 data)
{
  static u8 state = 0;
	
  if(state==0&&data==0xAA)//֡ͷ1
    state=1;

  else if(state==1)
  {
    state=2;
    MV_LEFT_DATA[0]=data;
  }
  else if(state==2)
  {
    state=3;
		MV_LEFT_DATA[1]=data;
  }
  else if(state==3)
  {
    state = 4;
		MV_LEFT_DATA[2]=data;
  }
	else if(state==4)
  {
    state = 5;
		MV_LEFT_DATA[3]=data;
		
  }
	else if(state==5)
  {
    state = 0;
		MV_LEFT_DATA[4]=data;
		if(position_control_flag==LEFT)
		{
			MV_DotX=MV_LEFT_DATA[0]*256+MV_LEFT_DATA[1];
			MV_DotY=MV_LEFT_DATA[2]*256+MV_LEFT_DATA[3];
			MV_line_flag=MV_LEFT_DATA[4];
		}
		
  }
  else state = 0;
	
}


void UART0_IRQHandler(void)//UART0�жϺ���
{	
  //��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //����жϱ�־	
  UARTIntClear(UART0_BASE,flag);		
  //�ж�FIFO�Ƿ�������		
  while(UARTCharsAvail(UART0_BASE))		
  {			
			OPEN_MV_LEFT_Data_Receive(UARTCharGet(UART0_BASE));
  }
}



void ConfigureUART0(void)//����0��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//ʹ��UART����
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIOģʽ���� PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
  //UARTЭ������ ������115200 8λ 1ֹͣλ  ��У��λ	
  //UART����FIFO Ĭ��FIFO LevelΪ4/8 �Ĵ�����8�ֽں�����ж�	//���ú����1λ�Ͳ����ж�	
  UARTFIFODisable(UART0_BASE);//ʹ��UART0�ж�	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART0, USER_INT4);
}



void UART1_IRQHandler(void)//UART0�жϺ���
{
  //��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־		
  uint32_t flag = UARTIntStatus(UART1_BASE,1);
  //����жϱ�־	
  UARTIntClear(UART1_BASE,flag);		
  //�ж�FIFO�Ƿ�������		
  while(UARTCharsAvail(UART1_BASE))		
  {			
     ANO_DT_Data_Receive_Prepare(UARTCharGet(UART1_BASE));
  }
	  
	
}


void ConfigureUART1(void)//����1��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//ʹ��UART����
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIOģʽ���� PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(1, 460800, 16000000);
  UARTFIFODisable(UART1_BASE);//ʹ��UART1�ж�	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//ʹ��UART1�����ж�		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1�жϵ�ַע��	
  IntPrioritySet(INT_UART1, USER_INT2);
}

void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}


void UART3_IRQHandler(void)//UART3�жϺ���
{	
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART3_BASE,flag);//����жϱ�־		
  while(UARTCharsAvail(UART3_BASE))//�ж�FIFO�Ƿ�������			
  {			
    //����õ�������			
    //UARTCharPut(UART1_BASE,UARTCharGet(UART1_BASE));
    RingBuf_Write(UARTCharGet(UART3_BASE),&COM0_Rx_Buf,4);//�����ζ�������д����		
  }
}


void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}

void ConfigureUART3(void)//����3��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//ʹ��UART����
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIOģʽ���� PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//ʹ��UART3�����ж�		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3�жϵ�ַע��	
  IntPrioritySet(INT_UART3, USER_INT3);
}

void UART6_IRQHandler(void)//UART6�жϺ���
{	
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART6_BASE,flag);//����жϱ�־		
  while(UARTCharsAvail(UART6_BASE))//�ж�FIFO�Ƿ�������			
  {			
    //����õ�������			
    OPEN_MV_Data_Receive(UARTCharGet(UART6_BASE));	
  }
}



void ConfigureUART6(void)//����6��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//ʹ��UART����
  GPIOPinConfigure(GPIO_PD4_U6RX);//GPIOģʽ���� 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//ʹ��UART5�����ж�		
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART3�жϵ�ַע��	
  IntPrioritySet(INT_UART6, USER_INT4);
}




void UART7_IRQHandler(void)//UART0�жϺ���
{
  //��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);
  //����жϱ�־	
  UARTIntClear(UART7_BASE,flag);		
  //�ж�FIFO�Ƿ�������		
  while(UARTCharsAvail(UART7_BASE))		
  {			
     GetOneByte(UARTCharGet(UART7_BASE));
//		AnoOF_GetOneByte(UARTCharGet(UART7_BASE));
  }  
	
}


void ConfigureUART7(void)//����0��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//ʹ��UART����
  GPIOPinConfigure(GPIO_PE0_U7RX);//GPIOģʽ���� PE0--RX PE1--TX 
  GPIOPinConfigure(GPIO_PE1_U7TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART7_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART7_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART7, USER_INT3);
}


