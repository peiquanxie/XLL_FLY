#include "Usart.h"
#include "atkp.h"
#include "open_mv.h"
#include "tof_height.h"
#include "ANO_optical.h"
#include "usart_task.h"



#define USBLINK_TX_QUEUE_SIZE 	30 /*接收队列个数*/


atkp_t rxPacket;


//串口循环队列缓冲数据定义
RingBuff_t COM0_Rx_Buf;

void ANO_DT_Data_Receive_Prepare(u8 data)//ANO地面站数据解析
{
  
	static u8 cksum = 0;
  static u8 _data_cnt = 0;
  static u8 state = 0;
	
  if(state==0&&data==0xAA)//帧头1
  {
    state=1;
		cksum = data;
  }
  else if(state==1&&data==0xAF)//帧头2
  {
    state=2;
    cksum += data;
  }
  else if(state==2&&data<0XF1)//功能字节
  {
    state=3;
		rxPacket.msgID = data;
    cksum += data;
  }
  else if(state==3&&data<ATKP_MAX_DATA_SIZE)//有效数据长度
  {
    state = 4;
		rxPacket.dataLen = data;
    _data_cnt = 0;
		cksum += data;
  }
  else if(state==4&&rxPacket.dataLen>0)//数据接收
  {
    rxPacket.data[_data_cnt++]=data;
		cksum += data;
    if(_data_cnt==rxPacket.dataLen)
        state = 5;
  }
  else if(state==5)//校验和
  {
    state = 0;
    if (cksum == data)	/*所有校验正确*/
		{
			atkpReceivePacketBlocking(&rxPacket);
		} 
		else	/*校验错误*/
       state = 0;	
  }
  else state = 0;
}




u8 position_control_flag = DOWN;

u8 MV_data[5];

void OPEN_MV_Data_Receive(u8 data)
{
  static u8 state = 0;
	
  if(state==0&&data==0xAA)//帧头1
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
	
  if(state==0&&data==0xAA)//帧头1
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


void UART0_IRQHandler(void)//UART0中断函数
{	
  //获取中断标志 原始中断状态 不屏蔽中断标志		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //清除中断标志	
  UARTIntClear(UART0_BASE,flag);		
  //判断FIFO是否还有数据		
  while(UARTCharsAvail(UART0_BASE))		
  {			
			OPEN_MV_LEFT_Data_Receive(UARTCharGet(UART0_BASE));
  }
}



void ConfigureUART0(void)//串口0初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//使能UART外设
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIO模式配置 PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000);
  //UART协议配置 波特率115200 8位 1停止位  无校验位	
  //UART禁用FIFO 默认FIFO Level为4/8 寄存器满8字节后产生中断	//禁用后接收1位就产生中断	
  UARTFIFODisable(UART0_BASE);//使能UART0中断	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//使能UART0接收中断		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART0, USER_INT4);
}



void UART1_IRQHandler(void)//UART0中断函数
{
  //获取中断标志 原始中断状态 不屏蔽中断标志		
  uint32_t flag = UARTIntStatus(UART1_BASE,1);
  //清除中断标志	
  UARTIntClear(UART1_BASE,flag);		
  //判断FIFO是否还有数据		
  while(UARTCharsAvail(UART1_BASE))		
  {			
     ANO_DT_Data_Receive_Prepare(UARTCharGet(UART1_BASE));
  }
	  
	
}


void ConfigureUART1(void)//串口1初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//使能UART外设
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIO模式配置 PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
  UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(1, 460800, 16000000);
  UARTFIFODisable(UART1_BASE);//使能UART1中断	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//使能UART1接收中断		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1中断地址注册	
  IntPrioritySet(INT_UART1, USER_INT2);
}

void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}


void UART3_IRQHandler(void)//UART3中断函数
{	
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART3_BASE,flag);//清除中断标志		
  while(UARTCharsAvail(UART3_BASE))//判断FIFO是否还有数据			
  {			
    //输出得到的数据			
    //UARTCharPut(UART1_BASE,UARTCharGet(UART1_BASE));
    RingBuf_Write(UARTCharGet(UART3_BASE),&COM0_Rx_Buf,4);//往环形队列里面写数据		
  }
}


void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//发送N个字节长度的数据
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}

void ConfigureUART3(void)//串口3初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//使能UART外设
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIO模式配置 PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 9600,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//使能UART0中断	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//使能UART3接收中断		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3中断地址注册	
  IntPrioritySet(INT_UART3, USER_INT3);
}

void UART6_IRQHandler(void)//UART6中断函数
{	
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志	
  UARTIntClear(UART6_BASE,flag);//清除中断标志		
  while(UARTCharsAvail(UART6_BASE))//判断FIFO是否还有数据			
  {			
    //输出得到的数据			
    OPEN_MV_Data_Receive(UARTCharGet(UART6_BASE));	
  }
}



void ConfigureUART6(void)//串口6初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//使能UART外设
  GPIOPinConfigure(GPIO_PD4_U6RX);//GPIO模式配置 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//使能UART0中断	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//使能UART5接收中断		
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART3中断地址注册	
  IntPrioritySet(INT_UART6, USER_INT4);
}




void UART7_IRQHandler(void)//UART0中断函数
{
  //获取中断标志 原始中断状态 不屏蔽中断标志		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);
  //清除中断标志	
  UARTIntClear(UART7_BASE,flag);		
  //判断FIFO是否还有数据		
  while(UARTCharsAvail(UART7_BASE))		
  {			
     GetOneByte(UARTCharGet(UART7_BASE));
//		AnoOF_GetOneByte(UARTCharGet(UART7_BASE));
  }  
	
}


void ConfigureUART7(void)//串口0初始化
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//使能GPIO外设		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//使能UART外设
  GPIOPinConfigure(GPIO_PE0_U7RX);//GPIO模式配置 PE0--RX PE1--TX 
  GPIOPinConfigure(GPIO_PE1_U7TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO的UART模式配置
  UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART7_BASE);//使能UART0中断	
  UARTIntEnable(UART7_BASE,UART_INT_RX);//使能UART0接收中断		
  UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART中断地址注册	
  IntPrioritySet(INT_UART7, USER_INT3);
}


