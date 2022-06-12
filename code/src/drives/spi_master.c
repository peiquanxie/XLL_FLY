#include "spi_master.h"

static bool isInit = false;

void SPI0_Init(void)
{
	
		if (isInit) return;
	
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);

    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 |
                   GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);  //1M

    SSIEnable(SSI0_BASE);
		
		isInit = true;
}



u8 SPI0TransferByte(u8 data)
{	
    uint16_t spiTimeout = 1000;
		uint32_t FIFO_DATA;

   while(SSIDataGetNonBlocking(SSI0_BASE, &FIFO_DATA))  //ȷ��������Ϊ��
	{
		if((spiTimeout--)==0)
			return false;
		}  
    SSIDataPut(SSI0_BASE, data); //ͨ������SPIx����һ��byte������

		spiTimeout = 1000;
   while(SSIBusy(SSI0_BASE))          //�ȴ��������
	{
		if ((spiTimeout--) == 0)
            return false;
		}
	
	SSIDataGet(SSI0_BASE, &FIFO_DATA);  //��������
  return ((u8)FIFO_DATA );
}

bool SPI0Transfer(uint8_t *out, const uint8_t *in, int len)
{
	uint16_t spiTimeout = 1000;
	uint32_t FIFO_DATA;

	while (len--) 
	{
			uint8_t b = in ? *(in++) : 0xFF;
			while(SSIDataGetNonBlocking(SSI0_BASE, &FIFO_DATA))  //ȷ��������Ϊ��
			{
			if((spiTimeout--)==0)
				return false;
			}  
			SSIDataPut(SSI0_BASE, b); //ͨ������SPIx����һ��byte������

			spiTimeout = 1000;
			while(SSIBusy(SSI0_BASE))          //�ȴ��������
			{
				if ((spiTimeout--) == 0)
            return false;
				}
			SSIDataGet(SSI0_BASE, &FIFO_DATA);  //��������
			b = ((u8)FIFO_DATA );
			if (out)
				*(out++) = b;
	}
	return true;
}


void SPI0SetSpeed(uint32_t speed)
{
	SSIDisable(SSI0_BASE);
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, speed, 8); 
	SSIEnable(SSI0_BASE);
}

void SPI1_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinConfigure(GPIO_PD0_SSI1CLK);
    GPIOPinConfigure(GPIO_PD2_SSI1RX);
    GPIOPinConfigure(GPIO_PD3_SSI1TX);

    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 |
                   GPIO_PIN_0);

    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);

    SSIEnable(SSI1_BASE);
}


//SPI ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{	
  uint32_t FIFO_DATA;	 
  while(SSIDataGetNonBlocking(SSI1_BASE, &FIFO_DATA));  //ȷ��������Ϊ��
	
  SSIDataPut(SSI1_BASE, TxData); //ͨ������SPIx����һ��byte������
	while(SSIBusy(SSI1_BASE));        //�ȴ��������
	
  SSIDataGet(SSI1_BASE, &FIFO_DATA);  //��������
	return ((u8)FIFO_DATA );//����ͨ��SPIx������յ�����			    
}
