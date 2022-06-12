#include "pmw3901.h"
#include "delay.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

static bool isInit = false;

#define NCS_PIN_High()  GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1)
#define NCS_PIN_Low()   GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,0)
#define OPTICAL_POWER_ON() GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_2,GPIO_PIN_2)
#define OPTICAL_POWER_OFF() GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_2,0)



//������Դ����
void opticalFlowPowerControl(bool state)
{
	if(state == true)
		OPTICAL_POWER_ON();
	else
		OPTICAL_POWER_OFF();
}

//д�Ĵ���
static void registerWrite(uint8_t reg, uint8_t value)
{
	// ���λΪ1 д�Ĵ���
	reg |= 0x80u;
	
	NCS_PIN_Low();
	
	delay_us(50);
	SPI1_ReadWriteByte(reg);
	delay_us(50);
	SPI1_ReadWriteByte(value);
	delay_us(50);

	NCS_PIN_High();

	delay_us(200);
}

//���Ĵ���
static uint8_t registerRead(uint8_t reg)
{
	uint8_t data = 0;

	// ���λΪ0 ���Ĵ���
	reg &= ~0x80u;

	NCS_PIN_Low();
	
 	delay_us(50);
  SPI1_ReadWriteByte(reg);
	delay_us(500);
	data=SPI1_ReadWriteByte(data);	
	delay_us(50);
	
	NCS_PIN_High();
	
	delay_us(200);

	return data;
}


void SPI_Get_Motion(size_t length, uint8_t * data_rx)
{
	u8 i;
	for(i=0;i<length;i++)
	{
			*data_rx=SPI1_ReadWriteByte(0xA5);
			data_rx++;
	}
}

//�ӹ�����ȡһ֡����
void readMotion(motionBurst_t * motion)
{
	uint8_t address = 0x16;
	
	NCS_PIN_Low();
	
	delay_us(50);	
	SPI1_ReadWriteByte(address);
	delay_us(50);
	SPI_Get_Motion(sizeof(motionBurst_t), (uint8_t*)motion);
	delay_us(50);

	NCS_PIN_High();

//	delay_us(50);

}

static void InitRegisters(void)
{	
	registerWrite(0x7F, 0x00);
	registerWrite(0x61, 0xAD);
	registerWrite(0x7F, 0x03);
	registerWrite(0x40, 0x00);
	registerWrite(0x7F, 0x05);
	registerWrite(0x41, 0xB3);
	registerWrite(0x43, 0xF1);
	registerWrite(0x45, 0x14);
	registerWrite(0x5B, 0x32);
	registerWrite(0x5F, 0x34);
	registerWrite(0x7B, 0x08);
	registerWrite(0x7F, 0x06);
	registerWrite(0x44, 0x1B);
	registerWrite(0x40, 0xBF);
	registerWrite(0x4E, 0x3F);
	registerWrite(0x7F, 0x08);
	registerWrite(0x65, 0x20);
	registerWrite(0x6A, 0x18);
	registerWrite(0x7F, 0x09);
	registerWrite(0x4F, 0xAF);
	registerWrite(0x5F, 0x40);
	registerWrite(0x48, 0x80);
	registerWrite(0x49, 0x80);
	registerWrite(0x57, 0x77);
	registerWrite(0x60, 0x78);
	registerWrite(0x61, 0x78);
	registerWrite(0x62, 0x08);
	registerWrite(0x63, 0x50);
	registerWrite(0x7F, 0x0A);
	registerWrite(0x45, 0x60);
	registerWrite(0x7F, 0x00);
	registerWrite(0x4D, 0x11);
	registerWrite(0x55, 0x80);
	registerWrite(0x74, 0x1F);
	registerWrite(0x75, 0x1F);
	registerWrite(0x4A, 0x78);
	registerWrite(0x4B, 0x78);
	registerWrite(0x44, 0x08);
	registerWrite(0x45, 0x50);
	registerWrite(0x64, 0xFF);
	registerWrite(0x65, 0x1F);
	registerWrite(0x7F, 0x14);
	registerWrite(0x65, 0x67);
	registerWrite(0x66, 0x08);
	registerWrite(0x63, 0x70);
	registerWrite(0x7F, 0x15);
	registerWrite(0x48, 0x48);
	registerWrite(0x7F, 0x07);
	registerWrite(0x41, 0x0D);
	registerWrite(0x43, 0x14);
	registerWrite(0x4B, 0x0E);
	registerWrite(0x45, 0x0F);
	registerWrite(0x44, 0x42);
	registerWrite(0x4C, 0x80);
	registerWrite(0x7F, 0x10);
	registerWrite(0x5B, 0x02);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x41);
	registerWrite(0x70, 0x00);

	vTaskDelay(10); // delay 10ms

	registerWrite(0x32, 0x44);
	registerWrite(0x7F, 0x07);
	registerWrite(0x40, 0x40);
	registerWrite(0x7F, 0x06);
	registerWrite(0x62, 0xF0);
	registerWrite(0x63, 0x00);
	registerWrite(0x7F, 0x0D);
	registerWrite(0x48, 0xC0);
	registerWrite(0x6F, 0xD5);
	registerWrite(0x7F, 0x00);
	registerWrite(0x5B, 0xA0);
	registerWrite(0x4E, 0xA8);
	registerWrite(0x5A, 0x50);
	registerWrite(0x40, 0x80);
	

}



/*��ʼ������ģ��*/
//NCS_PIN		PD1
//OPTICAL_POWER_ENABLE	PC2
void opticalFlowInit(void)
{
	if (!isInit) 
	{
		//��ʼ��PD1,��ΪSPIƬѡ�ź�CS
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//��ʼ������GPIO
    GPIOPadConfigSet(GPIO_PORTD_BASE,
                     GPIO_PIN_1,GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD);//����Ϊ2MA,����������,
    GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_1,GPIO_DIR_MODE_OUT);//����GPIO���ģʽ
    
		//��ʼ��PC2����Ϊ����ģ���Դ���ƶ˿�
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//��ʼ������GPIO
    GPIOPadConfigSet(GPIO_PORTC_BASE,
                     GPIO_PIN_2,GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD);//����Ϊ2MA,����������,
    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_2,GPIO_DIR_MODE_OUT);//����GPIO���ģʽ	
		
	}
	
	opticalFlowPowerControl(true);	/*�򿪵�Դ*/
	vTaskDelay(50);
	
	NCS_PIN_High();  //CS�ź�����
	SPI1_Init();
	vTaskDelay(40);

	uint8_t chipId = registerRead(0x00);   //����0x49
	uint8_t invChipId = registerRead(0x5f);  //����0xB6

	// �ϵ縴λ
	registerWrite(0x3a, 0x5a);
	vTaskDelay(5);

	InitRegisters();
	vTaskDelay(5);
	

	isInit = true;
}

	
	

