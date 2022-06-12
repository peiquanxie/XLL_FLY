#include "mpu6500.h"
#include "delay.h"
#include "spi_master.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"


// Bits
#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1


#define ACC_GYRO_RAWDATA_LEN	14

#define DISABLE_MPU6500()	  GPIOPinWrite(CS0_BASE,SPI0_CS,SPI0_CS);
#define ENABLE_MPU6500()   	GPIOPinWrite(CS0_BASE,SPI0_CS,0);


static bool isInit = false;   

bool mpu6500SpiWriteRegister(uint8_t reg, uint8_t data)   //����ֵΪ1��0
{
    ENABLE_MPU6500();
	vTaskDelay(1);
	SPI0TransferByte(reg);
	SPI0TransferByte(data);
    DISABLE_MPU6500();
	vTaskDelay(1);
	return true;
}

bool mpu6500SpiReadRegister(uint8_t reg, uint8_t length, uint8_t *data)
{
	 ENABLE_MPU6500();
	SPI0TransferByte(reg | 0x80); // read transaction
  SPI0Transfer(data, NULL, length);
	 DISABLE_MPU6500();
	return true;
}

bool mpu6500Init(void)
{
	if (isInit) return true; //�ж��Ƿ��Ѿ���ʼ����
	
	//SPI0��ʼ��
	SPI0_Init();  //Ӳ��SPI
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CS0);//��ʼ������GPIO
    GPIOPadConfigSet(CS0_BASE,
                     SPI0_CS,GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD);//����PF4Ϊ2MA,����������,
    GPIODirModeSet(CS0_BASE, SPI0_CS, GPIO_DIR_MODE_OUT);//����GPIO���ģʽ

    GPIOPinWrite(CS0_BASE, SPI0_CS ,SPI0_CS);
	
	//��λMPU6500
	mpu6500SpiWriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);
	vTaskDelay(50);
	mpu6500SpiWriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	vTaskDelay(50);
	mpu6500SpiWriteRegister(MPU_RA_PWR_MGMT_1, BIT_H_RESET);//��λ�������Ӵ������ȶ���
	vTaskDelay(50);
	mpu6500SpiWriteRegister(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
	vTaskDelay(50);
	
	//��ȡID
	u8 id = 0x00;
	mpu6500SpiReadRegister(MPU_RA_WHO_AM_I, 1, &id);
	
	//��ȡ��������ʼ��
	if(id == MPU6500_WHO_AM_I_CONST)
	{
		//����X��������Ϊʱ�� 
		mpu6500SpiWriteRegister(MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROX);
		vTaskDelay(15);
		
		//��ֹI2C�ӿ�
		mpu6500SpiWriteRegister(MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
		vTaskDelay(15);
		mpu6500SpiWriteRegister(MPU_RA_PWR_MGMT_2, 0x00);
		vTaskDelay(15);
		
		// Accel Sample Rate 1kHz
		// Gyroscope Output Rate =  1kHz when the DLPF is enabled
		mpu6500SpiWriteRegister(MPU_RA_SMPLRT_DIV, 0);//���ò�����1KHZ
		vTaskDelay(15);
		
		//���������� +/- 2000 DPS����
		mpu6500SpiWriteRegister(MPU_RA_GYRO_CONFIG, FSR_2000DPS << 3);
		vTaskDelay(15);
		
		//���ü��ٶ� +/- 8 G ����
		mpu6500SpiWriteRegister(MPU_RA_ACCEL_CONFIG, FSR_8G << 3);
		vTaskDelay(15);
		
		//�����ж����Ź���
		mpu6500SpiWriteRegister(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);//�ж���������
		vTaskDelay(15);
		
		//���õ�ͨ�˲�����
		mpu6500SpiWriteRegister(MPU_RA_CONFIG, BITS_DLPF_CFG_98HZ);
		vTaskDelay(1);
		
		isInit = true;
	}
	
    SPI0SetSpeed(5000000);//����SPIΪ����ģʽ

	return isInit;
}

bool mpu6500GyroRead(Axis3i16* gyroRaw)
{
	if(!isInit) 
		return false;
	u8 buffer[6];
	mpu6500SpiReadRegister(MPU_RA_GYRO_XOUT_H, 6, buffer);
	gyroRaw->x = (((int16_t) buffer[0]) << 8) | buffer[1];
	gyroRaw->y = (((int16_t) buffer[2]) << 8) | buffer[3];
	gyroRaw->z = (((int16_t) buffer[4]) << 8) | buffer[5];
	return true;
}

bool mpu6500AccRead(Axis3i16* accRaw)
{
	if(!isInit) 
		return false;
	u8 buffer[6];
	mpu6500SpiReadRegister(MPU_RA_ACCEL_XOUT_H, 6, buffer);
	accRaw->x = (((int16_t) buffer[0]) << 8) | buffer[1];
	accRaw->y = (((int16_t) buffer[2]) << 8) | buffer[3];
	accRaw->z = (((int16_t) buffer[4]) << 8) | buffer[5];
	return true;
}

