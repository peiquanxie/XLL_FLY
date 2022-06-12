#include "myiic.h"


void IIC_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); // Enable I2C1 peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    // Use alternate function
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7); // Use pin with I2C peripheral

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(),true); // Enable and set frequency to 400 kHz

    SysCtlDelay(2); // Insert a few cycles after enabling the I2C to allow the clock to be fully activated
}


void IIC_Send_Byte(u8 addr, u8 regAddr, u8 data) 
{
    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode

    I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

    I2CMasterDataPut(I2C1_BASE, data); // Place data into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
}

 u8 IIC_Read_Byte(u8 addr, u8 regAddr) 
{
    I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode

    I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done

    I2CMasterSlaveAddrSet(I2C1_BASE, addr, true); // Set to read mode

    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    return I2CMasterDataGet(I2C1_BASE); // Read data
}

short int Double_ReadI2C(u8 SlaveAddress,u8 REG_Address)
{
  u8 msb , lsb ;
  msb = IIC_Read_Byte(SlaveAddress,REG_Address);
  lsb = IIC_Read_Byte(SlaveAddress,REG_Address+1);
  return ( ((short int)msb) << 8 | lsb) ;
}


