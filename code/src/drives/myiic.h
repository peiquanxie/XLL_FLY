#ifndef __MYIIC_H
#define __MYIIC_H

#include "Headfile.h"

void IIC_Init(void);
void IIC_Send_Byte(u8 addr, u8 regAddr, u8 data);
u8 IIC_Read_Byte(u8 addr, u8 regAddr);
short int Double_ReadI2C(u8 SlaveAddress,u8 REG_Address);

#endif
















