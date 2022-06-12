#ifndef __SPI_MASTER_H__
#define __SPI_MASTER_H__

#include "Headfile.h"


void SPI0_Init(void);
u8 SPI0TransferByte(u8 data);
bool SPI0Transfer(uint8_t *out, const uint8_t *in, int len);
void SPI0SetSpeed(uint32_t speed);
void SPI1_Init(void);
u8 SPI1_ReadWriteByte(u8 TxData);


#endif

