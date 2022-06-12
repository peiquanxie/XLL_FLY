#ifndef __TIVA_FLASH_H__
#define __TIVA_FLASH_H__


#include "Headfile.h"

//FLASH起始地址
#define TIVA_FLASH_BASE      0x0000 	//TIVA FLASH的起始地址


void EEPROM_Init(void); 
void TIVAFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);		//从指定地址开始写入指定长度的数据
void TIVAFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		//从指定地址开始读出指定长度的数据


#endif

