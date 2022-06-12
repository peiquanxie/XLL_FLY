#ifndef __TIVA_FLASH_H__
#define __TIVA_FLASH_H__


#include "Headfile.h"

//FLASH��ʼ��ַ
#define TIVA_FLASH_BASE      0x0000 	//TIVA FLASH����ʼ��ַ


void EEPROM_Init(void); 
void TIVAFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);		//��ָ����ַ��ʼд��ָ�����ȵ�����
void TIVAFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		//��ָ����ַ��ʼ����ָ�����ȵ�����


#endif

