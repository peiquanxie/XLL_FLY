#include "tiva_flash.h"

void EEPROM_Init(void)
{
  /* EEPROM SETTINGS */
  SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0); // EEPROM activate
  EEPROMInit(); // EEPROM start
}

void TIVAFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
  EEPROMProgram(pBuffer,WriteAddr,NumToWrite*4);
}

void TIVAFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
  EEPROMRead(pBuffer,ReadAddr,NumToRead*4);
}



