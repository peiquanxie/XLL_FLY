#ifndef __USART_H__
#define __USART_H__

#include "Headfile.h"
#include "user_Ringbuf.h"

#define DOWN 1
#define LEFT 2

extern u16 distance;
extern u8 position_control_flag;

void ConfigureUART1(void);
void ConfigureUART0(void);
//void ConfigureUART2(unsigned long bound);
void ConfigureUART3(void);
void ConfigureUART6(void);//´®¿Ú6³õÊ¼»¯
//void ConfigureUART6(void);
void ConfigureUART7(void);
//void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
//void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
//void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
//void wust_sendware(unsigned char *wareaddr, int16_t waresize);
void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
void USART5_Send(uint8_t *pui8Buffer, uint32_t ui32Count);
//void ANO_DT_Data_Receive_Prepare(u8 data);
//void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
//void ANO_SEND_StateMachine(void);
//void Vcan_Send(void);
//extern uint8_t ANO_Send_PID_Flag[6];
extern RingBuff_t COM0_Rx_Buf;
#endif

