#ifndef __ATKP_H
#define __ATKP_H

#include "Headfile.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ATKflight飞控固件
 * 飞控通讯协议格式代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.0
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 * 说明：本格式部分基于匿名上位机通讯协议编写
********************************************************************************/

			
/*上行帧头*/
#define UP_BYTE1 0xAA
#define UP_BYTE2 0xAA

/*下行帧头*/
#define DOWN_BYTE1 0xAA
#define DOWN_BYTE2 0xAF

#define ATKP_MAX_DATA_SIZE 30

/*通讯数据结构*/
typedef struct
{
	uint8_t msgID;
	uint8_t dataLen;
	uint8_t data[ATKP_MAX_DATA_SIZE];
}atkp_t;

/*上行指令ID*/
typedef enum 
{
	UP_VERSION		= 0x00,
	UP_STATUS		= 0x01,
	UP_SENSER		= 0x02,
	UP_RCDATA		= 0x03,
	UP_GPSDATA		= 0x04,
	UP_POWER		= 0x05,
	UP_MOTOR		= 0x06,
	UP_SENSER2		= 0x07,
	UP_FLYMODE		= 0x0A,
	UP_SPEED 		= 0x0B,
	UP_PID1			= 0x10,
	UP_PID2			= 0x11,
	UP_PID3			= 0x12,
	UP_PID4			= 0x13,
	UP_PID5			= 0x14,
	UP_PID6			= 0x15,
	UP_RADIO		= 0x40,
	UP_MSG			= 0xEE,
	UP_CHECK		= 0xEF,
	
	UP_USER     =0xF1,
	
	UP_REMOTER		= 0x50,
	UP_PRINTF		= 0x51,
}upmsgID_e;


/*下行指令*/
#define  D_COMMAND_ACC_CALIB		0x01
#define  D_COMMAND_GYRO_CALIB		0x02
#define  D_COMMAND_MAG_CALIB		0x04
#define  D_COMMAND_BARO_CALIB		0x05
#define  D_COMMAND_ACC_CALIB_EXIT	0x20
#define  D_COMMAND_ACC_CALIB_STEP1	0x21
#define  D_COMMAND_ACC_CALIB_STEP2	0x22
#define  D_COMMAND_ACC_CALIB_STEP3	0x23
#define  D_COMMAND_ACC_CALIB_STEP4	0x24
#define  D_COMMAND_ACC_CALIB_STEP5	0x25
#define  D_COMMAND_ACC_CALIB_STEP6	0x26
#define  D_COMMAND_FLIGHT_LOCK		0xA0
#define  D_COMMAND_FLIGHT_ULOCK		0xA1

#define  D_ACK_READ_PID				0x01
#define  D_ACK_READ_VERSION			0xA0
#define  D_ACK_RESET_PARAM			0xA1
/*下行指令ID*/
typedef enum 
{
	DOWN_COMMAND	= 0x01,
	DOWN_ACK		= 0x02,
	DOWN_RCDATA		= 0x03,
	DOWN_POWER		= 0x05,
	DOWN_FLYMODE	= 0x0A,
	DOWN_PID1		= 0x10,
	DOWN_PID2		= 0x11,
	DOWN_PID3		= 0x12,
	DOWN_PID4		= 0x13,
	DOWN_PID5		= 0x14,
	DOWN_PID6		= 0x15,
	DOWN_RADIO		= 0x40,
	
	DOWN_REMOTER	= 0x50,
}downmsgID_e;


void sendStatus(float roll, float pitch, float yaw, s32 alt, u8 fly_model, u8 armed); //alt单位为cm
void sendSenser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void sendRCData(u16 thrust,u16 yaw,u16 roll,u16 pitch,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void sendPower(u16 votage, u16 current);
void sendMotorPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void sendSenser2(s32 bar_alt,float csb_alt);//bar_alt csb_alt单位为cm;
void senduserdata(u16 pitch);//bar_alt csb_alt单位为cm
void sendPid(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void sendCheck(u8 head, u8 check_sum);

void atkpReceiveAnl(atkp_t *anlPacket);
#endif /*ATKP_H*/

