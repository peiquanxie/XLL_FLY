#include "atkp.h"
#include "gyro.h"
#include "axis.h"
#include "state_control.h"
#include "accelerometer.h"
#include "config_param.h"
#include "runtime_config.h"


#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"




void usart0_niming_report(atkp_t *p)
{
	u8 send_buf[ATKP_MAX_DATA_SIZE+5];  //帧头+帧头+功能字+数据长度+ ATKP_MAX_DATA_SIZE+校验
	u8 i;
	if(p->dataLen>ATKP_MAX_DATA_SIZE)return;	//最多ATKP_MAX_DATA_SIZE字节数据 
	send_buf[p->dataLen+4]=0;	//校验数置零
	send_buf[0]=0XAA;	//帧头
	send_buf[1]=0XAA;	//帧头
	send_buf[2]=p->msgID;	//功能字
	send_buf[3]=p->dataLen;	//数据长度
	for(i=0;i<p->dataLen;i++)send_buf[4+i]=p->data[i];			//复制数据
	for(i=0;i<p->dataLen+4;i++)send_buf[p->dataLen+4]+=send_buf[i];	//计算校验和	
	for(i=0;i<p->dataLen+5;i++)UARTCharPut(UART1_BASE,send_buf[i]);	//发送数据到串口1 
}




/***************************发送至匿名上位机指令******************************/
 void sendStatus(float roll, float pitch, float yaw, s32 alt, u8 fly_model, u8 armed) //alt单位为cm
{
  u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	p.msgID = UP_STATUS;
	
	_temp = (int)(roll*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(pitch*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = (int)(yaw*100);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.data[_cnt++]=BYTE3(_temp2);
	p.data[_cnt++]=BYTE2(_temp2);
	p.data[_cnt++]=BYTE1(_temp2);
	p.data[_cnt++]=BYTE0(_temp2);
	
	p.data[_cnt++] = fly_model;
	p.data[_cnt++] = armed;
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}


void sendSenser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = UP_SENSER;

	_temp = a_x;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = 0;	
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);	
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

void sendRCData(u16 thrust,u16 yaw,u16 roll,u16 pitch,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_RCDATA;
	p.data[_cnt++]=BYTE1(thrust);
	p.data[_cnt++]=BYTE0(thrust);
	p.data[_cnt++]=BYTE1(yaw);
	p.data[_cnt++]=BYTE0(yaw);
	p.data[_cnt++]=BYTE1(roll);
	p.data[_cnt++]=BYTE0(roll);
	p.data[_cnt++]=BYTE1(pitch);
	p.data[_cnt++]=BYTE0(pitch);
	p.data[_cnt++]=BYTE1(aux1);
	p.data[_cnt++]=BYTE0(aux1);
	p.data[_cnt++]=BYTE1(aux2);
	p.data[_cnt++]=BYTE0(aux2);
	p.data[_cnt++]=BYTE1(aux3);
	p.data[_cnt++]=BYTE0(aux3);
	p.data[_cnt++]=BYTE1(aux4);
	p.data[_cnt++]=BYTE0(aux4);
	p.data[_cnt++]=BYTE1(aux5);
	p.data[_cnt++]=BYTE0(aux5);
	p.data[_cnt++]=BYTE1(aux6);
	p.data[_cnt++]=BYTE0(aux6);

	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

void sendPower(u16 votage, u16 current)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_POWER;
	
	p.data[_cnt++]=BYTE1(votage);
	p.data[_cnt++]=BYTE0(votage);
	p.data[_cnt++]=BYTE1(current);
	p.data[_cnt++]=BYTE0(current);
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

void sendMotorPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_MOTOR;
	
	p.data[_cnt++]=BYTE1(m_1);
	p.data[_cnt++]=BYTE0(m_1);
	p.data[_cnt++]=BYTE1(m_2);
	p.data[_cnt++]=BYTE0(m_2);
	p.data[_cnt++]=BYTE1(m_3);
	p.data[_cnt++]=BYTE0(m_3);
	p.data[_cnt++]=BYTE1(m_4);
	p.data[_cnt++]=BYTE0(m_4);
	p.data[_cnt++]=BYTE1(m_5);
	p.data[_cnt++]=BYTE0(m_5);
	p.data[_cnt++]=BYTE1(m_6);
	p.data[_cnt++]=BYTE0(m_6);
	p.data[_cnt++]=BYTE1(m_7);
	p.data[_cnt++]=BYTE0(m_7);
	p.data[_cnt++]=BYTE1(m_8);
	p.data[_cnt++]=BYTE0(m_8);
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

void sendSenser2(s32 bar_alt,float csb_alt)//bar_alt csb_alt单位为cm
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	p.msgID = UP_SENSER2;
	
	

	p.data[_cnt++]=BYTE3(bar_alt);
	p.data[_cnt++]=BYTE2(bar_alt);
	p.data[_cnt++]=BYTE1(bar_alt);
	p.data[_cnt++]=BYTE0(bar_alt);
	
	_temp = (int)(csb_alt);
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

//显示用户波形
void senduserdata(u16 pitch)//bar_alt csb_alt单位为cm
{
	u8 _cnt=0;
	atkp_t p;
	
	p.msgID = UP_USER;
	
	p.data[_cnt++]=BYTE1(pitch);
	p.data[_cnt++]=BYTE0(pitch);
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

void sendPid(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	atkp_t p;
	vs16 _temp;
	
	p.msgID = 0x10+group-1;

	_temp = p1_p;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_i;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p1_d;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_p;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_i;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p2_d;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_p;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_i;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	_temp = p3_d;
	p.data[_cnt++]=BYTE1(_temp);
	p.data[_cnt++]=BYTE0(_temp);
	
	p.dataLen = _cnt;
	usart0_niming_report(&p);
}

void sendCheck(u8 head, u8 check_sum)
{
	atkp_t p;
	
	p.msgID = UP_CHECK;
	p.dataLen = 2;
	p.data[0] = head;
	p.data[1] = check_sum;
	usart0_niming_report(&p);
}


static u8 atkpCheckSum(atkp_t *packet)
{
	u8 sum;
	sum = DOWN_BYTE1;
	sum += DOWN_BYTE2;
	sum += packet->msgID;
	sum += packet->dataLen;
	for(int i=0; i<packet->dataLen; i++)
	{
		sum += packet->data[i];
	}
	return sum;
}

void atkpReceiveAnl(atkp_t *anlPacket)
{
	if(anlPacket->msgID	== DOWN_COMMAND)
	{
		switch(anlPacket->data[0])
		{
			case D_COMMAND_ACC_CALIB:
				break;
				
			case D_COMMAND_GYRO_CALIB:
				gyroSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			
			case D_COMMAND_MAG_CALIB:
//				compassSetCalibrationStart();
				break;
			
			case D_COMMAND_BARO_CALIB:
				break;
			
			case D_COMMAND_ACC_CALIB_STEP1:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP2:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP3:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP4:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP5:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
			case D_COMMAND_ACC_CALIB_STEP6:
				accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
				break;
		}
	}			
	else if(anlPacket->msgID == DOWN_ACK)
	{
		if(anlPacket->data[0] == D_ACK_READ_PID)/*读取PID参数*/
		{
			sendPid(1, configParam.pid[RATE_ROLL].kp,   configParam.pid[RATE_ROLL].ki,   configParam.pid[RATE_ROLL].kd,
					   configParam.pid[RATE_PITCH].kp,  configParam.pid[RATE_PITCH].ki,  configParam.pid[RATE_PITCH].kd,
					   configParam.pid[RATE_YAW].kp,    configParam.pid[RATE_YAW].ki,    configParam.pid[RATE_YAW].kd 
				   );
			sendPid(2, configParam.pid[ANGLE_ROLL].kp,  configParam.pid[ANGLE_ROLL].ki,  configParam.pid[ANGLE_ROLL].kd,
					   configParam.pid[ANGLE_PITCH].kp, configParam.pid[ANGLE_PITCH].ki, configParam.pid[ANGLE_PITCH].kd,
					   configParam.pid[ANGLE_YAW].kp,   configParam.pid[ANGLE_YAW].ki,   configParam.pid[ANGLE_YAW].kd 
				   );
			sendPid(3, configParam.pid[VELOCITY_Z].kp,  configParam.pid[VELOCITY_Z].ki,  configParam.pid[VELOCITY_Z].kd,
					   configParam.pid[POSHOLD_Z].kp,   configParam.pid[POSHOLD_Z].ki,   configParam.pid[POSHOLD_Z].kd,
					   configParam.pid[VELOCITY_XY].kp, configParam.pid[VELOCITY_XY].ki, configParam.pid[VELOCITY_XY].kd
				   );
			sendPid(4, configParam.pid[POSHOLD_XY].kp,  configParam.pid[POSHOLD_XY].ki,  configParam.pid[POSHOLD_XY].kd,
					   configParam.pid[VELOCITY_LAND].kp, configParam.pid[VELOCITY_LAND].ki,  configParam.pid[VELOCITY_LAND].kd,
					   0, 0, 0
				   );
			sendPid(5, configParam.accBias.accZero[X] + 1000,  configParam.accBias.accZero[Y] + 1000,  configParam.accBias.accZero[Z] + 1000,//上位机不能显示负数，统一加1000
					   configParam.accBias.accGain[X],  configParam.accBias.accGain[Y],  configParam.accBias.accGain[Z],
					   configParam.magBias.magZero[X] + 1000,  configParam.magBias.magZero[Y] + 1000,  configParam.magBias.magZero[Z] + 1000
				   );
	
			
			bool pidBypassFlag = ARMING_FLAG(ARMING_DISABLED_PID_BYPASS); 
			sendPid(6, 0, 0, 0,
					   pidBypassFlag, 0, 0,
					   0, 0, 0
				   );
		}
		
		if(anlPacket->data[0] == D_ACK_RESET_PARAM)/*恢复默认参数*/
		{
			resetConfigParam();
			stateControlInit();
			
			sendPid(1, configParam.pid[RATE_ROLL].kp,   configParam.pid[RATE_ROLL].ki,   configParam.pid[RATE_ROLL].kd,
					   configParam.pid[RATE_PITCH].kp,  configParam.pid[RATE_PITCH].ki,  configParam.pid[RATE_PITCH].kd,
					   configParam.pid[RATE_YAW].kp,    configParam.pid[RATE_YAW].ki,    configParam.pid[RATE_YAW].kd 
				   );
			sendPid(2, configParam.pid[ANGLE_ROLL].kp,  configParam.pid[ANGLE_ROLL].ki,  configParam.pid[ANGLE_ROLL].kd,
					   configParam.pid[ANGLE_PITCH].kp, configParam.pid[ANGLE_PITCH].ki, configParam.pid[ANGLE_PITCH].kd,
					   configParam.pid[ANGLE_YAW].kp,   configParam.pid[ANGLE_YAW].ki,   configParam.pid[ANGLE_YAW].kd 
				   );
			sendPid(3, configParam.pid[VELOCITY_Z].kp,  configParam.pid[VELOCITY_Z].ki,  configParam.pid[VELOCITY_Z].kd,
					   configParam.pid[POSHOLD_Z].kp,   configParam.pid[POSHOLD_Z].ki,   configParam.pid[POSHOLD_Z].kd,
					   configParam.pid[VELOCITY_XY].kp, configParam.pid[VELOCITY_XY].ki, configParam.pid[VELOCITY_XY].kd
				   );
			sendPid(4, configParam.pid[POSHOLD_XY].kp,  configParam.pid[POSHOLD_XY].ki,  configParam.pid[POSHOLD_XY].kd,
					   configParam.pid[VELOCITY_LAND].kp, configParam.pid[VELOCITY_LAND].ki,  configParam.pid[VELOCITY_LAND].kd,
					   0, 0, 0
				   );
			sendPid(5, configParam.accBias.accZero[X] + 1000,  configParam.accBias.accZero[Y] + 1000,  configParam.accBias.accZero[Z] + 1000,//上位机不能显示负数，统一加1000
					   configParam.accBias.accGain[X],  configParam.accBias.accGain[Y],  configParam.accBias.accGain[Z],
					   configParam.magBias.magZero[X] + 1000,  configParam.magBias.magZero[Y] + 1000,  configParam.magBias.magZero[Z] + 1000
				   );
		}
	}
	else if(anlPacket->msgID == DOWN_PID1)
	{
		configParam.pid[RATE_ROLL].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[RATE_ROLL].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[RATE_ROLL].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[RATE_PITCH].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[RATE_PITCH].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[RATE_PITCH].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		configParam.pid[RATE_YAW].kp = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		configParam.pid[RATE_YAW].ki = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		configParam.pid[RATE_YAW].kd = ((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID2)
	{
		configParam.pid[ANGLE_ROLL].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[ANGLE_ROLL].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[ANGLE_ROLL].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[ANGLE_PITCH].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[ANGLE_PITCH].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[ANGLE_PITCH].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		configParam.pid[ANGLE_YAW].kp = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		configParam.pid[ANGLE_YAW].ki = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		configParam.pid[ANGLE_YAW].kd = ((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}		
	else if(anlPacket->msgID == DOWN_PID3)
	{
		configParam.pid[VELOCITY_Z].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[VELOCITY_Z].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[VELOCITY_Z].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[POSHOLD_Z].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[POSHOLD_Z].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[POSHOLD_Z].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
		
		configParam.pid[VELOCITY_XY].kp = ((s16)(*(anlPacket->data+12)<<8)|*(anlPacket->data+13));
		configParam.pid[VELOCITY_XY].ki = ((s16)(*(anlPacket->data+14)<<8)|*(anlPacket->data+15));
		configParam.pid[VELOCITY_XY].kd = ((s16)(*(anlPacket->data+16)<<8)|*(anlPacket->data+17));
			
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID4)
	{
		configParam.pid[POSHOLD_XY].kp = ((s16)(*(anlPacket->data+0)<<8)|*(anlPacket->data+1));
		configParam.pid[POSHOLD_XY].ki = ((s16)(*(anlPacket->data+2)<<8)|*(anlPacket->data+3));
		configParam.pid[POSHOLD_XY].kd = ((s16)(*(anlPacket->data+4)<<8)|*(anlPacket->data+5));
		
		configParam.pid[VELOCITY_LAND].kp = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		configParam.pid[VELOCITY_LAND].ki = ((s16)(*(anlPacket->data+8)<<8)|*(anlPacket->data+9));
		configParam.pid[VELOCITY_LAND].kd = ((s16)(*(anlPacket->data+10)<<8)|*(anlPacket->data+11));
			
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID5)
	{
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
	}
	else if(anlPacket->msgID == DOWN_PID6)//用于电机测试
	{
		bool pidBypassFlag = ((s16)(*(anlPacket->data+6)<<8)|*(anlPacket->data+7));
		
		if (pidBypassFlag)         // && (getBatteryState()==BATTERY_NOT_PRESENT)
			ENABLE_ARMING_FLAG(ARMING_DISABLED_PID_BYPASS);
		else
			DISABLE_ARMING_FLAG(ARMING_DISABLED_PID_BYPASS);
		
		u8 cksum = atkpCheckSum(anlPacket);
		sendCheck(anlPacket->msgID,cksum);
		
		stateControlInit();//初始化PID
		saveConfigAndNotify();//保存PID至Flash
	}
}





