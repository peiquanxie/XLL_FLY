#ifndef __CONFIG_H__
#define __CONFIG_H__




/*遥控器配置*/
#define RC_MIN			1000
#define RC_MID			1500
#define RC_MAX			2000

#define RC_COMMANDER_MINCHECK	1100 //遥杆命令最小检查值
#define RC_COMMANDER_MAXCHECK	1900 //遥杆命令最大检查值

#define MINTHROTTLE		1180 //解锁后怠速油门值
#define MAXTHROTTLE		1850 //控制时最大油门值	


///*电调协议配置（只能选一）*/
//#define USE_ESC_PROTOCOL_STANDARD  //标准PWM协议





#define  USER_INT0  0x00
#define  USER_INT1  0x20
#define  USER_INT2  0x40
#define  USER_INT3  0x60
#define  USER_INT4  0x80
#define  USER_INT5  0xA0
#define  USER_INT6  0xD0
#define  USER_INT7  0xE0


#define RATE_1_HZ		1
#define RATE_2_HZ		2
#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_1000_HZ / RATE_HZ)) == 0)
#define RATE_POSITION_CTR(RATE_HZ, TICK) ((TICK % (RATE_100_HZ / RATE_HZ)) == 0)




#endif


