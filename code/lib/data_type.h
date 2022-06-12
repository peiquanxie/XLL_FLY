#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H


#include <string.h>
#include <stdint.h>
#include <stdbool.h>




//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )


/* IO definitions (access restrictions to peripheral registers) */
/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */

typedef   signed char      int8;
typedef unsigned char      u8;
typedef unsigned char      uint8;
typedef unsigned char      byte;
typedef   signed short int int16;
typedef unsigned short int uint16;
typedef unsigned short int u16;
typedef unsigned long  int u32; 

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */


typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */

#if defined(__CC_ARM) 
	#pragma anon_unions
#endif

typedef union 
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;   //三轴线数据结构体16位 Axis-轴线

typedef union 
{
	struct 
	{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	int32_t axis[3];
} Axis3i32;    //三轴线数据结构体32位 Axis-轴线

typedef union 
{
	struct 
	{
		int64_t x;
		int64_t y;
		int64_t z;
	};
	int64_t axis[3];
} Axis3i64;      //三轴线数据结构体64位 Axis-轴线

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;     //三轴线数据结构体浮点型 Axis-轴线



//姿态数据结构
typedef struct  
{
	u32 timestamp;	/*时间戳*/
	float roll;
	float pitch;
	float yaw;
} attitude_t;

struct  vec3_s 
{
	u32 timestamp;
	float x;
	float y;
	float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

//Z轴距离传感器数据结构
typedef struct zDistance_s 
{
	uint32_t timestamp;
	float distance;
} zDistance_t;

//光流数据结构
typedef struct flowMeasurement_s 
{
	uint32_t timestamp;
	union 
	{
		struct 
		{
			float dpixelx;  // Accumulated pixel count x
			float dpixely;  // Accumulated pixel count y
		};
		float dpixel[2];  // Accumulated pixel count
	};
	float stdDevX;      // Measurement standard deviation
	float stdDevY;      // Measurement standard deviation
	float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;

//TOF数据结构
typedef struct tofMeasurement_s 
{
	uint32_t timestamp;
	float distance;
	float stdDev;
} tofMeasurement_t;

//气压数据结构
typedef struct
{
	uint32_t timestamp;
	float pressure;
	float temperature;
	float asl;
} baro_t;

//超声波数据结构
typedef struct
{
	float distance_pre;
	float distance;
} US100_t;

////TOF数据结构
//typedef struct
//{
//	float distance_pre;
//	float distance;
//} TOF_t;

//所有传感器数据集合
typedef struct
{
	Axis3f acc;				//加速度（G）
	Axis3f gyro;			//陀螺仪（deg/s）
	Axis3f mag;				//磁力计（gauss）
	baro_t baro;	
	US100_t height;
} sensorData_t;

//四轴姿态数据结构
typedef struct
{
	attitude_t 	attitude;	//姿态角度（deg）
	point_t 	position;	//估算的位置（cm）
	velocity_t 	velocity;	//估算的速度（cm/s）
	acc_t acc;				//估算的加速度（cm/ss）
} state_t;


typedef enum
{
	modeDisable = 0,
	modeAbs,
	modeVelocity
} mode_e;

typedef struct
{
	mode_e x;
	mode_e y;
	mode_e z;
	mode_e roll;
	mode_e pitch;
	mode_e yaw;
}mode_t;

//目标姿态数据结构
typedef struct
{
	attitude_t attitude;		//目标姿态角度（deg） //结构体，内含三个值
	attitude_t attitudeRate;	//目标角速度（deg/s）  //结构体，内含三个值
	point_t position;         	//目标位置（cm）  //结构体，内含3个值
	velocity_t velocity;      	//目标速度（cm/s）
	mode_t mode;				
	float thrust;				
} setpoint_t;

//控制数据结构
typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float thrust;  //推力，油门
} control_t;


#endif

