#ifndef __DATA_TYPE_H
#define __DATA_TYPE_H


#include <string.h>
#include <stdint.h>
#include <stdbool.h>




//���ݲ�ֺ궨��
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
} Axis3i16;   //���������ݽṹ��16λ Axis-����

typedef union 
{
	struct 
	{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	int32_t axis[3];
} Axis3i32;    //���������ݽṹ��32λ Axis-����

typedef union 
{
	struct 
	{
		int64_t x;
		int64_t y;
		int64_t z;
	};
	int64_t axis[3];
} Axis3i64;      //���������ݽṹ��64λ Axis-����

typedef union 
{
	struct 
	{
		float x;
		float y;
		float z;
	};
	float axis[3];
} Axis3f;     //���������ݽṹ�帡���� Axis-����



//��̬���ݽṹ
typedef struct  
{
	u32 timestamp;	/*ʱ���*/
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

//Z����봫�������ݽṹ
typedef struct zDistance_s 
{
	uint32_t timestamp;
	float distance;
} zDistance_t;

//�������ݽṹ
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

//TOF���ݽṹ
typedef struct tofMeasurement_s 
{
	uint32_t timestamp;
	float distance;
	float stdDev;
} tofMeasurement_t;

//��ѹ���ݽṹ
typedef struct
{
	uint32_t timestamp;
	float pressure;
	float temperature;
	float asl;
} baro_t;

//���������ݽṹ
typedef struct
{
	float distance_pre;
	float distance;
} US100_t;

////TOF���ݽṹ
//typedef struct
//{
//	float distance_pre;
//	float distance;
//} TOF_t;

//���д��������ݼ���
typedef struct
{
	Axis3f acc;				//���ٶȣ�G��
	Axis3f gyro;			//�����ǣ�deg/s��
	Axis3f mag;				//�����ƣ�gauss��
	baro_t baro;	
	US100_t height;
} sensorData_t;

//������̬���ݽṹ
typedef struct
{
	attitude_t 	attitude;	//��̬�Ƕȣ�deg��
	point_t 	position;	//�����λ�ã�cm��
	velocity_t 	velocity;	//������ٶȣ�cm/s��
	acc_t acc;				//����ļ��ٶȣ�cm/ss��
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

//Ŀ����̬���ݽṹ
typedef struct
{
	attitude_t attitude;		//Ŀ����̬�Ƕȣ�deg�� //�ṹ�壬�ں�����ֵ
	attitude_t attitudeRate;	//Ŀ����ٶȣ�deg/s��  //�ṹ�壬�ں�����ֵ
	point_t position;         	//Ŀ��λ�ã�cm��  //�ṹ�壬�ں�3��ֵ
	velocity_t velocity;      	//Ŀ���ٶȣ�cm/s��
	mode_t mode;				
	float thrust;				
} setpoint_t;

//�������ݽṹ
typedef struct
{
	float roll;
	float pitch;
	float yaw;
	float thrust;  //����������
} control_t;


#endif

