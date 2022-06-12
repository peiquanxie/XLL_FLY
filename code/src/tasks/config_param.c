#include <string.h>
#include "config_param.h"
#include "tiva_flash.h"
#include "led.h"
#include "runtime_config.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"


#define VERSION 11	/*11 表示V1.1*/


configParam_t configParam;


 configParam_t configParamDefault=
{
	.version = VERSION,		/*软件版本号*/

	.pid = 
	{
		[RATE_ROLL]   = {290, 327, 33},
		[RATE_PITCH]  = {290, 327, 33},
		[RATE_YAW]    = {200, 0, 0},
		[ANGLE_ROLL]  = {316, 0, 0},
		[ANGLE_PITCH] = {316, 0, 0},
		[ANGLE_YAW]   = {200, 0, 0},
		[VELOCITY_Z]  = {0, 0, 0},
		[POSHOLD_Z]   = {45, 0, 0},
		[VELOCITY_XY] = {0, 0, 0},
		[POSHOLD_XY]  = {0, 0, 0},
		[VELOCITY_LAND] = {2, 0, 15},
	},
	.accBias= 	/*加速度校准值*/
	{
		.accZero = 
		{
			0,
			0,
			0,
		},
		.accGain = 
		{
			4096,
			4096,
			4096,
		}
	},
	.magBias=
	{
		.magZero = 
		{
			0,
			0,
			0,
		},
	},
	.boardAlign=
	{
		.rollDeciDegrees = 0,
		.pitchDeciDegrees = 0,
		.yawDeciDegrees = 0,
	},
};

static u32 lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;
static SemaphoreHandle_t  xSemaphore = NULL;
static u32 semaphoreGiveTime;

static u8 configParamCksum(configParam_t* data)
{
	int i;
	u8 cksum=0;	
	u8* c = (u8*)data;  	
	size_t len=sizeof(configParam_t);

	for (i=0; i<len; i++)
		cksum += *(c++);
	cksum-=data->cksum;
	
	return cksum;
}

void configParamInit(void)	/*参数配置初始化*/
{
	if(isInit) return;
	
	EEPROM_Init();
	lenth=sizeof(configParam);
	lenth=lenth/4+(lenth%4 ? 1:0);

	TIVAFLASH_Read(TIVA_FLASH_BASE, (uint32_t *)&configParam, lenth);
	
	if (configParam.version == VERSION)	/*版本正确*/
	{
		if(configParamCksum(&configParam) == configParam.cksum)	/*校验正确*/
			   isConfigParamOK = true;
		else
			   isConfigParamOK = false;
	}
	else	/*版本更新*/
	{
		isConfigParamOK = false;
	}
	
	if(isConfigParamOK == false)	/*配置参数错误，写入默认参数*/
	{
		memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);				/*计算校验值*/
		TIVAFLASH_Write(TIVA_FLASH_BASE,(uint32_t *)&configParam, lenth);	/*写入stm32 flash*/
		isConfigParamOK=true;
	}	
	
	xSemaphore = xSemaphoreCreateBinary();   //创建二值信号量，默认为空
	
	isInit=true;
}

void configParamTask(void* param)
{
	u8 cksum = 0;
	
	while(1) 
	{	

		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		
		for (;;)
		{
			if ( (xTaskGetTickCount() - semaphoreGiveTime > 2000) &&(!ARMING_FLAG(ARMED)))//上次释放信号量时间至少大于2秒才写Flash，防止频繁写入    
			{
				cksum = configParamCksum(&configParam);
				if((configParam.cksum != cksum))
				{
					configParam.cksum = cksum;					
					TIVAFLASH_Write(TIVA_FLASH_BASE,(uint32_t *)&configParam, lenth);
				}
				DISABLE_STATE(FLASH_WRITING);
			}
			else
			{
				xSemaphoreTake(xSemaphore, 100);
			}
		}
	}
}

void saveConfigAndNotify(void)
{
	if (!ARMING_FLAG(ARMED)) //锁定状态才可以操作Flash
	{	
		BLED_Change();
		xSemaphoreGive(xSemaphore);
		semaphoreGiveTime = xTaskGetTickCount();
		ENABLE_STATE(FLASH_WRITING);
	}
}

//恢复为默认参数
void resetConfigParam(void)
{
	configParam = configParamDefault;
	saveConfigAndNotify();
}

