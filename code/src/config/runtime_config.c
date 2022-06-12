#include "runtime_config.h"
#include "led.h"
#include "gyro.h"
#include "led_task.h"
#include "open_mv.h"
#include "state_control.h"
#include "accelerometer.h"
#include "alt_control.h"

/**********************运行监测系统**************************/ 

 	


uint32_t armingFlags = 0;
uint32_t stateFlags = 0;
uint32_t flightModeFlags = 0;


//使能给定飞行模式
uint32_t enableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        RLED_Change();
    return flightModeFlags;
}

//禁能给定飞行模式
uint32_t disableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
       RLED_Change();
    return flightModeFlags;
}

//校准进行中
bool isCalibrating(void)
{
	
	if (!accIsCalibrationComplete()) 
	{
		return true;
	}

	if (!gyroIsCalibrationComplete()) 
	{
		return true;
	}

    return false;
}

//更新解锁标志位状态
void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
     RLED_ON();   
  } 
	else 
	{
		//传感器校准中
		if (isCalibrating()) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
		}

		//检查水平最小角
		if (!STATE(SMALL_ANGLE)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}

		//检查加速度计是否已经校准完成
		if (!STATE(ACCELEROMETER_CALIBRATED)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}

		
		//检查是否在写Flash
		if (STATE(FLASH_WRITING))
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		
		// 检查激光传感器 是否 正常
				if (!STATE(CALIBRATE_MAG))   //激光无数据
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		
		//刷新LED灯状态
        if (!isArmingDisabled()) 
		{
            warningLedFlash();
        } 
		else 
		{
            warningLedON();
        }
		
        warningLedUpdate();
    }
}

//锁定飞控
void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
		
        DISABLE_ARMING_FLAG(ARMED);
				WORK_MODE=START_POINT;
				AUTO_FLY=0;   //自动飞行关闭
						
    }
}

//解锁飞控
void mwArm(void)
{
    updateArmingStatus();

    if (!isArmingDisabled()) 
	{
     if (ARMING_FLAG(ARMED)) 
		{
            return;
    }
		
		Init_FlyMode();
		Init_Flag();	
		WORK_MODE=START_POINT;
				
				
		ENABLE_ARMING_FLAG(ARMED);
		ENABLE_ARMING_FLAG(WAS_EVER_ARMED);
		
		stateControlResetYawHolding();//复位航向角锁定值
				
        return;
    }
}

