#include "runtime_config.h"
#include "led.h"
#include "gyro.h"
#include "led_task.h"
#include "open_mv.h"
#include "state_control.h"
#include "accelerometer.h"
#include "alt_control.h"

/**********************���м��ϵͳ**************************/ 

 	


uint32_t armingFlags = 0;
uint32_t stateFlags = 0;
uint32_t flightModeFlags = 0;


//ʹ�ܸ�������ģʽ
uint32_t enableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags |= (mask);
    if (flightModeFlags != oldVal)
        RLED_Change();
    return flightModeFlags;
}

//���ܸ�������ģʽ
uint32_t disableFlightMode(flightModeFlags_e mask)
{
    uint32_t oldVal = flightModeFlags;

    flightModeFlags &= ~(mask);
    if (flightModeFlags != oldVal)
       RLED_Change();
    return flightModeFlags;
}

//У׼������
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

//���½�����־λ״̬
void updateArmingStatus(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
     RLED_ON();   
  } 
	else 
	{
		//������У׼��
		if (isCalibrating()) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_SENSORS_CALIBRATING);
		}

		//���ˮƽ��С��
		if (!STATE(SMALL_ANGLE)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_NOT_LEVEL);
		}

		//�����ٶȼ��Ƿ��Ѿ�У׼���
		if (!STATE(ACCELEROMETER_CALIBRATED)) 
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}
		else 
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_ACCELEROMETER_NOT_CALIBRATED);
		}

		
		//����Ƿ���дFlash
		if (STATE(FLASH_WRITING))
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		
		// ��鼤�⴫���� �Ƿ� ����
				if (!STATE(CALIBRATE_MAG))   //����������
		{
			ENABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		else
		{
			DISABLE_ARMING_FLAG(ARMING_DISABLED_FLASH_WRITING);
		}
		
		//ˢ��LED��״̬
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

//�����ɿ�
void mwDisarm(void)
{
    if (ARMING_FLAG(ARMED)) 
	{
		
        DISABLE_ARMING_FLAG(ARMED);
				WORK_MODE=START_POINT;
				AUTO_FLY=0;   //�Զ����йر�
						
    }
}

//�����ɿ�
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
		
		stateControlResetYawHolding();//��λ���������ֵ
				
        return;
    }
}

