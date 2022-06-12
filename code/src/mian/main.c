#include "Headfile.h"       //tiva��ͷ�ļ�
#include "system_init.h"    //�û����輰��ʼ��

/* freertos �����ļ� */
#include "FreeRTOSConfig.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"


/****����ͷ�ļ�*****/
#include "led_task.h"  
#include "sensors_task.h" 
#include "stabilizer_task.h"
#include "usart_task.h"
#include "config_param.h"
#include "fsrx_task.h"
#include "position_task.h"



TaskHandle_t startTaskHandle;
static void startTask(void *arg);

int main() 
{
  sys_Init();   //�ײ��ʼ��
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 5, &startTaskHandle);	/*������ʼ����*/

	vTaskStartScheduler();	/*�����������*/

	while(1){};
}

/*��������*/
void startTask(void *arg)
{
	
#if(usercfgCPU_USAGE_CALCULATE==1)
    uTaskCPUUsageInit();
  #endif
	
	taskENTER_CRITICAL();	/*�����ٽ���*/
	
	xTaskCreate(stabilizerTask, "stabilizerTask", 500, NULL, 6, NULL);	
	
	xTaskCreate(sensorsTask, "sensorsTask", 450, NULL, 6, NULL);
	
	xTaskCreate(positionTask, "positionTask", 500, NULL, 5, NULL);
	
	xTaskCreate(ppmRxTask, "ppmRxTask", 150, NULL, 4, NULL);					/*����ң����PPM�źŽ�������*/
	
	xTaskCreate(fsRxTask, "fsRxTask", 150, NULL, 4, NULL);					/*����ң����������������*/
	
	xTaskCreate(usartTxTask, "usartTxTask", 350, NULL, 3, NULL);		/*������λ����������*/
	
	xTaskCreate(usartRxTask, "usartRxTask", 500, NULL, 3, NULL);		/*����atkp��������*/
	
	xTaskCreate(configParamTask, "configTask", 150, NULL, 1, NULL);	/*����������������*/
	
	xTaskCreate(led1Task, "led1Task", 100, NULL, 1, NULL);

	vTaskDelete(startTaskHandle);										/*ɾ����ʼ����*/

	
	taskEXIT_CRITICAL();	/*�˳��ٽ���*/
} 




void vApplicationIdleHook( void )
{

}



void vApplicationStackOverflowHook(void )
{
	while(1)
	{
		BLED_ON();
	}
}


void vApplicationMallocFailedHook( void )
{
	portDISABLE_INTERRUPTS();
	while(1);
}
