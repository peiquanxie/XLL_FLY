#include "Headfile.h"       //tiva库头文件
#include "system_init.h"    //用户外设及初始化

/* freertos 配置文件 */
#include "FreeRTOSConfig.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"


/****任务头文件*****/
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
  sys_Init();   //底层初始化
	
	xTaskCreate(startTask, "START_TASK", 300, NULL, 5, &startTaskHandle);	/*创建起始任务*/

	vTaskStartScheduler();	/*开启任务调度*/

	while(1){};
}

/*创建任务*/
void startTask(void *arg)
{
	
#if(usercfgCPU_USAGE_CALCULATE==1)
    uTaskCPUUsageInit();
  #endif
	
	taskENTER_CRITICAL();	/*进入临界区*/
	
	xTaskCreate(stabilizerTask, "stabilizerTask", 500, NULL, 6, NULL);	
	
	xTaskCreate(sensorsTask, "sensorsTask", 450, NULL, 6, NULL);
	
	xTaskCreate(positionTask, "positionTask", 500, NULL, 5, NULL);
	
	xTaskCreate(ppmRxTask, "ppmRxTask", 150, NULL, 4, NULL);					/*创建遥控器PPM信号接收任务*/
	
	xTaskCreate(fsRxTask, "fsRxTask", 150, NULL, 4, NULL);					/*创建遥控器处理任务任务*/
	
	xTaskCreate(usartTxTask, "usartTxTask", 350, NULL, 3, NULL);		/*创建上位机发送任务*/
	
	xTaskCreate(usartRxTask, "usartRxTask", 500, NULL, 3, NULL);		/*创建atkp解析任务*/
	
	xTaskCreate(configParamTask, "configTask", 150, NULL, 1, NULL);	/*创建参数配置任务*/
	
	xTaskCreate(led1Task, "led1Task", 100, NULL, 1, NULL);

	vTaskDelete(startTaskHandle);										/*删除开始任务*/

	
	taskEXIT_CRITICAL();	/*退出临界区*/
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
