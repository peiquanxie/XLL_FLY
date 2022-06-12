/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.


    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************/



#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H



#define configUSE_PREEMPTION					       1            //1ʹ����ռʽ�ںˣ�0ʹ��Э��
#define configUSE_TIME_SLICING					     1						//1ʹ��ʱ��Ƭ����(Ĭ��ʽʹ�ܵ�)
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1         //1�������ⷽ����ѡ����һ��Ҫ���е�����

#define configUSE_TICKLESS_IDLE					    0             //1���õ͹���ticklessģʽ
#define configUSE_QUEUE_SETS					      1             //Ϊ1ʱ���ö���
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) 80000000 )   //CPUƵ��
#define configTICK_RATE_HZ                  ( ( portTickType ) 1000 )    //ʱ�ӽ���Ƶ�ʣ���������Ϊ1000�����ھ���1ms
#define configMAX_PRIORITIES                ( 6 )                   	   //��ʹ�õ�����������ȼ�
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 100 )
#define configMAX_TASK_NAME_LEN             ( 16 )
#define configUSE_16_BIT_TICKS					    0              //ϵͳ���ļ����������������ͣ�                                                                       //1��ʾΪ16λ�޷������Σ�0��ʾΪ32λ�޷�������
#define configIDLE_SHOULD_YIELD					    0              //Ϊ1ʱ�����������CPUʹ��Ȩ������ͬ���ȼ����û�����
#define configUSE_TASK_NOTIFICATIONS        1              //Ϊ1ʱ��������֪ͨ
#define configUSE_MUTEXES                   1              //ʹ�û����ź���
#define configQUEUE_REGISTRY_SIZE           10
#define configCHECK_FOR_STACK_OVERFLOW      1              //Ϊ1ʱ��ջ��������
#define configUSE_RECURSIVE_MUTEXES				  1              //Ϊ1ʱʹ�õݹ黥���ź���
#define configUSE_MALLOC_FAILED_HOOK			  1              //1ʹ���ڴ�����ʧ�ܹ��Ӻ���
#define configUSE_APPLICATION_TASK_TAG			1                       
#define configUSE_COUNTING_SEMAPHORES			  1              //Ϊ1ʱʹ�ü����ź���             



/***************************************************************************************************************/
/*                                FreeRTOS��ѡ��������ѡ��                                                     */
/***************************************************************************************************************/
#define INCLUDE_vTaskPrioritySet			      1
#define INCLUDE_uxTaskPriorityGet			      1
#define INCLUDE_vTaskDelete					        1
#define INCLUDE_vTaskCleanUpResources		    1
#define INCLUDE_vTaskSuspend				        1
#define INCLUDE_vTaskDelayUntil				      1
#define INCLUDE_vTaskDelay					        1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetSchedulerState  	  1

/***************************************************************************************************************/
/*                                FreeRTOS���ڴ������й�����ѡ��                                               */
/***************************************************************************************************************/
#define configSUPPORT_DYNAMIC_ALLOCATION        1                       //֧�ֶ�̬�ڴ�����
#define configTOTAL_HEAP_SIZE					( ( size_t ) ( 20000 ) )     //ϵͳ�����ܵĶѴ�С30K

/***************************************************************************************************************/
/*                                FreeRTOS�빳�Ӻ����йص�����ѡ��                                             */
/***************************************************************************************************************/
#define configUSE_IDLE_HOOK						1                       //1��ʹ�ÿ��й��ӣ�0����ʹ��
#define configUSE_TICK_HOOK						0                       //1��ʹ��ʱ��Ƭ���ӣ�0����ʹ��

/***************************************************************************************************************/
/*                                FreeRTOS������ʱ�������״̬�ռ��йص�����ѡ��                               */
/***************************************************************************************************************/
#define configGENERATE_RUN_TIME_STATS	        0                       //Ϊ1ʱ��������ʱ��ͳ�ƹ���
#define configUSE_TRACE_FACILITY				1                       //Ϊ1���ÿ��ӻ����ٵ���
#define configUSE_STATS_FORMATTING_FUNCTIONS	1                       //���configUSE_TRACE_FACILITYͬʱΪ1ʱ���������3������
                                                                        //prvWriteNameToBuffer(),vTaskList(),

 /***************************************************************************************************************/
/*                                FreeRTOS��Э���йص�����ѡ��                                                 */
/***************************************************************************************************************/
#define configUSE_CO_ROUTINES 			        0                       //Ϊ1ʱ����Э�̣�����Э���Ժ��������ļ�croutine.c
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )                   //Э�̵���Ч���ȼ���Ŀ                                                                        //vTaskGetRunTimeStats()
                                                                       //MCUû����ЩӲ��ָ��Ļ��˺�Ӧ������Ϊ0��

/***************************************************************************************************************/
/*                                FreeRTOS�������ʱ���йص�����ѡ��                                           */
/***************************************************************************************************************/
#define configUSE_TIMERS				        (1)                             //Ϊ1ʱ���������ʱ��
#define configTIMER_TASK_PRIORITY		        (1)       				 		//�����ʱ�����ȼ�
#define configTIMER_QUEUE_LENGTH		        (200)                           //�����ʱ�����г���
#define configTIMER_TASK_STACK_DEPTH	        (configMINIMAL_STACK_SIZE)    	//�����ʱ�������ջ��С

/***************************************************************************************************************/
/*                                FreeRTOS���ж��йص�����ѡ��                                                 */
/***************************************************************************************************************/
#ifdef __NVIC_PRIO_BITS
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		3                  
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			7                      //�ж�������ȼ�
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	2                       //ϵͳ�ɹ��������ж����ȼ�
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )


/***************************************************************************************************************/
/*                                FreeRTOS���жϷ������йص�����ѡ��                                         */
/***************************************************************************************************************/
//#define xPortPendSVHandler 	PendSV_Handler
//#define vPortSVCHandler 	SVC_Handler

//#define configASSERT( x )  if( ( x ) == 0 ) assertFail(#x, __FILE__, __LINE__ )

//ms to OS Ticks
#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))


#define usercfgCPU_USAGE_CALCULATE   1

#ifdef usercfgCPU_USAGE_CALCULATE
#define usercfgCPU_USAGE_CALC_PERIOD 500//ms
#endif

#endif /* FREERTOS_CONFIG_H */
