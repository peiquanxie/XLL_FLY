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



#define configUSE_PREEMPTION					       1            //1使用抢占式内核，0使用协程
#define configUSE_TIME_SLICING					     1						//1使能时间片调度(默认式使能的)
#define configUSE_PORT_OPTIMISED_TASK_SELECTION	1         //1启用特殊方法来选择下一个要运行的任务

#define configUSE_TICKLESS_IDLE					    0             //1启用低功耗tickless模式
#define configUSE_QUEUE_SETS					      1             //为1时启用队列
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) 80000000 )   //CPU频率
#define configTICK_RATE_HZ                  ( ( portTickType ) 1000 )    //时钟节拍频率，这里设置为1000，周期就是1ms
#define configMAX_PRIORITIES                ( 6 )                   	   //可使用的任务最大优先级
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 100 )
#define configMAX_TASK_NAME_LEN             ( 16 )
#define configUSE_16_BIT_TICKS					    0              //系统节拍计数器变量数据类型，                                                                       //1表示为16位无符号整形，0表示为32位无符号整形
#define configIDLE_SHOULD_YIELD					    0              //为1时空闲任务放弃CPU使用权给其他同优先级的用户任务
#define configUSE_TASK_NOTIFICATIONS        1              //为1时打开任务间的通知
#define configUSE_MUTEXES                   1              //使用互斥信号量
#define configQUEUE_REGISTRY_SIZE           10
#define configCHECK_FOR_STACK_OVERFLOW      1              //为1时打开栈满溢出检测
#define configUSE_RECURSIVE_MUTEXES				  1              //为1时使用递归互斥信号量
#define configUSE_MALLOC_FAILED_HOOK			  1              //1使用内存申请失败钩子函数
#define configUSE_APPLICATION_TASK_TAG			1                       
#define configUSE_COUNTING_SEMAPHORES			  1              //为1时使用计数信号量             



/***************************************************************************************************************/
/*                                FreeRTOS可选函数配置选项                                                     */
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
/*                                FreeRTOS与内存申请有关配置选项                                               */
/***************************************************************************************************************/
#define configSUPPORT_DYNAMIC_ALLOCATION        1                       //支持动态内存申请
#define configTOTAL_HEAP_SIZE					( ( size_t ) ( 20000 ) )     //系统所有总的堆大小30K

/***************************************************************************************************************/
/*                                FreeRTOS与钩子函数有关的配置选项                                             */
/***************************************************************************************************************/
#define configUSE_IDLE_HOOK						1                       //1，使用空闲钩子；0，不使用
#define configUSE_TICK_HOOK						0                       //1，使用时间片钩子；0，不使用

/***************************************************************************************************************/
/*                                FreeRTOS与运行时间和任务状态收集有关的配置选项                               */
/***************************************************************************************************************/
#define configGENERATE_RUN_TIME_STATS	        0                       //为1时启用运行时间统计功能
#define configUSE_TRACE_FACILITY				1                       //为1启用可视化跟踪调试
#define configUSE_STATS_FORMATTING_FUNCTIONS	1                       //与宏configUSE_TRACE_FACILITY同时为1时会编译下面3个函数
                                                                        //prvWriteNameToBuffer(),vTaskList(),

 /***************************************************************************************************************/
/*                                FreeRTOS与协程有关的配置选项                                                 */
/***************************************************************************************************************/
#define configUSE_CO_ROUTINES 			        0                       //为1时启用协程，启用协程以后必须添加文件croutine.c
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )                   //协程的有效优先级数目                                                                        //vTaskGetRunTimeStats()
                                                                       //MCU没有这些硬件指令的话此宏应该设置为0！

/***************************************************************************************************************/
/*                                FreeRTOS与软件定时器有关的配置选项                                           */
/***************************************************************************************************************/
#define configUSE_TIMERS				        (1)                             //为1时启用软件定时器
#define configTIMER_TASK_PRIORITY		        (1)       				 		//软件定时器优先级
#define configTIMER_QUEUE_LENGTH		        (200)                           //软件定时器队列长度
#define configTIMER_TASK_STACK_DEPTH	        (configMINIMAL_STACK_SIZE)    	//软件定时器任务堆栈大小

/***************************************************************************************************************/
/*                                FreeRTOS与中断有关的配置选项                                                 */
/***************************************************************************************************************/
#ifdef __NVIC_PRIO_BITS
	#define configPRIO_BITS       		__NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       		3                  
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			7                      //中断最低优先级
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	2                       //系统可管理的最高中断优先级
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )


/***************************************************************************************************************/
/*                                FreeRTOS与中断服务函数有关的配置选项                                         */
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
