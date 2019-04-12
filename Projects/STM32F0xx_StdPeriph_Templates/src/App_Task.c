/* Includes ------------------------------------------------------------------*/
#include "App_Task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//FREERTOR variable
TaskHandle_t  			xTask1Handle;
TaskHandle_t				xTask2Handle;

QueueHandle_t 			xQueue;
QueueHandle_t			xIntegerQueue;
QueueHandle_t			xStringQueue;
QueueHandle_t			xPrintQueue;

SemaphoreHandle_t 		xBinarySemaphore;
SemaphoreHandle_t 		xCountingSemaphore;
SemaphoreHandle_t 		xMutex;

TaskHandle_t				xTaskHardwareInit;
TaskHandle_t				xTaskApplicationInit;

TaskHandle_t				xTaskAppStateMachine;
/* Private functions ---------------------------------------------------------*/
#define xPause(void)	\
{	\
	vTaskSuspendAll();	\
	xTaskResumeAll();	\
}	\


#if defined(__ICCARM__)
__weak void vPortEnableVFP(void)
{

}

__weak void vPortStartFirstTask(void)
{

}
#endif

__weak void vApplicationIdleHook(void)
{

}

__weak void vApplicationTickHook( void )
{

}

__weak void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

__weak void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}

void vTaskInitApplication(void * pvParameters)
{
	(void) pvParameters;

	vPrintString("vTaskInitApplication\r\n");
	
	I2C_DEVICE_Init();
	I2C_DEVICE_Setup();

	for( ;; ) 
	{
		/* Delete the Init Thread */ 
		vTaskDelete(xTaskApplicationInit);
	
	}
}

void vTaskAppBehavior(void *pvParameters)
{
	uint32_t millisec = 20;
	portTickType xLastWakeTime; 	
	unsigned portBASE_TYPE uxPriority; 
	
	(void) pvParameters;	

	xLastWakeTime = xTaskGetTickCount(); 	
    uxPriority = uxTaskPriorityGet( NULL ); 
	
	for( ;; ) 
	{
		
		Custom_ADC1_Voltage();
//		vPrintStringAndNumber("---------vTaskAppBehavior(ms):",millisec);
        vTaskPrioritySet( xTaskAppStateMachine, ( uxPriority + 1 ) ); 
		vTaskDelayUntil( &xLastWakeTime, ( millisec / portTICK_RATE_MS ) );
	}
}

void vTaskAppStateMachine(void *pvParameters)
{
	uint32_t millisec = 40;
	portTickType xLastWakeTime; 	
	unsigned portBASE_TYPE uxPriority; 
	
	(void) pvParameters;	

	xLastWakeTime = xTaskGetTickCount(); 	
    uxPriority = uxTaskPriorityGet( NULL ); 
	
	for( ;; ) 
	{

//		vPrintStringAndNumber("---------LED_Red(ms):",millisec);
        vTaskPrioritySet( NULL, ( uxPriority - 2 ) ); 
		vTaskDelayUntil( &xLastWakeTime, ( millisec / portTICK_RATE_MS ) );
	}
}

void vTaskAppIdle( void *pvParameters ) 
{ 
    /* The string to print out is passed in via the parameter.  Cast this to a 
    character pointer. */ 
    (void) pvParameters; 
 
    /* As per most tasks, this task is implemented in an infinite loop. */ 
    for( ;; ) 
    { 
        /* Print out the name of this task.  This task just does this repeatedly 
        without ever blocking or delaying. */ 

    } 
} 

/**
  * @brief  Pre Sleep Processing
  * @param  ulExpectedIdleTime: Expected time in idle state
  * @retval None
  */
void PreSleepProcessing(unsigned long ulExpectedIdleTime)
{
	/* Called by the kernel before it places the MCU into a sleep mode because
	configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().

	NOTE:  Additional actions can be taken here to get the power consumption
	even lower.  For example, peripherals can be turned off here, and then back
	on again in the post sleep processing function.  For maximum power saving
	ensure all unused pins are in their lowest power state. */

	/* Avoid compiler warnings about the unused parameter. */
	(void) ulExpectedIdleTime;

	/* Disable the peripheral clock during Low Power (Sleep) mode.*/
//	__HAL_RCC_GPIOG_CLK_SLEEP_DISABLE();

	vPrintString("PreSleep\r\n");


}

/**
  * @brief  Post Sleep Processing
  * @param  ulExpectedIdleTime : Not used
  * @retval None
  */
void PostSleepProcessing(unsigned long ulExpectedIdleTime)
{
	/* Called by the kernel when the MCU exits a sleep mode because
	configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */

	/* Avoid compiler warnings about the unused parameter. */
	(void) ulExpectedIdleTime;

	vPrintString("PostSleep\r\n");	
}


void vPrintStringAndNumber( const portCHAR *pcString ,const portLONG pcNum ) 
{
	#if 1
	vTaskSuspendAll();
	{
		printf( "%s%d\r\n", pcString,pcNum); 
		fflush( stdout ); 
	}
	xTaskResumeAll();
	#else
	
	taskENTER_CRITICAL(); 
	{ 
		printf( "%s%d\r\n", pcString,pcNum); 
//		printf( "%s \r\n",pcNumString);		
		fflush( stdout ); 
	} 
	taskEXIT_CRITICAL(); 
	#endif	
} 

void vPrintString( const portCHAR *pcString ) 
{
	#if 1
	vTaskSuspendAll();
	{
		printf("%s",pcString);
		fflush(stdout);
	}
	xTaskResumeAll();
	#else
	taskENTER_CRITICAL(); 
	{ 
		printf( "%s", pcString ); 
		fflush( stdout ); 
	} 
	taskEXIT_CRITICAL(); 
	#endif
} 

void vTaskInitHardware(void * pvParameters)
{
	(void) pvParameters;

	USART_Config();
	SysTickConfig();
	LED_Config();
	TIM1_PWM_Config();
	Custom_ADC1_DMA_Config();

//	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );	

	/*	
		//http://www.freertos.org/RTOS-Cortex-M3-M4.html
		Relevance when using the RTOS
		It is recommended to assign all the priority bits to be preempt priority bits, 
		leaving no priority bits as subpriority bits. 
		Any other configuration complicates the otherwise direct relationship between 
		the configMAX_SYSCALL_INTERRUPT_PRIORITY setting and the priority assigned to 
		individual peripheral interrupts.

		Most systems default to the wanted configuration, with the noticeable 
		exception of the STM32 driver library. If you are using an STM32 with 
		the STM32 driver library then ensure all the priority bits are assigned 
		to be preempt priority bits by calling NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 ); 
		before the RTOS is started. 
	*/

	for( ;; )
	{
		/* Delete the Init Thread */ 
		vTaskDelete(xTaskHardwareInit);
	}
	
}


