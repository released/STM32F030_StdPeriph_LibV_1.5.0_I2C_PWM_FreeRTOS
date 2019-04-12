/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_TASK_H
#define __APP_TASK_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "Macro.h"
#include "stm32f0xx.h"
#include "hw_config.h"

//FREERTOS include header file
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"

#include "Task.h"
#include "Queue.h"
#include "semphr.h"

#include <cmsis_os.h> 

/* Define config -------------------------------------------------------------*/
//typedef xTaskHandle osThreadId; 

extern TaskHandle_t  		xTask1Handle;
extern TaskHandle_t			xTask2Handle;

extern QueueHandle_t 		xQueue;
extern QueueHandle_t		xIntegerQueue;
extern QueueHandle_t		xStringQueue;
extern QueueHandle_t		xPrintQueue;

extern SemaphoreHandle_t 	xBinarySemaphore;
extern SemaphoreHandle_t 	xCountingSemaphore;
extern SemaphoreHandle_t 	xMutex;

extern TaskHandle_t			xTaskHardwareInit;
extern TaskHandle_t			xTaskApplicationInit;

extern TaskHandle_t			xTaskAppStateMachine;
/* Macro ---------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

void vTaskInitApplication(void * pvParameters);
void vTaskAppBehavior(void *pvParameters);
void vTaskAppStateMachine(void *pvParameters);
void vTaskAppIdle(void *pvParameters) ;

/* Common function types -----------------------------------------------------*/
void vPrintStringAndNumber(const portCHAR *pcString ,const portLONG pcNum) ;
void vPrintString(const portCHAR *pcString) ;

void vTaskInitHardware(void * pvParameters);

/* Exported constants --------------------------------------------------------*/

#endif  /* __APP_TASK_H */

