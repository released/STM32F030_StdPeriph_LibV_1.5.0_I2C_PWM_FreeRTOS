/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "Macro.h"
#include "stm32f0xx.h"
#include <stdio.h>

#include "i2c_device.h"
#include "Custom_ADC.h"

/* Define config -------------------------------------------------------------*/

#define SIMPLEDELAY

#if  defined ( __GNUC__ )
  #ifndef __weak
    #define __weak   __attribute__((weak))
  #endif /* __weak */
  #ifndef __packed
    #define __packed __attribute__((__packed__))
  #endif /* __packed */
#endif /* __GNUC__ */

#define LED3_ON(void)					GPIO_SetBits(GPIOC,GPIO_Pin_9)
#define LED3_OFF(void)					(GPIO_ResetBits(GPIOC,GPIO_Pin_9))
#define LED3_TOGGLE(void)				(GPIOC->ODR^= GPIO_Pin_9)

#define LED4_ON(void)					GPIO_SetBits(GPIOC,GPIO_Pin_8)
#define LED4_OFF(void)					(GPIO_ResetBits(GPIOC,GPIO_Pin_8))
#define LED4_TOGGLE(void)				(GPIOC->ODR^= GPIO_Pin_8)

typedef enum
{
	VL6180_None = 0,	
	VL6180_COMM_Error,	
	VL6180_READ_Error,	

	VL6180_READ_OK,
	

}VL6180_State;


/* Macro ---------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
void TIM1_PWM_Config(void);

void Delay(__IO uint32_t mTime);

void USART_Test(void);
void USART_Config(void);

uint32_t Button_GetState(void);
void Button_Config(void);
void LED_Config(void);

void SysTickConfig(void);
void TimingDelay_Decrement(void);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

