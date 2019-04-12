/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_ADC_H
#define __CUSTOM_ADC_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <stdio.h>

/* Define config -------------------------------------------------------------*/

typedef enum
{
	ADC_IN1 = 0,	//PA1
	ADC_IN2,		//PA2
	
}ADC_CHANNEL;

/* Macro ---------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
void Custom_ADC1_Voltage(void);
void Custom_ADC1_Filter(void);
void Custom_ADC1_DMA_Config(void);

/* Exported constants --------------------------------------------------------*/

#endif  /* __CUSTOM_ADC_H */

