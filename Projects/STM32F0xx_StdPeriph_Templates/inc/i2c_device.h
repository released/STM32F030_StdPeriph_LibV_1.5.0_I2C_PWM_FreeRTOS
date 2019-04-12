
#include "stm32f0xx.h"
#include <stdio.h>

typedef enum _I2C_DEVICE_MESSAGE{
	I2C_DEVICE_success = 0,
	I2C_DEVICE_error,
}I2C_DEVICE_MESSAGE;

//#define _DEBUG_MODE_DEVICE

#if defined (_DEBUG_MODE_DEVICE)
void result_var_failed(uint16_t paramters ,uint8_t* file, uint32_t line);
//#define result_var(res,reg) ((res) ? (void)0 : result_var_failed(reg,(uint8_t *)__FILE__, __LINE__))

#define result_var(res,reg)	\
{	\
	if (res == I2C_DEVICE_error)	\
	{	\
		result_var_failed(reg,(uint8_t *)__FILE__, __LINE__);	\
	}	\
}	\

#else
#define result_var(res,reg)	((void)0)
#endif /*_DEBUG_MODE_DEVICE*/

#define I2C_DEVICE								I2C1
#define DEVICE_DeviceAddr						0x52

#define I2C_DEVICE_CLK							RCC_APB1Periph_I2C1
#define I2C_DEVICE_SCL_PIN						GPIO_Pin_6                  
#define I2C_DEVICE_SCL_GPIO_PORT				GPIOB                       
#define I2C_DEVICE_SCL_GPIO_CLK					RCC_AHBPeriph_GPIOB
#define I2C_DEVICE_SCL_SOURCE					GPIO_PinSource6
#define I2C_DEVICE_SCL_AF						GPIO_AF_1

#define I2C_DEVICE_SDA_PIN						GPIO_Pin_7                  
#define I2C_DEVICE_SDA_GPIO_PORT				GPIOB                       
#define I2C_DEVICE_SDA_GPIO_CLK					RCC_AHBPeriph_GPIOB
#define I2C_DEVICE_SDA_SOURCE					GPIO_PinSource7
#define I2C_DEVICE_SDA_AF						GPIO_AF_1

#define I2C_DEVICE_ClockSpeed					100000	//0x00210507

#define I2C_DEVICE_FLAG_TIMEOUT				((uint32_t)0x100)
#define I2C_DEVICE_LONG_TIMEOUT				((uint32_t)(10 * I2C_DEVICE_FLAG_TIMEOUT))

/*====================== CPAL Structure configuration ========================*/ 
/* Select I2C device (uncomment relative define) */
//#define DEVICE_DevStructure                		I2C1_DevStructure

/*============== TIMING Configuration ==========================*/
#define I2C_DEVICE_TIMING 						I2C_DEVICE_ClockSpeed	//0x1045061D


void I2C_DEVICE_Init(void);
I2C_DEVICE_MESSAGE I2C_DEVICE_Read(uint16_t Address,uint16_t DeviceAddr,uint8_t* pBuffer, uint8_t NumByteToWrite);
I2C_DEVICE_MESSAGE I2C_DEVICE_Write(uint16_t Address,uint16_t DeviceAddr,uint8_t* pBuffer,uint8_t NumByteToWrite);

uint8_t ReadByte(uint16_t Address);
void WriteByte(uint16_t Address,uint8_t Val);

void I2C_DEVICE_Setup(void);

