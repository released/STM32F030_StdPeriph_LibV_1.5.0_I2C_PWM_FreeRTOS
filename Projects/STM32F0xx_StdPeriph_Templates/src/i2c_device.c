#include "i2c_device.h"
//#include <stdio.h>

__IO uint16_t DeviceAddress = 0;


#if defined (_DEBUG_MODE_DEVICE)
void result_var_failed(uint16_t paramters ,uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	printf("Result parameters (0x%3X) value: file %s on line %d\r\n",paramters, file, line);
}
#endif /*_DEBUG_MODE_DEVICE*/

__IO uint32_t  DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT; 

void I2C_DEVICE_LowLevel_DeInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Disable I2C_DEVICE */
	I2C_Cmd(I2C_DEVICE, DISABLE);

	/* DeInitializes the I2C_DEVICE */
	I2C_DeInit(I2C_DEVICE);

	/* I2C_DEVICE Periph clock disable */
	RCC_APB1PeriphClockCmd(I2C_DEVICE_CLK, DISABLE);

	/* Configure I2C_DEVICE pins: SCL */
	GPIO_InitStructure.GPIO_Pin = I2C_DEVICE_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(I2C_DEVICE_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* Configure I2C_DEVICE pins: SDA */
	GPIO_InitStructure.GPIO_Pin = I2C_DEVICE_SDA_PIN;
	GPIO_Init(I2C_DEVICE_SDA_GPIO_PORT, &GPIO_InitStructure);

}

/**
  * @brief  Initializes the I2C source clock and IOs used to drive the I2C_DEVICE
  * @param  None
  * @retval None
  */
void I2C_DEVICE_LowLevel_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* I2C_DEVICE Periph clock enable */
	RCC_APB1PeriphClockCmd(I2C_DEVICE_CLK, ENABLE);

	/* Configure the I2C clock source. The clock is derived from the HSI */
	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Reset sI2C_DEVICE IP */
	RCC_APB1PeriphResetCmd(I2C_DEVICE_CLK, ENABLE);
	/* Release reset signal of sI2C_DEVICE IP */
	RCC_APB1PeriphResetCmd(I2C_DEVICE_CLK, DISABLE);

	/* I2C_DEVICE_SCL_GPIO_CLK, I2C_DEVICE_SDA_GPIO_CLK 
	   and I2C_DEVICE_SMBUSALERT_GPIO_CLK Periph clock enable */
	RCC_AHBPeriphClockCmd(I2C_DEVICE_SCL_GPIO_CLK | I2C_DEVICE_SDA_GPIO_CLK , ENABLE);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(I2C_DEVICE_SCL_GPIO_PORT, I2C_DEVICE_SCL_SOURCE, I2C_DEVICE_SCL_AF);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(I2C_DEVICE_SDA_GPIO_PORT, I2C_DEVICE_SDA_SOURCE, I2C_DEVICE_SDA_AF); 

	/* Configure I2C_DEVICE pins: SCL */
	GPIO_InitStructure.GPIO_Pin = I2C_DEVICE_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(I2C_DEVICE_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* Configure I2C_DEVICE pins: SDA */
	GPIO_InitStructure.GPIO_Pin = I2C_DEVICE_SDA_PIN;
	GPIO_Init(I2C_DEVICE_SDA_GPIO_PORT, &GPIO_InitStructure);
}


void I2C_DEVICE_Delay(uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

void I2C_DEVICE_Init(void)
{
	I2C_InitTypeDef  I2C_InitStructure;

	I2C_DEVICE_LowLevel_Init();

	/* I2C_DEVICE configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_Timing = I2C_DEVICE_TIMING;

	/* Apply I2C_DEVICE configuration after enabling it */
	I2C_Init(I2C_DEVICE, &I2C_InitStructure);

	/* I2C_DEVICE Peripheral Enable */
	I2C_Cmd(I2C_DEVICE, ENABLE);
}

I2C_DEVICE_MESSAGE I2C_DEVICE_TIMEOUT_UserCallback(void)
{
	I2C_DeInit(I2C_DEVICE);
	I2C_DEVICE_Init();

#if defined (_DEBUG_MODE_DEVICE)	
	printf("I2C Timeout!!\n\r");
#endif /*_DEBUG_MODE_DEVICE*/

	return I2C_DEVICE_error;
}

I2C_DEVICE_MESSAGE I2C_DEVICE_Write(uint16_t Address,uint16_t DeviceAddr,uint8_t* pBuffer, uint8_t NumByteToWrite)
{

	uint32_t DataNum = 0;

	/* Configure slave address, nbytes, reload and generate start */
	I2C_TransferHandling(I2C_DEVICE, DeviceAddr, 2, I2C_Reload_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TXIS) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Send MSB of memory address */
	I2C_SendData(I2C_DEVICE, (uint8_t)((Address & 0xFF00) >> 8));  

	/* Wait until TXIS flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TXIS) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Send LSB of memory address  */
	I2C_SendData(I2C_DEVICE, (uint8_t)(Address & 0x00FF));


	/* Wait until TCR flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TCR) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Update CR2 : set Slave Address , set write request, generate Start and set end mode */
	I2C_TransferHandling(I2C_DEVICE, DeviceAddr, (uint8_t)(NumByteToWrite), I2C_AutoEnd_Mode, I2C_No_StartStop);

	while (DataNum != (NumByteToWrite))
	{      
		/* Wait until TXIS flag is set */
		DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
		while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TXIS) == RESET)
		{
		  if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
		}  

		/* Write data to TXDR */
		I2C_SendData(I2C_DEVICE, (uint8_t)(pBuffer[DataNum]));

		/* Update number of transmitted data */
		DataNum++;   
	}  

	/* Wait until STOPF flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_STOPF) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}   

	/* Clear STOPF flag */
	I2C_ClearFlag(I2C_DEVICE, I2C_ICR_STOPCF);

	/* If all operations OK, return I2C_DEVICE_OK (0) */
	return I2C_DEVICE_success;

}


I2C_DEVICE_MESSAGE I2C_DEVICE_Read(uint16_t Address,uint16_t DeviceAddr,uint8_t* pBuffer,uint8_t NumByteToWrite)
{
//	uint8_t I2C_DEVICE_BufferRX[2] ={0,0}; 

	/* Test on BUSY Flag */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_BUSY) != RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(I2C_DEVICE, DeviceAddr, 2, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TXIS) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Send MSB of memory address */
	I2C_SendData(I2C_DEVICE, (uint8_t)((Address & 0xFF00) >> 8));

	/* Wait until TXIS flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;  
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TXIS) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Send LSB of memory address  */
	I2C_SendData(I2C_DEVICE, (uint8_t)(Address & 0x00FF));

	/* Wait until TC flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_TC) == RESET)
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}  

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(I2C_DEVICE, DeviceAddr, 1, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	/* Wait until RXNE flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;  
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_RXNE) == RESET)  
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Read data from RXDR */
	*pBuffer= I2C_ReceiveData(I2C_DEVICE);  

	/* Wait until STOPF flag is set */
	DEVICE_Timeout = I2C_DEVICE_LONG_TIMEOUT;
	while(I2C_GetFlagStatus(I2C_DEVICE, I2C_ISR_STOPF) == RESET) 
	{
		if((DEVICE_Timeout--) == 0) return I2C_DEVICE_TIMEOUT_UserCallback();
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(I2C_DEVICE, I2C_ICR_STOPCF);

//	/* Return Register value */
//	return (uint8_t)I2C_DEVICE_BufferRX[0];

	return I2C_DEVICE_success;

}

void I2C_DEVICE_Setup(void)
{

}


uint8_t ReadByte(uint16_t Address)
{
	uint8_t Data;
	I2C_DEVICE_Read(Address,DeviceAddress,(uint8_t*)&Data,1);
	return Data;
}

void WriteByte(uint16_t Address,uint8_t Val)
{
	uint8_t Data[1]={0};
	Data[0] = Val;
	I2C_DEVICE_Write(Address,DeviceAddress,(uint8_t*)&Data,1);
}

