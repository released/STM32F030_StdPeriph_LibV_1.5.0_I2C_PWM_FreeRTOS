/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(void)  
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)  
#endif /* __GNUC__ */

// TODO: for printf function , need to confirm use USART1 or USART2
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

//GETCHAR_PROTOTYPE
//{
//	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE == RESET));
//	return (USART_ReceiveData(USART1));
//}

GETCHAR_PROTOTYPE
{
    int ch;
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {
    }
    ch = USART_ReceiveData(USART1);
   
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t) ch);
    return ch;
}

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef RCC_Clocks;

uint8_t 			u8Counter = 0;

/* Private functions ---------------------------------------------------------*/

void TIM1_PWM_Config(void)	//PA9,TIM1_CH2:100Hz
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	uint16_t TimerPeriod = 0;		
	uint16_t Channel4Pulse = 0;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);

	/*************************************************************************
		TIMx_ARR = TIM_Period , Auto-Reload Register
		TIMx_CCR1 = TIM_Pulse , Capture Compare Register

		** Prescaler = (TIMxCLK / TIMx counter clock) - 1
		
		** Timer clock = PWM frequency * 2^PWM resolution

		The objective is to generate PWM signal at xx KHz:
		** TIMx Frequency = TIMx counter clock/(TIM_Period + 1)
			-> 1/ TIMx Frequency = (TIM_Period + 1) * (TIM_Prescaler + 1) / TIMxCLK
			-> TIMx Frequency =  TIMxCLK / ((TIM_Period + 1) * (TIM_Prescaler + 1))
				=> example : 
					Freq = 100 Hz = TIMxCLK / ((TIM_Period + 1) * (TIM_Prescaler + 1))
								  = 48000000 / (4000-1     + 1) * (120 -1        + 1))
			
		** TIMx_Period = (SystemCoreClock / TIMx Frequency) - 1 , if prescaler = 1-1

		** duty cycle = (TIM_Pulse/ TIM_Period + 1)* 100
		** ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100	
	*************************************************************************/

	TimerPeriod = 480 - 1;
	/* Compute CCR1 value to generate a duty cycle at 50% for channel 1 */
	Channel4Pulse = (uint16_t) (((uint32_t) 50 * (TimerPeriod - 1)) / 100);

	/* Time Base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 1000 -1 ;
	TIM_TimeBaseStructure.TIM_Period = TimerPeriod;	
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 1, 2,3 and 4 Configuration in PWM mode */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);

	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
}


void USART_Test(void)
{
//	uint32_t i=0;
//	uint32_t j=0;
//	uint32_t displaycounter=0;
	__IO uint8_t temp;
//	uint16_t Color = 0 ;	
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	{
			temp = USART_ReceiveData(USART1);
			printf("temp = 0x%x \n\r",temp);

			switch (temp)
			{

			}
	}
}

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USARTx configured as follows:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	#if 1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
	
	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

//	printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");

}

uint32_t Button_GetState(void)
{
  return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
}

void Button_Config_EXTI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the BUTTON Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Button pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 	
}

void Button_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the BUTTON Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Button pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void LED_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SysTickConfig(void)
{
	/* This function fills the RCC_ClockFreq structure with the current
	 frequencies of different on chip clocks (for debug purpose) */	
	RCC_GetClocksFreq(&RCC_Clocks);

	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d Hz\n\r",RCC_Clocks.SYSCLK_Frequency);	
	printf("HCLK = %d Hz\n\r",RCC_Clocks.HCLK_Frequency);
	printf("PCLK = %d Hz\n\r",RCC_Clocks.PCLK_Frequency);
	printf("I2C1CLK = %d Hz\n\r",RCC_Clocks.I2C1CLK_Frequency);
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t mTime)
{ 
	#if !defined (SIMPLEDELAY)
	uwTimingDelay = mTime;
	while(uwTimingDelay != 0);
	#else
    /* Decrement nCount value */
    while (mTime != 0)
    {
        mTime--;
    }
	#endif
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/
uint8_t UART_GetByte(void)
{
	while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	{
	}
	return (uint8_t)USART_ReceiveData(USART1);
}

void UART_SendByte(uint8_t Data)
{
	USART_SendData(USART1 , (unsigned char)Data);
	while (USART_GetFlagStatus(USART1 , USART_FLAG_TC)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	#if 1
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
	#else	//ignore len
    while(*Data)  
    {  
        USART_SendData(USART1, (unsigned char) *Data++);  
        /* Loop until the end of transmission */  
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  //USART_FLAG_TXE
    } 
	#endif
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


