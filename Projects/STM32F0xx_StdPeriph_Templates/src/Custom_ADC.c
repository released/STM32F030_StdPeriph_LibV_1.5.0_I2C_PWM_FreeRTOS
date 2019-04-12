/* Includes ------------------------------------------------------------------*/
#include "Custom_ADC.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*ADC variable*/
#define ADC_VREF						3300	//3300
#define ADC_BITFACTOR					0xFFF
#define ADC_RESOLUTION					ADC_Resolution_12b

#define ADC1_SAMPLE					(16)
#define ADC1_CH						(2)

//ADC variable
__IO uint16_t ADC1_Value[ADC1_SAMPLE][ADC1_CH];
__IO uint16_t After_filter1[ADC1_CH];

/* Private functions ---------------------------------------------------------*/

void Custom_ADC1_Voltage(void)
{
	#if 0
	uint16_t CH1,CH2=0;
	CH1 = (After_filter[0] *ADC_VREF)/ADC_BITFACTOR;
	//CH2 = (After_filter[1] *ADC_VREF)/ADC_BITFACTOR;	

	printf("CH1 : %4d (0x%4X) , CH2 : %4d (0x%4X)\r\n",CH1,CH1,CH2,CH2);
	#endif

	uint16_t i=0;
	uint16_t CH=0;
	
	for (i=0;i<ADC1_CH;i++)
	{
		CH = (After_filter1[i] *ADC_VREF)/ADC_BITFACTOR;	

		#if 0		//debug only
		printf("ADC1:CH%2d:%4dmv(0x%3X),",i,CH,CH);
		if (i==(ADC1_CH-1))
		{
			printf("\r\n");
		}
		#endif
	}	
}

void Custom_ADC1_Filter(void)
{
	uint32_t  	sum = 0;
	uint8_t 	count,i;

	for(i=0;i<ADC1_CH;i++)
	{
		for ( count=0;count<ADC1_SAMPLE;count++)
		{
		   sum += ADC1_Value[count][i];
		}
		
		// After_filter1[i]= sum>>4;	//sum/ADC1_SAMPLE;
		After_filter1[i] = (sum + (ADC1_SAMPLE >> 1)) >>4;	
		sum=0;
	} 
}

/*
	ADC_IN1		PA1
	ADC_IN2		PA2

*/
void Custom_ADC1_DMA_Config(void)	
{
	DMA_InitTypeDef   	DMA_InitStructure;
	ADC_InitTypeDef     ADC_InitStructure;
	GPIO_InitTypeDef    GPIO_InitStructure;
	NVIC_InitTypeDef 	NVIC_InitStructure;
	
	/* ADC1 DeInit */  
	ADC_DeInit(ADC1);

	/* GPIOC Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* ADC1 Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA1 clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

	/* Configure ADC Channel11 and channel10 as analog input */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR ;//ADC1_DR_Address , 0x40012440
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1_Value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = (ADC1_SAMPLE)*(ADC1_CH);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	#if 1	// Enable the DMA Interrupt 
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE); 
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);          
	#endif
	
	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);
  
	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);

	/* Configure the ADC1 in continuous mode withe a resolution equal to 12 bits  */
	ADC_InitStructure.ADC_Resolution = ADC_RESOLUTION;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; 
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;	
	ADC_Init(ADC1, &ADC_InitStructure); 

	/* Convert the ADC1 Channel11 and channel10 with 55.5 Cycles as sampling time */ 
	ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_55_5Cycles); 
	ADC_ChannelConfig(ADC1, ADC_Channel_2 , ADC_SampleTime_55_5Cycles); 
	
	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* ADC DMA request in circular mode */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable ADC_DMA */
	ADC_DMACmd(ADC1, ENABLE);  

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);     

	/* Wait the ADRDY flag */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY)); 
	
	/* ADC1 regular Software Start Conv */ 
	ADC_StartOfConversion(ADC1);
}




