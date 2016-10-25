#include <stddef.h>
#include "stm32l1xx.h"
#include "vrs_cv5.h"
#include<string.h>

GPIO_InitTypeDef gpioInitStruc;
NVIC_InitTypeDef NVIC_InitStructure;
ADC_InitTypeDef ADC_InitStructure;
USART_InitTypeDef USART_InitStructure;

extern uint16_t value;
extern uint8_t zmena;
extern uint16_t value2;


void gpio_init(void)
{
	          /* LED */
	          RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

			  //GPIO_InitTypeDef gpioInitStructure;
	          GPIO_StructInit(&gpioInitStruc);
			  gpioInitStruc.GPIO_Mode = GPIO_Mode_OUT;
			  gpioInitStruc.GPIO_OType = GPIO_OType_PP;
			  gpioInitStruc.GPIO_Pin = GPIO_Pin_5;
			  gpioInitStruc.GPIO_Speed = GPIO_Speed_400KHz;
			  GPIO_Init(GPIOA, &gpioInitStruc);

			  /* ADC */
			  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
			  GPIO_StructInit(&gpioInitStruc);
			  gpioInitStruc.GPIO_Pin = GPIO_Pin_0 ;
			  gpioInitStruc.GPIO_Mode = GPIO_Mode_AN;
			  gpioInitStruc.GPIO_PuPd = GPIO_PuPd_NOPULL ;
			  GPIO_Init(GPIOC, &gpioInitStruc);

			  /* usart */
			  GPIO_StructInit(&gpioInitStruc);
			  gpioInitStruc.GPIO_Mode=GPIO_Mode_AF;
			  gpioInitStruc.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
			  gpioInitStruc.GPIO_Speed=GPIO_Speed_40MHz;
			  GPIO_Init(GPIOA,&gpioInitStruc);

			  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
			  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
}

void adc_init()
{
    ADC_InitTypeDef ADC_InitStructure;
	/* Enable GPIO clock */

	/*
	 * RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;

	GPIO_Init(GPIOC, &GPIO_InitStructure); */

	/* Enable ADC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Enable the HSI oscillator */
	RCC_HSICmd(ENABLE);

	/* Check that HSI oscillator is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	/* Initialize ADC structure */
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADCx regular channel8 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_16Cycles);

	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	ADC_ITConfig(ADC1,ADC_IT_OVR,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitStructure.NVIC_IRQChannel=ADC1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);

    /* konverzia */
	ADC_Cmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
	ADC_SoftwareStartConv(ADC1);
}

void usart_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2,&USART_InitStructure);

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel=USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART2,ENABLE);
}

void ADC1_IRQHandler()
{
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
		{
			value = ADC_GetConversionValue(ADC1);
			value2 = value*3300/4095;
		}
	if(ADC_GetFlagStatus(ADC1,ADC_FLAG_OVR))
		{
			ADC_ClearFlag(ADC1,ADC_FLAG_OVR);
		}
}

void Posielanie(char *hodnota)
{
	uint8_t x=0;
	while(strlen(hodnota) > x)
	{
		USART_SendData(USART2,hodnota[x]);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
		x++;
	}
	USART_SendData(USART2,'\n');
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
	USART_SendData(USART2,'\r');
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)==RESET);
}


void USART2_IRQHandler()
{
	if(USART_GetFlagStatus(USART2,USART_FLAG_RXNE))
	{
		if (USART_ReceiveData(USART2)=='m') zmena = 1;

		if (USART_ReceiveData(USART2) != 'm') zmena = 0;

	}
}

void delay(uint32_t time)
{
	uint32_t i;
	for(i=0;i<time;i++)
	{
	}
}
