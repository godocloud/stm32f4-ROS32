/**
********************************************************************
*	@file		stm32f4_GPIO_Drv.c
*	@author		Jun.Lu
*	@version	V1.0.0
*	@date		4-March-2015
*	@brief		
********************************************************************
*	@attention
*	This file is the low level driver of gpio init.
*-----------------------------
*
*	Change Logs:
*	Date					Author					Notes
*	2015-3-4				Jun.Lu					Version V1.0.0
*
********************************************************************
*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "stm32f4_GPIO_Drv.h"
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
GPIO_TypeDef* GPIO_PORT[9]={GPIOA,GPIOB,GPIOC,GPIOD,GPIOE,GPIOF,GPIOG,GPIOH,GPIOI};
uint32_t GPIO_PIN[17]={GPIO_Pin_0,GPIO_Pin_1,GPIO_Pin_2,GPIO_Pin_3,
	GPIO_Pin_4,GPIO_Pin_5,GPIO_Pin_6,GPIO_Pin_7,
	GPIO_Pin_8,GPIO_Pin_9,GPIO_Pin_10,GPIO_Pin_11,
	GPIO_Pin_12,GPIO_Pin_13,GPIO_Pin_14,GPIO_Pin_15,
	GPIO_Pin_All};
GPIOMode_TypeDef GPIO_MODE[4]={GPIO_Mode_IN,GPIO_Mode_OUT,GPIO_Mode_AF,GPIO_Mode_AN};
GPIOOType_TypeDef GPIO_OTYPE[2]={GPIO_OType_PP,GPIO_OType_OD};
GPIOPuPd_TypeDef GPIO_PUPD[3]={GPIO_PuPd_NOPULL,GPIO_PuPd_UP,GPIO_PuPd_DOWN};

/*	Private function prototypes ------------------------------------*/
/*	Private functions -----------------------------------------------*/

/**
*	@brief	using for what
*	@note		description
*	@param	
*	@retval	
*/
static void GPIO_CLK_Init(uint8_t A_TO_I)
{
	RT_ASSERT(A_TO_I<=8);
	
	switch (A_TO_I)
	{
	case A:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
		break;
	case B:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		break;
	case C:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		break;
	case D:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
		break;
	case E:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
		break;
	case F:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
		break;
	case G:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
		break;
	case H:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
		break;
	case I:
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
		break;
	default:
		break;
	}
}

void GPIO_Pin_Setup(uint8_t GPIOx,uint8_t Pinx,uint8_t OUT_IN_AF_AN, uint8_t PP_OD,uint8_t PullUp_PullDown)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_TypeDef *GPIO_PORT_X;

	RT_ASSERT(GPIOx<=8);
	RT_ASSERT(Pinx<=17);
	RT_ASSERT(OUT_IN_AF_AN<=3);
	RT_ASSERT(PP_OD<=1);
	RT_ASSERT(PullUp_PullDown<=2);
	
	GPIO_PORT_X = GPIO_PORT[GPIOx];
	GPIO_CLK_Init(GPIOx);
	GPIO_InitStructure.GPIO_Mode  = GPIO_MODE[OUT_IN_AF_AN];
	GPIO_InitStructure.GPIO_OType = GPIO_OTYPE[PP_OD];
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PUPD[PullUp_PullDown];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Pinx];
	GPIO_Init(GPIO_PORT_X, &GPIO_InitStructure);
}

void GPIO_Pin_Write(uint8_t GPIOx,uint8_t Pinx,uint8_t value)
{
	GPIO_TypeDef *GPIO_PORT_X;
	GPIO_PORT_X = GPIO_PORT[GPIOx];

	RT_ASSERT(GPIOx<=8);
	RT_ASSERT(Pinx<=17);
	RT_ASSERT(value==1||value==0);
	
	if (value==1)
	{
		GPIO_SetBits(GPIO_PORT_X,GPIO_PIN[Pinx]);
	}
	else if(value==0)
	{
		GPIO_ResetBits(GPIO_PORT_X,GPIO_PIN[Pinx]);
	}
}

int GPIO_Pin_Read(uint8_t GPIOx,uint8_t Pinx)
{
	int value;
	GPIO_TypeDef *GPIO_PORT_X;
	GPIO_PORT_X = GPIO_PORT[GPIOx];
	
	RT_ASSERT(GPIOx<=8);
	RT_ASSERT(Pinx<=17);

	if (Pinx!=ALL)
	{
		if((((GPIO_PORT_X->MODER)>>(2*Pinx))&0x03)==0x00)
		{
			if (GPIO_ReadInputDataBit(GPIO_PORT_X, GPIO_PIN[Pinx]) == Bit_RESET)
			{
				value = 0x00;
			}
			else
			{
				value = 0x01;
			}
		}
		else if (((GPIO_PORT_X->MODER>>(2*Pinx))&0x03)==0x01)
		{
			if (GPIO_ReadOutputDataBit(GPIO_PORT_X, GPIO_PIN[Pinx]) == Bit_RESET)
			{
				value = 0x00;
			}
			else
			{
				value = 0x01;
			}
		}
	}
	else
	{
		if((GPIO_PORT_X->MODER)==0x00)
		{
			if (GPIO_ReadInputData(GPIO_PORT_X) == Bit_RESET)
			{
				value = 0x00;
			}
			else
			{
				value = 0x01;
			}
		}
		else
		{
			return -1;
		}
		return -1;
	}

	return value;
}

void GPIO_Pin_Toggle(uint8_t GPIOx,uint8_t Pinx)
{
	GPIO_TypeDef *GPIO_PORT_X;
	
	RT_ASSERT(GPIOx<=8);
	RT_ASSERT(Pinx<=17);
	
	GPIO_PORT_X = GPIO_PORT[GPIOx];
	
	GPIO_ToggleBits(GPIO_PORT_X,GPIO_PIN[Pinx]);
}

//¹¹ÔìGpioÀà
CTOR(Gpio)
	FUNCTION_SETTING(init, GPIO_Pin_Setup);
	FUNCTION_SETTING(pinWrite, GPIO_Pin_Write);
	FUNCTION_SETTING(pinRead, GPIO_Pin_Read);
	FUNCTION_SETTING(pinToggle, GPIO_Pin_Toggle);
END_CTOR

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
