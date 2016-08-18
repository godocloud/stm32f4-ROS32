/**
********************************************************************
*	@file		stm32f4_TIMER_Drv.c
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
#include "stm32f4_TIMER_Drv.h"
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/

/*	Private function prototypes ------------------------------------*/
/*	Private functions -----------------------------------------------*/
TIM_TypeDef* TIM[15]={NULL,TIM1,TIM2,TIM3,TIM4,TIM5,TIM6,TIM7,TIM8,
						TIM9,TIM10,TIM11,TIM12,TIM13,TIM14};
uint16_t Counter_Mode[5]={TIM_CounterMode_Up, TIM_CounterMode_Down,
						TIM_CounterMode_CenterAligned1,
						TIM_CounterMode_CenterAligned2,
						TIM_CounterMode_CenterAligned3};
uint16_t Clk_Div[3]={TIM_CKD_DIV1, TIM_CKD_DIV2, TIM_CKD_DIV4};
uint16_t OC_Mode[6]={TIM_OCMode_Timing,TIM_OCMode_Active,
						TIM_OCMode_Inactive,TIM_OCMode_Toggle,
						TIM_OCMode_PWM1,TIM_OCMode_PWM2};
uint16_t Channel[4]={TIM_Channel_1,TIM_Channel_2,TIM_Channel_3,TIM_Channel_4};

/**
*	@brief	using for what
*	@note		description
*	@param	
*	@retval	
*/
void TIMER_CLK_Init(uint8_t Timer_Num)
{
	RT_ASSERT(Timer_Num<=14);
	
	switch (Timer_Num)
	{
	case TIMER_1:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		break;
	case TIMER_2:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		break;
	case TIMER_3:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
		break;
	case TIMER_4:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		break;
	case TIMER_5:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		break;
	case TIMER_6:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
		break;
	case TIMER_7:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
		break;
	case TIMER_8:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
		break;
	case TIMER_9:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
		break;
	case TIMER_10:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
		break;
	case TIMER_11:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
		break;
	case TIMER_12:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
		break;
	case TIMER_13:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);
		break;
	case TIMER_14:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
		break;
	default:
		break;
	}
}

void TIMER_Base_Setup(uint8_t Timer_Num, uint32_t Timer_Freq, uint16_t Div, uint8_t Mode,uint16_t Period)
{
	uint16_t PrescalerValue = 0;
	
	TIM_TimeBaseInitTypeDef  			TIM_TimeBaseStructure;
	
	RT_ASSERT(Timer_Num<=14);
	RT_ASSERT(Timer_Freq<=42000000);
	RT_ASSERT(Div<=3);
	RT_ASSERT(Mode<=5);
	RT_ASSERT(Period<=0xFFFF);
	
	TIMER_CLK_Init(Timer_Num);
	
	PrescalerValue=(uint16_t)((SystemCoreClock /2)/Timer_Freq)-1;	
	//timer base config
	TIM_TimeBaseStructure.TIM_Period = Period;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = Clk_Div[Div];
	TIM_TimeBaseStructure.TIM_CounterMode = Counter_Mode[Mode];
	TIM_TimeBaseInit(TIM[Timer_Num], &TIM_TimeBaseStructure);
}

void TIMER_OC_Func_Setup(uint8_t Timer_Num, uint8_t Channel_Num, uint8_t OCMode, uint8_t Timer_Type, uint16_t Pulse)
{
	TIM_OCInitTypeDef  					TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCMode = OCMode;
	TIM_OCInitStructure.TIM_Pulse = Pulse;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	if(Timer_Type==ADVANCED)
	{
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	}
	switch(Channel_Num)
	{
		case CH_1:
			TIM_OC1Init(TIM[Timer_Num], &TIM_OCInitStructure);
			TIM_OC1PreloadConfig(TIM[Timer_Num],TIM_OCPreload_Enable);
			break;
		case CH_2:
			TIM_OC2Init(TIM[Timer_Num], &TIM_OCInitStructure);
			TIM_OC2PreloadConfig(TIM[Timer_Num],TIM_OCPreload_Enable);
			break;
		case CH_3:
			TIM_OC3Init(TIM[Timer_Num], &TIM_OCInitStructure);
			TIM_OC3PreloadConfig(TIM[Timer_Num],TIM_OCPreload_Enable);
			break;
		case CH_4:
			TIM_OC4Init(TIM[Timer_Num], &TIM_OCInitStructure);
			TIM_OC4PreloadConfig(TIM[Timer_Num],TIM_OCPreload_Enable);
			break;
		default:break;	
	}

	//preload config
	TIM_ARRPreloadConfig(TIM[Timer_Num], ENABLE);
	/* TIMx enable counter */
	TIM_Cmd(TIM[Timer_Num], ENABLE);

}

void TIMER_PWM_Simple_Setup(void *t, uint8_t Timer_Num, uint8_t Channel_Num, uint32_t Timer_Freq, uint16_t Period, uint16_t Pulse)
{
	Simple_Timer* cthis=(Simple_Timer*)t;
	cthis->timer_num=Timer_Num;
	cthis->channel_num=Channel_Num;
	cthis->timer_freq=Timer_Freq;
	cthis->period=Period;
	cthis->pulse=Pulse;
	
	TIMER_Base_Setup(Timer_Num, Timer_Freq, DIV_1, UP, Period);
	TIMER_OC_Func_Setup(Timer_Num, Channel_Num, PWM1, ADVANCED, Pulse);
	TIM_CtrlPWMOutputs(TIM[Timer_Num], ENABLE);
}

void TIMER_Simple_Set_Duty(void *t, uint8_t Timer_Num, uint8_t Channel_Num, uint16_t Duty)
{
	Simple_Timer* cthis=(Simple_Timer*)t;
	
	cthis->timer_num=Timer_Num;
	cthis->channel_num=Channel_Num;
	cthis->duty=Duty;

	if(Duty==0)//0=off
	{
		TIM_CCxCmd(TIM[Timer_Num],Channel[Channel_Num], DISABLE);
		TIM_CCxNCmd(TIM[Timer_Num],Channel[Channel_Num], DISABLE);
	}
	else
	{
		TIM_CCxCmd(TIM[Timer_Num],Channel[Channel_Num], ENABLE);
		switch(Channel_Num)
		{
			case CH_1:
				TIM_SetCompare1(TIM[Timer_Num],Duty);
				break;
			case CH_2:
				TIM_SetCompare2(TIM[Timer_Num],Duty);
				break;
			case CH_3:
				TIM_SetCompare3(TIM[Timer_Num],Duty);
				break;
			case CH_4:
				TIM_SetCompare4(TIM[Timer_Num],Duty);
				break;
			default:break;
		}
		TIM_Cmd(TIM[Timer_Num], ENABLE);
	}
}


//ππ‘ÏGpio¿‡
CTOR(Simple_Timer)
	FUNCTION_SETTING(simple_init, TIMER_PWM_Simple_Setup);
	FUNCTION_SETTING(simple_set_duty, TIMER_Simple_Set_Duty);
END_CTOR

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
