/**
********************************************************************
*	@file		stm32f4_TIMER_Drv.h
*	@author		Jun.Lu
*	@version	V1.0
*	@date		16-Aug-2016
*	@brief		
********************************************************************
*	@attention
*	This file is the header of low level driver of gpio init.
*-----------------------------
*
*	Change Logs:
*	Date					Author					Notes
*	2015-3-4				Jun.Lu					Version V1.0.0
*
********************************************************************
*/
#ifndef __TIMER_DRV_H__
#define __TIMER_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "stm32f4xx.h"
#include "lw_oopc.h"
	
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
enum TIMER_NUM
{
	TIMER_1=1,
	TIMER_2,
	TIMER_3,
	TIMER_4,
	TIMER_5,
	TIMER_6,
	TIMER_7,
	TIMER_8,
	TIMER_9,
	TIMER_10,
	TIMER_11,
	TIMER_12,
	TIMER_13,
	TIMER_14
};

enum COUTER_MODE
{
	UP=0x00,
	DOWN,
	CENTER_1,
	CENTER_2,
	CENTER_3
};

enum CLK_DIV
{
	DIV_1=0x00,
	DIV_2,
	DIV_4
};

enum OC_Mode
{
	TIMING=0,
	ACTIVE,
	INACTIVE,
	TOGGLE,
	PWM1,
	PWM2
};

enum TIMER_TYPE
{
	ADVANCED=0,
	GENERAL,
	BASIC
};

enum CHANNEL_NUM
{
	CH_1=0,
	CH_2,
	CH_3,
	CH_4
};
	


CLASS(Simple_Timer){
	void (*simple_init)(void*, uint8_t Timer_Num, uint8_t Channel_Num, uint32_t Timer_Freq, uint16_t Period, uint16_t Pulse);
	void (*simple_set_duty)(void*, uint8_t Timer_Num, uint8_t Channel_Num, uint16_t Duty);
	uint8_t timer_num;
	uint8_t channel_num;
	uint16_t duty;
	uint16_t pulse;//ccr
	uint16_t period;
	uint32_t timer_freq;
};
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/

/*	Private function prototypes ------------------------------------*/


/*	Private functions -----------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /*__TIMER_DRV_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
