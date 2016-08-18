/**
	********************************************************************
	*	@file			pulse.h
	*	@author			Jun.Lu
	*	@version 		alpha	
	*	@date			1-Feb-2015
	*	@brief			This file is the header of pulse.c
	********************************************************************
	*	Change Logs:
	*	Date					Author					Notes
	*	2016-2-1				Jun.Lu					first implement
	* 
	********************************************************************
	*/

/* Define to prevent recursive inclusion -----------------------------*/
#ifndef __PULSE_H__
#define __PULSE_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
/*	Includes ---------------------------------------------------------*/
#include "stm32f4xx.h"
#include "lw_oopc.h"
#include "stm32f4_GPIO_Drv.h"
#include "stm32f4_TIMER_Drv.h"

/*	Private typedef ------------------------------------------------*/
extern void* GpioNew();
extern void* Simple_TimerNew();
	 
enum MOTO_STATE
{
	STOP=0,
	CW,
	CCW
};	

CLASS(Motor_tb6612){
	void (*hw_init)(void*, uint8_t pwm_gpiox, uint8_t pwm_pinx,
                    uint8_t in1_gpiox, uint8_t in1_pinx,
                    uint8_t in2_gpiox, uint8_t in2_pinx);
    void (*hw_timer_init)(void*, uint8_t timer_num, uint8_t channel_num,
                       uint32_t timer_freq, uint16_t period,
                       uint16_t pulse);
	void (*init)(void*);
	void (*set_duty)(void*, uint16_t duty);
	uint16_t (*get_duty)(void*);
	void (*update_state)(void*, uint8_t state);
	uint8_t (*get_state)(void*);
	uint8_t state;
	uint16_t duty;
    Simple_Timer* ad_timer;
	Gpio* pwmPin;
    Gpio* in1Pin;
	Gpio* in2Pin;
};

/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /*__PULSE_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
