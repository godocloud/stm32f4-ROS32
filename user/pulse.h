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

/*	Private typedef ------------------------------------------------*/
extern void* GpioNew();
	 
CLASS(Pulse){
	void (*hw_init)(void*, uint8_t touchGpiox, uint8_t touchPinx, uint8_t beatGpiox, uint8_t beatPinx);
	void (*init)(void*);
	void (*setPulse)(void*, uint8_t power, uint8_t rate);
	void (*beat)(void*);
	uint8_t (*getPulseState)(void*);
	uint8_t power;
	uint16_t rate;
	uint8_t state;
	Gpio* Touchpin;
	Gpio* Beatpin;
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
