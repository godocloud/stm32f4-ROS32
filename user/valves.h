/**
	********************************************************************
	*	@file			valves.h
	*	@author			Jun.Lu
	*	@version 		alpha	
	*	@date			3-Feb-2015
	*	@brief			This file is the header of valves.c
	********************************************************************
	*	Change Logs:
	*	Date					Author					Notes
	*	2016-2-3				Jun.Lu					first implement
	* 
	********************************************************************
	*	
	* 
	*/

/* Define to prevent recursive inclusion -----------------------------*/
#ifndef __VALVES_H__
#define __VALVES_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
/*	Includes ---------------------------------------------------------*/
#include "stm32f4xx.h"
#include "lw_oopc.h"
#include "stm32f4_GPIO_Drv.h"

/*	Private typedef ------------------------------------------------*/ 
extern void* GpioNew();
	 
CLASS(DRV8860){
	void (*hw_init)(void*, uint8_t Gpiox, uint8_t enPinx, uint8_t nfaultPinx, 
					uint8_t doutPinx, uint8_t latchPinx, uint8_t clkPinx,
					uint8_t dinPinx);
	void (*init)(void*);
	void (*writeData)(void*, uint8_t data);
	uint16_t (*readData)(void*);
	void (*reset)(void*);
	void (*en)(void*,uint8_t state);
	uint16_t faultValue;
	uint8_t cmd;
	Gpio* EnPin;
	Gpio* nFaultPin;
	Gpio* DoutPin;
	Gpio* LatchPin;
	Gpio* ClkPin;
	Gpio* DinPin;
};

enum{
	AIR_HEAD=0x00,
	AIR_L_LUNG,
	AIR_PNEU,
	AIR_R_LUNG,
	BLOOD_HEAD,
	BLOOD_HEMO,
	BLOOD_1,
	BLOOD_2
}VALVES;
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /*__VALVES_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
