/**
	********************************************************************
	*	@file			wifi_module.h
	*	@author		Jun.Lu
	*	@version 	beta	
	*	@date			10-Jun-2014
	*	@brief		This file is the header of wifi_module.c
	********************************************************************
	*	@attention
	*	
	*-------------------------------------------------------------------
	* Pins definition:
	*	MOTORs:
	*	PIN					FUNCTION				NOTES
	*	PB1					GPIO					WIFI STATUS
	*	PE11				GPIO					WIFI BOOT
	*	PE12 				GPIO					WIFI_RESET
	*
	*-------------------------------------------------------------------
	*	Change Logs:
	*	Date					Author					Notes
	*	2014-6-10			Jun.Lu					beta
	* 
	********************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __WIFI_MODULE_H__
#define __WIFI_MODULE_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
/*	Includes -------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_GPIO_Drv.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -----------------------------------------------*/

#define WIFI_CTRL_PORT            		GPIOE
#define WIFI_CTRL_CLK             		RCC_AHB1Periph_GPIOE

#define WIFI_STATUS_PIN					GPIO_Pin_13//PE13
#define WIFI_BOOT_CONNCT_PIN			GPIO_Pin_11//PE11
#define WIFI_RESET_PIN 	    			GPIO_Pin_12//PE12

#define WIFI_SETUP_PIN					GPIO_Pin_14//PE14
															
#define WIFI_STATUS_H() 				GPIO_Pin_Write(E,13,1)

#define WIFI_STATUS_L() 				GPIO_Pin_Write(E,13,0)
																																
#define WIFI_RESET_H() 					GPIO_Pin_Write(E,12,1)

#define WIFI_RESET_L() 					GPIO_Pin_Write(E,12,0)

/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /*__WIFI_MODULE_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/

