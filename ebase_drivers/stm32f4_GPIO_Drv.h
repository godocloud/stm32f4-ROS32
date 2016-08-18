/**
********************************************************************
*	@file		stm32f4_GPIO_Drv.h
*	@author		Jun.Lu
*	@version	V1.0.0
*	@date		4-March-2015
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
#ifndef __GPIO_DRV_H__
#define __GPIO_DRV_H__

#ifdef __cplusplus
extern "C" {
#endif

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "stm32f4xx.h"
#include "lw_oopc.h"
	
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
enum A_TO_I
{
	A=0,
	B,
	C,
	D,
	E,
	F,
	G,
	H,
	I
};

enum GPIO_MODE
{
	IN=0x00,
	OUT,
	AF,
	AN
};

enum GPIO_OTYPE
{
	PP=0x00,
	OD
};

enum GPIO_SPEED
{
	SPD_2=0x00,
	SPD_25,
	SPD_50,
	SPD_100
};

enum GPIO_PUPD
{
	NO=0x00,
	PULLUP,
	PULLDOWN
};

CLASS(Gpio){
	uint8_t GPIOx;
	uint8_t Pinx;
	uint8_t pinState;
	void (*init)(uint8_t GPIOx,uint8_t Pinx,uint8_t OUT_IN_AF_AN, uint8_t PP_OD,uint8_t PullUp_PullDown);
	void (*pinWrite)(uint8_t GPIOx,uint8_t Pinx,uint8_t value);
	int (*pinRead)(uint8_t GPIOx,uint8_t Pinx);
	void (*pinToggle)(uint8_t GPIOx,uint8_t Pinx);
};
/*	Private define -------------------------------------------------*/
#define ALL						16
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
void GPIO_Pin_Setup(uint8_t GPIOx,uint8_t Pinx,uint8_t mode, uint8_t otype,uint8_t pupd);
void GPIO_Pin_Write(uint8_t GPIOx,uint8_t Pinx,uint8_t value);
int GPIO_Pin_Read(uint8_t GPIOx,uint8_t Pinx);
void GPIO_Pin_Toggle(uint8_t GPIOx,uint8_t Pinx);

/*	Private functions -----------------------------------------------*/




#ifdef __cplusplus
}
#endif

#endif /*__GPIO_DRV_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
