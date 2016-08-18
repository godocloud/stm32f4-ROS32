/**
	********************************************************************
	*	@file			comm_overtime.h
	*	@author		Jun.Lu
	*	@version	V1.1.0
	*	@date			8-Oct-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	The header of comm_overtime.c.
	*-----------------------------
	*
	*	Change Logs:
	*	Date							Author						Notes
	*	2013-10-8					Jun.Lu						First Implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
#define PACKAGE_SIZE_MAX						16
#define ID_BUFFER_SIZE							20
#define SAVED_BUFFER_SIZE						1024
typedef struct
{
		uint16_t package_id;
		uint8_t flag;
		uint8_t delay_counter;
		uint8_t structure_id;
		uint8_t resend_count;
}ID_BUFFER;

typedef enum
{
		ACK_CLEAR=0,
		CMD_SEND,
		ACK_RECEIVED
}ACK_STATES;
						
/*	Private define -------------------------------------------------*/

/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/







/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
