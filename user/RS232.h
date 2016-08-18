/**
	********************************************************************
	*	@file			RS232.h
	*	@author		Jun.Lu
	*	@version	V1.0.0
	*	@date			13-May-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	The header of RS232.c.
	*-----------------------------
	*
	*	Change Logs
	*	Date							Author						Notes
	*	2013-5-13					Jun.Lu						Version V1.0.0
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <stm32f4xx.h>
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
typedef enum
{
	SYNC_FLAG=0,
	PROTOCOL_VERSION,
	MESSAGE_LENGTH_LOW,
	MESSAGE_LENGTH_HIGH,
	LENGTH_CHECKSUM,
	TOPIC_ID_LOW,
	TOPIC_ID_HIGH,
	DATA_GET,
	OVERALL_CHECKSUM,
	ACK
	
}RECEIVE_STATES;
typedef enum
{
	LENGTH=0,
	OVERALL
}CHECK_STATE;
typedef struct
{
	uint8_t topic_id[2];
	uint8_t cmd_length;
	uint8_t cmd_data[20];
	
}RECEIVE_DATA;
						
/*	Private define -------------------------------------------------*/
#define HEAD_DATA_READ_OK   					(1<<4)
#define LOWER_DATA_READ_OK   				  (1<<5)

/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/







/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
