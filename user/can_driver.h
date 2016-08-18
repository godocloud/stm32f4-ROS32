/**
	********************************************************************
	*	@file			can_driver.h
	*	@author			Jun.Lu
	*	@version 		beta	
	*	@date			10-Jun-2014
	*	@brief			This file is the header of can_driver.c
	********************************************************************
	*	Change Logs:
	*	Date					Author					Notes
	*	2016-1-14				Jun.Lu					first implement
	* 
	********************************************************************
	*/

/* Define to prevent recursive inclusion -----------------------------*/
#ifndef __CAN_DRIVER_H__
#define __CAN_DRIVER_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
/*	Includes ---------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_GPIO_Drv.h"
/*	Global variables -------------------------------------------------*/
/*	Private typedef --------------------------------------------------*/
typedef struct
{
	uint32_t StdId;  /*!< Specifies the standard identifier.
					This parameter can be a value between 0 to 0x7FF. */

	uint32_t ExtId;  /*!< Specifies the extended identifier.
					This parameter can be a value between 0 to 0x1FFFFFFF. */

	uint8_t LEN;     /*!< Specifies the length of the frame that will be 
					transmitted. This parameter can be a value between 
					0 to 8 */

	uint8_t BUF[8]; /*!< Contains the data to be transmitted. It ranges from 0 
                        to 0xFF. */
}CAN_MSG;

uint8_t CAN_PushMessage(CanTxMsg  *pTransmitBuf );
/*	Private define ---------------------------------------------------*/


#define USE_CAN1
/* #define USE_CAN2 */

#ifdef  USE_CAN1
  #define CANx                       CAN1
  #define CAN_CLK                    RCC_APB1Periph_CAN1
  #define CAN_RX_PIN                 GPIO_Pin_11
  #define CAN_TX_PIN                 GPIO_Pin_12
  #define CAN_GPIO_PORT              GPIOA
  #define CAN_GPIO_CLK               RCC_AHB1Periph_GPIOA
  #define CAN_AF_PORT                GPIO_AF_CAN1
  #define CAN_RX_SOURCE              GPIO_PinSource11
  #define CAN_TX_SOURCE              GPIO_PinSource12          
#endif  /* USE_CAN1 */

#ifdef __cplusplus
}
#endif

#endif /*__CAN_DRIVER_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
