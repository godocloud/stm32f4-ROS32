/**
	********************************************************************
	*	@file			i2c_bit.h
	*	@author		Jun.Lu
	*	@version	V1.0.0
	*	@date			20-March-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	This file is the driver of I2C multiplexer PCA9547 and 5 or more 
	* distance IR sensors. Since the STM32 FWLib of I2C is not so smart,
	* we use the gpio simulation way to get the data of BH1772.
	*-----------------------------
	* Pins definition:
	*	I2C:
	*	PIN							FUNCTION							NOTES
	*	-----INPUT PINS:
	*	PB6							I2C1_SCL							I2C1 SCL
	*	PB7							I2C1_SDA							I2C1 SDA
	*-------------------------------------------------------------------
	*	Change Logs:
	*	Date							Author						Notes
	*	2013-3-20					Jun.Lu						First Implement
	*
	********************************************************************
	*/
/*	Includes -------------------------------------------------------*/
#include "stm32f4xx.h"

#ifndef _I2C_BIT_H_
#define _I2C_BIT_H_


#ifdef __cplusplus
 extern "C" {
#endif


#define BH1772_ENABLE 1

/*	Private define -------------------------------------------------*/
/*****PCA9547 define*****/
#define PCA9547_ADDR_WR							0xE0
#define PCA9547_ADDR_RD							0xE1

/*****BH1772 define*****/
#define	BH1772_ADDR_WR							0x70
#define	BH1772_ADDR_RD							0x71

#define BH1772_PS_MODE							0x41
#define BH1772_OI_LED								0x42
#define	BH1772_PS_MEAS_RATE					0x45
#define	PS_DATA											0x4F

#define PS_MODE_STANDBY							0x03
#define BH1772_RETRY_COUNT 3 				//repeat times

#define SENSE_LEVEL									200//ps_data channge value

#define I2C_SCL_0 GPIO_ResetBits(GPIOB,GPIO_Pin_6)
#define I2C_SCL_1 GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define I2C_SDA_0 GPIO_ResetBits(GPIOB,GPIO_Pin_7)
#define I2C_SDA_1 GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define I2C_SDA_STATE GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)//GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7)
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
enum ENUM_BH1772_REPLY
{
		BH1772_NACK=0,
		BH1772_ACK=1
};

enum ENUM_BH1772_BUS_STATE
{
		BH1772_READY=0,
		BH1772_BUS_BUSY=1,
		BH1772_BUS_ERROR=2
};
typedef enum
{
		CH0=0x08,
		CH1,
		CH2,
		CH3,
		CH4,
		CH5//this channel is not used
}PCA9547_CHANNEL;

typedef enum
{
		I_5MA=0x00,
		I_10MA,
		I_20MA,//default the IR-LED current set to 20ma
		I_50MA,
		I_100MA
		
}LED_CURRENT;

typedef enum
{
		DELAY_10MS=0x00,
		DELAY_20MS,
		DELAY_30MS,
		DELAY_50MS,//default the PS meas rate set to 50ms
		DELAY_70MS,
		DELAY_100MS,
		DELAY_200MS
		
}PS_MEAS_RATE;
/*	Private function prototypes ------------------------------------*/
extern void Delay_mS(uint32_t n);
#define DELAY Delay_mS(40)
#define RETRY_DELAY Delay_mS(50)

void BH1772_Initialize(void);
uint8_t BH1772_START(void);
void BH1772_STOP(void);
uint8_t  BH1772_SendByte(uint8_t Data);
uint16_t BH1772_ReceiveByte(void);
void BH1772_SendACK(void);
void BH1772_SendNACK(void);


#ifdef __cplusplus
}
#endif

#endif /*_I2C_BIT_H_*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
