/**
	********************************************************************
	*	@file			nrf24l01.h
	*	@author			Jun.Lu
	*	@version		V1.0.0
	*	@date			28-Oct-2013
	*	@brief			This file is the header of nrf24l01.c. 
	********************************************************************
	*	@attention
	*-------------------------------------------------------------------
	* Pins definition:
	*	NRF24L01 Pins:
	*	PIN					FUNCTION					NOTES
	*	PC4				 	GPIO						SPI1_CE
	*	PA4					SPI1_NSS					SPI1_CSN
	*	PA5					SPI1_SCK					SPI1_SCK
	*	PA7					SPI1_MOSI					SPI1_MOSI
	*	PA6					SPI1_MISO					SPI1_MISO
	*	PC5					GPIO						SPI1_IRQ
	*-------------------------------------------------------------------
	*	Change Logs:
	*	Date					Author					Notes
	*	2013-10-28				Jun.Lu				Version alpha	
	* 
	********************************************************************
	*/

/* Define to prevent recursive inclusion ---------------------------*/
#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#ifdef __cplusplus
 extern "C" {
#endif
	 
/*	Includes -------------------------------------------------------*/
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
typedef enum
{
	NORMAL=0,
	PAIRING
}NRF_STATUS;
/*	Private define -------------------------------------------------*/
#define TEMPERATURE_TIME_IN_SECOND	5//180√Î=3∑÷÷”

#define TX_ADR_WIDTH				5
#define TX_PLOAD_WIDTH 				32
#define RX_ADR_WIDTH 				5
#define RX_PLOAD_WIDTH 				32

#define CHANNEL 					0
//SPI commands for NRF24L01
#define NRF_READ_REG 				0x00//read comm from register
#define NRF_WRITE_REG 				0x20//write comm to register
#define RD_RX_PLOAD 				0x61//RX payload register address
#define WR_TX_PLOAD 				0xA0//TX payload register address
#define FLUSH_TX 					0xE1//flush TX register command
#define FLUSH_RX 					0xE2//flush RX register command
#define REUSE_TX_PL 				0xE3//reuse TX payload register command
#define NOP 						0xFF//No operation

//SPI registers or addresses for NRF24L01
#define CONFIG 						0x00//Config register address
#define EN_AA 						0x01//Enable Auto Ack register address
#define EN_RXADDR 					0x02//Enabled RX addresses register address
#define SETUP_AW 					0x03//Setup address width register address
#define SETUP_RETR 					0x04//Setup Auto retrans register address
#define RF_CH 						0x05//RF channel register address
#define RF_SETUP 					0x06//RF setup register address
#define STATUS 						0x07//Status register address
#define OBSERVE_TX 					0x08//observe TX register address
#define CD 							0x09//Carrier Detect register address
#define RX_ADDR_P0 					0x0A//RX address pipe0 register address
#define RX_ADDR_P1 					0x0B//RX address pipe1 register address
#define RX_ADDR_P2 					0x0C//RX address pipe2 register address
#define RX_ADDR_P3 					0x0D//RX address pipe3 register address
#define RX_ADDR_P4 					0x0E//RX address pipe4 register address
#define RX_ADDR_P5 					0x0F//RX address pipe5 register address
#define TX_ADDR 					0x10//RX address register address
#define RX_PW_P0 					0x11//RX payload width,pipe0 register address
#define RX_PW_P1 					0x12//RX payload width,pipe1 register address
#define RX_PW_P2 					0x13//RX payload width,pipe2 register address
#define RX_PW_P3 					0x14//RX payload width,pipe3 register address
#define RX_PW_P4 					0x15//RX payload width,pipe4 register address
#define RX_PW_P5 					0x16//RX payload width,pipe5 register address
#define FIFO_STATUS 				0x17//FIFO Status register address

#define MAX_RT 						0x10//max resent times flag
#define TX_DS 						0x20//Trans done flag
#define RX_DR 						0x40//Receive done flag

	/*	PC4				 	GPIO						SPI1_CE
	*	PA4					SPI1_NSS					SPI1_CSN
	*	PA5					SPI1_SCK					SPI1_SCK
	*	PA7					SPI1_MOSI					SPI1_MOSI
	*	PA6					SPI1_MISO					SPI1_MISO
	*	PC5					GPIO						SPI1_IRQ*/
#define SPI 						SPI1
#define GPIO_Pin_CE 				GPIO_Pin_4//PC4
#define GPIO_CE 					GPIOC

#define GPIO_Pin_CS 				GPIO_Pin_4//PA4
#define GPIO_CS 					GPIOA

#define GPIO_Pin_IRQ 				GPIO_Pin_5
#define GPIO_IRQ 					GPIOC

#define RCC_AHB1Periph_GPIO_CE_IRQ 	RCC_AHB1Periph_GPIOC

#define GPIO_SPI 					GPIOA
#define GPIO_Pin_SPI_SCK 			GPIO_Pin_5
#define GPIO_Pin_SPI_MISO 			GPIO_Pin_6
#define GPIO_Pin_SPI_MOSI 			GPIO_Pin_7
#define RCC_APBPeriph_SPI 			RCC_APB2Periph_SPI1
#define GPIO_Pin_SPI_CS_SOURCE 		GPIO_PinSource4
#define GPIO_Pin_SPI_SCK_SOURCE 	GPIO_PinSource5
#define GPIO_Pin_SPI_MISO_SOURCE 	GPIO_PinSource6
#define GPIO_Pin_SPI_MOSI_SOURCE 	GPIO_PinSource7
#define RCC_AHB1Periph_GPIO_SPI 	RCC_AHB1Periph_GPIOA




#define NRF_CSN_HIGH() 				GPIO_SetBits(GPIOA,GPIO_Pin_4)
#define NRF_CSN_LOW() 				GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define NRF_CE_HIGH() 				GPIO_SetBits(GPIOC,GPIO_Pin_4)
#define NRF_CE_LOW() 				GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define NRF_Read_IRQ() 				GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)


static uint8_t SPI_NRF_RW(uint8_t dat);
static uint8_t SPI_NRF_ReadReg(uint8_t reg);

static uint8_t SPI_NRF_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes);
static uint8_t SPI_NRF_WriteBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes);

static void NRF_TX_Mode(uint8_t *COMM_ADDRESS);
static void NRF_RX_Mode(void);
static uint8_t NRF_Rx_Dat(uint8_t *rxbuf);
static uint8_t NRF_Tx_Dat(uint8_t *txbuf);
static uint8_t NRF_Check(void);



/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif /*__NRF24L01_H__*/

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/

