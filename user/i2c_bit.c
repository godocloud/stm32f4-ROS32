/**
	********************************************************************
	*	@file			i2c_bit.c
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
	*
	*	Change Logs:
	*	Date							Author						Notes
	*	2013-3-20					Jun.Lu						First Implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "i2c_bit.h"
#include "time.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
#define GPIO_PORT_I2C_SCL   		GPIOB
#define RCC_I2C_SCL        	 		RCC_AHB1Periph_GPIOB
#define PIN_I2C_SCL		    		GPIO_Pin_6

#define GPIO_PORT_I2C_SDA   		GPIOB
#define RCC_I2C_SDA         		RCC_AHB1Periph_GPIOB
#define PIN_I2C_SDA		    		GPIO_Pin_7
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/

extern struct rt_semaphore static_sem; 
/**
	*	@brief	BH1772_Delay
	*	@note		us delay func
	*	@param	just 5us for delay
	*	@retval	None
	*/
void I2C_Delay(void)
{
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	rt_sem_take(&static_sem, RT_WAITING_FOREVER);
}

/**
	*	@brief		I2C_Initialize
	*	@note		Init of I2C
	*	@param		None
	*	@retval		None
	*/
void IIC_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_I2C_SCL | RCC_I2C_SDA, ENABLE);

		/* config SCL PIN */
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

		GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL;
		GPIO_SetBits(GPIO_PORT_I2C_SCL, PIN_I2C_SCL);
		GPIO_Init(GPIO_PORT_I2C_SCL, &GPIO_InitStructure);

		/* config SDA PIN */
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

		GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
		GPIO_SetBits(GPIO_PORT_I2C_SDA, PIN_I2C_SDA);
		GPIO_Init(GPIO_PORT_I2C_SDA, &GPIO_InitStructure);
  
		//I2C_SDA_1;
		//I2C_SCL_1;  
 
		rt_kprintf("Software BH1772 Initializing...\n"); 
}

/**
	*	@brief		I2C_Start
	*	@note		START of I2C Bus Line
	*	@param		None
	*	@retval		None
	*/
void IIC_Start(void)
{ 
	I2C_SDA_1; 		 
	I2C_SCL_1; 
	I2C_Delay();

	I2C_SDA_0;
	I2C_Delay();

	I2C_SCL_0;  
	I2C_Delay();
}

/**
	*	@brief		IIC_Stop
	*	@note		STOP of I2C Bus Line
	*	@param		None
	*	@retval		None
	*/
void IIC_Stop(void)
{
	I2C_SCL_0;	
	I2C_SDA_0; 
	I2C_Delay();
	 
	I2C_SCL_1; 
	I2C_SDA_1;
	I2C_Delay();
}

/**
	*	@brief		IIC_Wait_Ack
	*	@note		Send Acknowledge to I2C Bus Line
	*	@param		None
	*	@retval		1=fail;0=ok
	*/
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
  
	I2C_SDA_1;
	I2C_Delay();
 	
	I2C_SCL_1;
	I2C_Delay();

	while(I2C_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	I2C_SCL_0;//时钟输出0 	   
	return 0;  
} 

/**
	*	@brief		I2C_SendACK
	*	@note		Send Acknowledge to I2C Bus Line
	*	@param		None
	*	@retval		None
	*/
void I2C_SendACK(void)
{
	I2C_SCL_0;	
	I2C_SDA_0;
	I2C_Delay();
	I2C_SCL_1;
	I2C_Delay();
	I2C_SCL_0;  
}

/**
	*	@brief		I2C_SendNACK
	*	@note		Send NoAcknowledge to I2C Bus Line
	*	@param		None
	*	@retval		None
	*/
void IIC_SendNACK(void)
{
	I2C_SCL_0; 
	I2C_SDA_1;
	I2C_Delay();
	I2C_SCL_1;
	I2C_Delay();
	I2C_SCL_0;    
}

/**
	*	@brief		IIC_Send_Byte
	*	@note		Send a Byte data to I2C Bus Line
	*	@param		Data
	*	@retval		None
	*/
uint8_t IIC_Send_Byte(uint8_t Data)
{
		uint8_t i;
		I2C_SCL_0;
		for(i=0;i<8;i++)
		{  
				//*****data setup*****
				if(Data&0x80)
				{
						I2C_SDA_1;
				}
				else
				{
						I2C_SDA_0;
				} 
				Data<<=1;
				I2C_Delay();
				//*****delay for data setup*****

				//*****make a rising edge
				I2C_SCL_1;
				I2C_Delay();
				I2C_SCL_0;
				I2C_Delay(); 
		}   
}


/**
	*	@brief		IIC_Read_Byte
	*	@note		Receive a Byte data from I2C Bus Line
	*	@param		None
	*	@retval		Data
	*/
uint16_t IIC_Read_Byte(void)
{
		uint8_t i,Dat;
		uint16_t temp;

		I2C_SDA_1;
		I2C_SCL_0; 
		Dat=0;
		for(i=0;i<8;i++)
		{
				I2C_SCL_1;//generate a rising edge to make sure slave ready
				I2C_Delay();
				Dat<<=1;
				if(I2C_SDA_STATE) //read the state of sda pin of I2C
				{
						Dat|=0x01; 
				}   
				I2C_SCL_0;//get ready for next receiving 
				I2C_Delay();       
		}
		//rt_kprintf("@@@@@@@@@@BH1772_Dat:%d\r\n",temp);
		return Dat;
}
