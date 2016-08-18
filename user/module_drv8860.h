#ifndef __MODULE_DRV8860_H__
#define __MODULE_DRV8860_H__

#include "stm32f4xx.h"
#include <rtthread.h>


#define   SOLE_CTRL_PORT    GPIOD
#define   SOLE_CTRL_CLK     RCC_AHB1Periph_GPIOD

#define   EN_OUT_PIN      	GPIO_Pin_8
#define   EN_SET(x)       	GPIO_WriteBit(SOLE_CTRL_PORT,EN_OUT_PIN,(BitAction)(x))

#define   FAULT_IN_PIN    	GPIO_Pin_9
#define   FAULT_READ()    	GPIO_ReadInputDataBit(SOLE_CTRL_PORT,FAULT_IN_PIN)

#define   DATA_IN_PIN    	GPIO_Pin_10
#define   DATA_READ()    	GPIO_ReadInputDataBit(SOLE_CTRL_PORT,DATA_IN_PIN)

#define   LATCH_OUT_PIN     GPIO_Pin_11
#define   LATCH_SET(x)      GPIO_WriteBit(SOLE_CTRL_PORT,LATCH_OUT_PIN,(BitAction)(x))

#define   CLK_OUT_PIN     	GPIO_Pin_12
#define   CLK_SET(x)      	GPIO_WriteBit(SOLE_CTRL_PORT,CLK_OUT_PIN,(BitAction)(x))

#define   DATA_OUT_PIN     	GPIO_Pin_13
#define   DATA_SET(x)      	GPIO_WriteBit(SOLE_CTRL_PORT,DATA_OUT_PIN,(BitAction)(x))


void DRV8860_GPIO_Init(void);
void DRV8860_DataWrite(rt_uint8_t data);
void DRV8860_FaultReset(void);
rt_uint16_t DRV8860_FaultRead(void);

#endif

