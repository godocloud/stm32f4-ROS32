/**
********************************************************************
*	@file		core_test.c
*	@author		Jun.Lu
*	@version	V1.0.0
*	@date		5-March-2015
*	@brief		
********************************************************************
*	@attention
*	This file is the low level driver of gpio init.
*-----------------------------
*
*	Change Logs:
*	Date					Author					Notes
*	2015-3-4				Jun.Lu					Version V1.0.0
*
********************************************************************
*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "stm32f4_GPIO_Drv.h"
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private functions -----------------------------------------------*/

static void GPIO_Configuration(void)
{
	uint8_t i;
	for(i=0;i<13;i++)
	{
		GPIO_Pin_Setup(A,i,OUT,PP,NO);
	}
	GPIO_Pin_Setup(B,ALL,OUT,PP,NO);
	GPIO_Pin_Setup(C,ALL,OUT,PP,NO);
	GPIO_Pin_Setup(D,ALL,OUT,PP,NO);
	GPIO_Pin_Setup(E,ALL,OUT,PP,NO);
}


ALIGN(RT_ALIGN_SIZE)
static char thread_core_test_stack[512];
struct rt_thread thread_core_test;
static void rt_thread_entry_core_test(void* parameter)
{
    uint8_t i;
	GPIO_Configuration();

	while (1)
    {
		
		for(i=0;i<13;i++)
		{
			GPIO_Pin_Write(A,i,1);
		}
		GPIO_Pin_Write(B,ALL,1);
		GPIO_Pin_Write(C,ALL,1);
		GPIO_Pin_Write(D,ALL,1);
		GPIO_Pin_Write(E,ALL,1);
		rt_thread_delay(RT_TICK_PER_SECOND);
		for(i=0;i<13;i++)
		{
			GPIO_Pin_Write(A,i,0);
		}
		GPIO_Pin_Write(B,ALL,0);
		GPIO_Pin_Write(C,ALL,0);
		GPIO_Pin_Write(D,ALL,0);
		GPIO_Pin_Write(E,ALL,0);
		rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

int core_test_init(void)
{
    rt_thread_init(&thread_core_test,
                   "core_test",
                   rt_thread_entry_core_test,
                   RT_NULL,
                   &thread_core_test_stack[0],
                   sizeof(thread_core_test_stack),24,5);
    rt_thread_startup(&thread_core_test);

    return 0;
}






/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
