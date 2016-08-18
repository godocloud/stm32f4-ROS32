/*
 * File      : application.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      the first version
 */

/**
 * @addtogroup STM32
 */
/*@{*/

#include <stdio.h>
#include "stm32f4xx.h"
#include <rtthread.h>
#include "protocol.h"

extern void body_online(uint8_t structure_id,uint8_t content);

static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//init gpio configuration 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

extern uint8_t g_heartbeat_cmd;
extern struct rt_mailbox cmd2can_mailbox;
ALIGN(RT_ALIGN_SIZE)
static char thread_heartbeat_stack[512];
struct rt_thread thread_heartbeat;
static void rt_thread_entry_heartbeat(void* parameter)
{
//    uint8_t cmd[8]={0};
//	CanTxMsg *TxMessage;
	GPIO_Configuration();

	while (1)
    {
		
		GPIO_SetBits(GPIOE,GPIO_Pin_15);
		//GPIO_SetBits(GPIOA,GPIO_Pin_12);
        rt_thread_delay(RT_TICK_PER_SECOND*2/50);
		
		GPIO_ResetBits(GPIOE,GPIO_Pin_15);
		//GPIO_ResetBits(GPIOA,GPIO_Pin_12);
        rt_thread_delay(RT_TICK_PER_SECOND*7/50);
		
		GPIO_SetBits(GPIOE,GPIO_Pin_15);
		//GPIO_SetBits(GPIOA,GPIO_Pin_12);
        rt_thread_delay(RT_TICK_PER_SECOND*1/50);
		
		GPIO_ResetBits(GPIOE,GPIO_Pin_15);
			//body_online(SI_MODELOFPERSON, 0);
			
		//GPIO_ResetBits(GPIOA,GPIO_Pin_12);
//		g_heartbeat_cmd=1;
//		rt_kprintf("alive\r\n");
//		cmd[0]=0x88;
//		cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,0x70,1,0x99,cmd);
//		rt_kprintf("alive...\r\n");
        rt_thread_delay(RT_TICK_PER_SECOND*40/50);
    }
}

int nano_board_heartbeat_init(uint8_t prio)
{
    rt_thread_init(&thread_heartbeat,
                   "heartbeat",
                   rt_thread_entry_heartbeat,
                   RT_NULL,
                   &thread_heartbeat_stack[0],
                   sizeof(thread_heartbeat_stack),prio,5);
    rt_thread_startup(&thread_heartbeat);

    return 0;
}

/*@}*/
