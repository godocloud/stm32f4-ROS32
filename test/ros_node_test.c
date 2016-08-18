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



ALIGN(RT_ALIGN_SIZE)
static char thread_ros_node_test_stack[512];
struct rt_thread thread_ros_node_test;
static void rt_thread_entry_ros_node_test(void* parameter)
{

	
	while (1)
    {

        rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

int ros_node_test_init(uint8_t prio)
{
    rt_thread_init(&thread_ros_node_test,
                   "ros_node_test",
                   rt_thread_entry_ros_node_test,
                   RT_NULL,
                   &thread_ros_node_test_stack[0],
                   sizeof(thread_ros_node_test_stack),prio,5);
    rt_thread_startup(&thread_ros_node_test);

    return 0;
}

/*@}*/
