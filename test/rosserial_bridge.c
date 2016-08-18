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
#include "rosserial_bridge.h"

uint8_t rosserial_sendbuff[20]={0xff ,0xfe, 0x00, 0x00, 0xff, 0x00,0x00,0xff};
extern uint8_t g_ros_test_flag;
union {
	int32_t real;
	uint32_t base;
}u_buffer_size;
int serialize(uint8_t *outbuffer,TopicInfo *topicinfo)
{
	int offset = 0;
	uint32_t length_md5sum = strlen(topicinfo->md5sum);
	uint32_t length_topic_name = strlen(topicinfo->topic_name);
	uint32_t length_message_type = strlen(topicinfo->message_type);
	
	*(outbuffer + offset + 0) = (topicinfo->topic_id >> (8 * 0)) & 0xFF;
	*(outbuffer + offset + 1) = (topicinfo->topic_id >> (8 * 1)) & 0xFF;
	offset += sizeof(topicinfo->topic_id);
	
	memcpy(outbuffer + offset, &length_topic_name, sizeof(uint32_t));
	offset += 4;
	memcpy(outbuffer + offset, topicinfo->topic_name, length_topic_name);
	offset += length_topic_name;
	
	memcpy(outbuffer + offset, &length_message_type, sizeof(uint32_t));
	offset += 4;
	memcpy(outbuffer + offset, topicinfo->message_type, length_message_type);
	offset += length_message_type;
	
	memcpy(outbuffer + offset, &length_md5sum, sizeof(uint32_t));
	offset += 4;
	memcpy(outbuffer + offset, topicinfo->md5sum, length_md5sum);
	offset += length_md5sum;

	u_buffer_size.real = topicinfo->buffer_size;
	*(outbuffer + offset + 0) = (u_buffer_size.base >> (8 * 0)) & 0xFF;
	*(outbuffer + offset + 1) = (u_buffer_size.base >> (8 * 1)) & 0xFF;
	*(outbuffer + offset + 2) = (u_buffer_size.base >> (8 * 2)) & 0xFF;
	*(outbuffer + offset + 3) = (u_buffer_size.base >> (8 * 3)) & 0xFF;
	offset += sizeof(topicinfo->buffer_size);
	return offset;
}



ALIGN(RT_ALIGN_SIZE)
static char thread_rosserial_bridge_stack[512];
struct rt_thread thread_rosserial_bridge;
static void rt_thread_entry_rosserial_bridge(void* parameter)
{
	rt_device_t rosserial_bridge_device;
	uint8_t sendout_buff[100],sendout_length;
	TopicInfo temp={150,"chatter", "std_msgs/String", "0ad51f88fc44892f8c10684077646005", 200};
	uint8_t buff_size=0;
	rt_thread_delay(500);
	rosserial_bridge_device = rt_device_find("uart3"); 
	if (rosserial_bridge_device!= RT_NULL) 
	{ 
		rt_device_open(rosserial_bridge_device, RT_DEVICE_OFLAG_RDWR); 
		
	} 
	buff_size=sizeof(rosserial_sendbuff);
	
		
	sendout_length=serialize(&sendout_buff[0],&temp);
	
	while (1)
    {
//		if(g_ros_test_flag==1)
		{
			if (rosserial_bridge_device != RT_NULL) 
			{
				rt_device_write(rosserial_bridge_device, 0, 
								sendout_buff, sendout_length);
			}
			g_ros_test_flag=0;
		}
        rt_thread_delay(RT_TICK_PER_SECOND);
    }
}

int rosserial_bridge_init(uint8_t prio)
{
    rt_thread_init(&thread_rosserial_bridge,
                   "rosserial_bridge",
                   rt_thread_entry_rosserial_bridge,
                   RT_NULL,
                   &thread_rosserial_bridge_stack[0],
                   sizeof(thread_rosserial_bridge_stack),prio,5);
    rt_thread_startup(&thread_rosserial_bridge);

    return 0;
}

/*@}*/
