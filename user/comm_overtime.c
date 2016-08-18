/**
	********************************************************************
	*	@file			comm_overtime.c
	*	@author		Jun.Lu
	*	@version	V1.1.0
	*	@date			8-Oct-2013
	*	@brief		
	********************************************************************
	*	@attention
	* Work out with overtime of the community between board and PC.
	*-----------------------------
	*
	*	Change Logs:
	*	Date							Author						Notes
	*	2013-10-8					Jun.Lu						First Implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "comm_overtime.h"
#include "protocol.h"

#include "RS232.h"
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
uint16_t package_id=0;
ID_BUFFER comm[ID_BUFFER_SIZE];
uint8_t resend_flag=0;
extern uint8_t back_data_send(CMD_DATA_STRUCT data4back,uint16_t package_id);
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
/**
	*	@brief	using for what
	*	@note		description
	*	@param	
	*	@retval	
	*/
extern CMD_DATA_STRUCT mb_receive_to_send_old[30];

ALIGN(RT_ALIGN_SIZE)
static char comm_overtime_thread_stack[1024];
struct rt_thread comm_overtime_thread;
static void comm_overtime_thread_entry(void *param)
{
		uint8_t i;
		uint16_t buff_n;
		rt_time_t	next_delay;
    while(1)
    {
				for(buff_n=0;buff_n<ID_BUFFER_SIZE;buff_n++)
				{
						if(comm[buff_n].delay_counter<10)//200ms以内的时候
						{
								if(comm[buff_n].flag==CMD_SEND)//如果该序号的flag标志为发送，则开始计数
								{
										comm[buff_n].delay_counter++;
								}
								else if(comm[buff_n].flag==ACK_RECEIVED)
								{
										comm[buff_n].package_id=0;
										comm[buff_n].flag=ACK_CLEAR;
										comm[buff_n].delay_counter=0;
									comm[buff_n].resend_count=0;
								}
						}
						else if(comm[buff_n].delay_counter>=10)//超过200ms
						{
							comm[buff_n].delay_counter=0;
													
							if(comm[buff_n].resend_count<3)
							{
								resend_flag=1;
			
								back_data_send(mb_receive_to_send_old[comm[buff_n].structure_id],comm[buff_n].package_id);
								comm[buff_n].resend_count++;
							}
							else
							{
								comm[buff_n].package_id=0;
								comm[buff_n].flag=ACK_CLEAR;
								comm[buff_n].delay_counter=0;
								comm[buff_n].resend_count=0;
							}
																										
		
						}
						//重发三次刚才包id的对应的package
				}
				
		next_delay=2;//20ms       
        rt_thread_delay(next_delay); 
    }
}

/**
	*	@name		cmb_hw_comm_overtime_init
	*	@brief	Init the Water Pumps(including the keys )
	*	@param	None
	*	@retval	None
	*/
int cmb_hw_comm_overtime_init(void)
{
    
		
		//debug1 thread init
    rt_thread_init(&comm_overtime_thread,"comm_overtime",
                    comm_overtime_thread_entry,RT_NULL,
                    &comm_overtime_thread_stack[0],
                    sizeof(comm_overtime_thread_stack),10,5);
    rt_thread_startup(&comm_overtime_thread);
		
		return 0;   
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
