/**
	********************************************************************
	*	@file			CAN.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* Setup the can module and control it.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2016-1-13			Jun.Lu						first implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
//#include "can.h"
#include <board.h>
#include "protocol.h"
#include "can_driver.h"

/*	Global variables -----------------------------------------------*/
//struct rt_mailbox can_received_mailbox;
//static char can_received_mailbox_pool[200];

extern struct rt_mailbox cmd2can_mailbox;

struct rt_messagequeue can_rx_mq;//消息队列控制块对象
char can_rx_msg_pool[1024];//消息池
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
extern void CAN_Config(void);
extern void NVIC_Config(void);
/*	Private fuctions -----------------------------------------------*/
/**
	*	@name		GPIO_Configuration
	*	@brief		wifi module GPIO Configuration
	*	@param		None
	*	@retval		None
	*/

ALIGN(RT_ALIGN_SIZE)
static char can_send_thread_stack[1024];
struct rt_thread can_send_thread;
static void can_send_thread_entry(void *param)
{
	CanTxMsg *cmd_receive2can;
	//CMD OLD_CMD[STURCTURE_MAX];
	
	CAN_Config();
	NVIC_Config();
	
    while(1)
    {
    	if (rt_mb_recv(&cmd2can_mailbox,(rt_uint32_t *)&cmd_receive2can, 1)== RT_EOK)
		{
			//CAN_PushMessage(cmd_receive2can);
			//没有空闲邮箱时就等待发送完成
			while(CAN_PushMessage(cmd_receive2can) == CAN_TxStatus_NoMailBox)
			{
				rt_thread_delay(1);
			}
			
			//收完释放掉空间
			if(cmd_receive2can!=RT_NULL)
			{
				rt_free(cmd_receive2can);
			}
			
		}
    }
}

extern void can_data_update(CanRxMsg can_receive_buff);
//can接收线程
ALIGN(RT_ALIGN_SIZE)
static char can_receive_thread_stack[1024];
struct rt_thread can_receive_thread;
static void can_receive_thread_entry(void *param)
{
	CanRxMsg can_receive_buff;
	
	while(1)
	{
		if (rt_mq_recv(&can_rx_mq, &can_receive_buff,  
				sizeof(CanRxMsg), 50)== RT_EOK)
		{
			can_data_update(can_receive_buff);
//			//收完释放掉空间
//			if(can_receive_buff!=RT_NULL)
//			{
//				rt_free(can_receive_buff);
//			}
		}
	}
}

/**
	*	@name		cmb_hw_wifi_module_init
	*	@brief		Init the wifi module thread
	*	@param		None
	*	@retval		None
	*/

int cmb_can_init(uint8_t prio)
{
//    //CAN接收缓存邮箱
//	rt_mb_init(&can_received_mailbox,
//	"can_receive", 
//	&can_received_mailbox_pool[0], 
//	sizeof(can_received_mailbox_pool)/4, 
//	RT_IPC_FLAG_FIFO);
	
	//CAN接收消息队列
	rt_mq_init(&can_rx_mq,"can_rx_mq",&can_rx_msg_pool[0],\
							sizeof(CanRxMsg),sizeof(can_rx_msg_pool),\
							RT_IPC_FLAG_FIFO);
							
	//can send thread init
    rt_thread_init(&can_send_thread,"can_send",
                   can_send_thread_entry,RT_NULL,
                   &can_send_thread_stack[0],
                   sizeof(can_send_thread_stack),prio,10);
    rt_thread_startup(&can_send_thread);
    
	//can receive thread init
    rt_thread_init(&can_receive_thread,"can_receive",
                   can_receive_thread_entry,RT_NULL,
                   &can_receive_thread_stack[0],
                   sizeof(can_receive_thread_stack),prio+1,10);
    rt_thread_startup(&can_receive_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
