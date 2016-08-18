/**
	********************************************************************
	*	@file			RfuckS232.c
	*	@author			Jun.Lu
	*	@version		V1.0.0
	*	@date			13-May-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	The driver of rs232 using for communicate outside.
	*-----------------------------
	*
	*	Change Logs:
	*	Date					Author				Notes
	*	2013-4-25				Jun.Lu				Version V1.0.0
	*	2013-12-2 				Jun.Lu 				alpha
	*	2016-1-4				Jun.Lu				for 150902
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "RS232.h"

#include <stdio.h>
#include <stdlib.h>

#include "protocol.h"
#include "comm_overtime.h"
#include "stm32f4xx.h"

/*	Global variables -----------------------------------------------*/
struct rt_event backdate_event;
uint16_t k=0;
uint8_t cmd_update_flag=0;
uint8_t first_rs232_flag=0;
uint8_t uart_tx_buffer_old[20]={0};
uint8_t tx_length_old=0,resend_count=0;
char rs232_ack[20]={0x0A,0x06,0x05,0x00,0x00,0xFF,0xFE};
//static char rs232_nack[7]={0x08,0x0A,0x06,0x04,0xFF,0xFF,0xFE};
extern ID_BUFFER comm[ID_BUFFER_SIZE];
extern uint8_t data_from_head[4],data_from_lower[4];
extern uint8_t resend_flag;
//上行更新事件
extern struct rt_event update_event;
rt_device_t device_232, write_device_232;
struct rt_mailbox mb_for_cmd_analysis;
static char mb_for_cmd_analysis_pool[100];
struct rt_mailbox mb_for_data_sendback;
static char mb_for_data_sendback_pool[100];
struct rt_mailbox mb_for_data_sendback_special;
static char mb_for_data_sendback_special_pool[100];
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
//#define RS232_DEBUG_ON
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/

extern void cmd_update(uint8_t structure_name[],uint8_t imformation[],uint8_t imformation_length);
extern void data_update(uint8_t structure_name[],uint8_t imformation[],uint8_t imformation_length);
								
//***
/* UART接收消息结构*/ 
struct rx_msg_uart3 
{ 
  rt_device_t dev; 
  rt_size_t size; 
}; 
/* 用于接收消息的消息队列*/ 
static rt_mq_t rx_mq; 
/* 接收线程的接收缓冲区*/ 
static uint8_t uart_rx_buffer[20]={0}; 
static uint8_t uart_tx_buffer[20]={0};
//static char uart_tx_buffer_ack[10]={0};

/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/


//在线状态发送
void body_online(uint8_t structure_id,uint8_t content)
{
	rt_uint32_t tx_length;
	uint8_t uart_tx_buffer_ack[10]={0};
	uint8_t package_length=0,package[30]={0};

	//uart_tx_buffer_ack[0]=0x08;//去数据中心
	uart_tx_buffer_ack[0]=0x18;//从模型人来
	uart_tx_buffer_ack[1]=0x03;//不需要应答,0x13
	uart_tx_buffer_ack[2]=0x06;//package length
	uart_tx_buffer_ack[3]=structure_id;
	uart_tx_buffer_ack[4]=0x00;
	uart_tx_buffer_ack[5]=content;
	uart_tx_buffer_ack[6]=0xFF;
	uart_tx_buffer_ack[7]=0xFE;

	tx_length = 8;

	/* 写到写设备中 */
	if (write_device_232 != RT_NULL) 
	{
		rt_device_write(write_device_232, 0, 
						&package[0], package_length);
	}
}
/* 数据到达回调函数*/ 
rt_err_t uart3_input(rt_device_t dev, rt_size_t size) 
{ 
	struct rx_msg_uart3 msg_uart3; 
	msg_uart3.dev = dev; 
	msg_uart3.size = size; 
 
	/* 发送消息到消息队列中*/ 
	rt_mq_send(rx_mq, &msg_uart3, sizeof(struct rx_msg_uart3)); 

	return RT_EOK; 
} 


data_sum_check(uint8_t state)
{
	if(state==LENGTH)
	{
		
	}
}

uint8_t g_ros_test_flag=0;
extern uint8_t g_communication_start;
void rs232_thread_entry(void *param)
{ 
	struct rx_msg_uart3 msg_rs232; 
	uint8_t cmd_state=SYNC_FLAG;
	uint8_t rs232_init_flag=0;
	uint8_t msg_length_low=0,msg_length_high=0,topic_id_received[2]={0},data[20]={0},verify=0;
	uint8_t i=0,n=0;
	uint16_t information_length=0;
	
	rt_err_t result = RT_EOK; 

	//CMD_DATA cmd_to_analysis;
	RECEIVE_DATA *receive_data;

	rt_thread_delay(500); //delay for init
	//rt_time_t	next_delay;

 
	while (1) 
	{ 
		if(rs232_init_flag==0)
		{
			//if(g_communication_start==1)
			{
				rs232_init_flag=1;
				/* 查找系统中的串口设备 */ 
				device_232 = rt_device_find("uart3"); 
				if (device_232!= RT_NULL) 
				{ 
					/* 设置回调函数及打开设备*/ 
					rt_device_set_rx_indicate(device_232, uart3_input); 
					rt_device_open(device_232, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX); 
					
				} 
				/* 设置写设备为uart设备 */ 
				
				write_device_232 = rt_device_find("uart1"); 
				if (write_device_232!= RT_NULL) 
				{ 
					rt_device_open(write_device_232, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX); 
					
				} 

			}
		}
		else
		{
			//2013 12 29
			//在此加上接收两个wifi_socket接受相关的事件和运算，接收线程优先级要比发送的优先级高
			/* 从消息队列中读取消息*/ 
			result = rt_mq_recv(rx_mq, &msg_rs232,  
				sizeof(struct rx_msg_uart3), 50); 
			/*if (result == -RT_ETIMEOUT) 
			{ 
				// 接收超时
				rt_kprintf("timeout count:%d\n", ++count); 
			} */
	 
			/* 成功收到消息*/ 
			if (result == RT_EOK) 
			{ 
				
				rt_uint32_t rx_length,tx_length; 
				rx_length = (sizeof(uart_rx_buffer) - 1) > 
							msg_rs232.size ? msg_rs232.size:sizeof(uart_rx_buffer)-1; 
				//rt_kprintf("1st: rx_length=%d",rx_length);
//				rt_kprintf("get data\r\n");
				/* 读取消息 */ 
				rx_length =rt_device_read(msg_rs232.dev, 0, 
										&uart_rx_buffer[0], rx_length); 
				//rt_kprintf("2nd: rx_length=%d\r\n",rx_length);
				uart_rx_buffer[rx_length] = '\0'; 
				
				//--------send back received things
				rt_device_write(write_device_232, 0,\
									&uart_rx_buffer[0], rx_length);	
				//--------end of send back
				//rt_kprintf("rx_length1=%d",rx_length);			
				{				
					/*拆包分析代码开始*/
					for(i=0;i<rx_length;i++)
					//for(i=package_start;i<package_count;i++)
					{							
						switch(cmd_state)
						{
							//**************sync flag**************
							case SYNC_FLAG:
								if((uart_rx_buffer[i])==0xFF)//Man data receiving right
								{
									{
										cmd_state=PROTOCOL_VERSION;
#ifdef RS232_DEBUG_ON
										rt_kprintf("sync flag~~~\r\n");
#endif										
									}
								}
								break;
							//**************version**************		
							case PROTOCOL_VERSION:
								if(uart_rx_buffer[i]==0xFE)//command，0x11
								{
									cmd_state=MESSAGE_LENGTH_LOW;
#ifdef RS232_DEBUG_ON									
									rt_kprintf("type=version~~~\r\n");
#endif									
								}
								else
								{
									cmd_state=SYNC_FLAG;
								}
								break;
								//**************length**************
							case MESSAGE_LENGTH_LOW:
								msg_length_low=uart_rx_buffer[i];
#ifdef RS232_DEBUG_ON								
								rt_kprintf("msg_length_low=%d\r\n",msg_length_low);
#endif								
								cmd_state=MESSAGE_LENGTH_HIGH;
								break;
							case MESSAGE_LENGTH_HIGH:
								msg_length_high=uart_rx_buffer[i];
								information_length=(msg_length_high<<8)+msg_length_low;
								cmd_state=LENGTH_CHECKSUM;
#ifdef RS232_DEBUG_ON								
								rt_kprintf("information_length=%d\r\n",information_length);
#endif
								break;
							//**************LENGTH_CHECKSUM**************
							case LENGTH_CHECKSUM://收到的是命令，开始解析
								if((0xff-msg_length_low-msg_length_high)==uart_rx_buffer[i])
								{
									cmd_state=TOPIC_ID_LOW;
								}
#ifdef RS232_DEBUG_ON								
								rt_kprintf("checksum=%x,uart_rx_buffer[i]=%x\r\n",\
											(0xff-msg_length_low-msg_length_high),
											uart_rx_buffer[i]);
#endif											
								break;
							//**************topic**************
							case TOPIC_ID_LOW://收到的是数据，开始解析
								topic_id_received[0]=uart_rx_buffer[i];
								verify+=topic_id_received[0];
								cmd_state=TOPIC_ID_HIGH;
								break;
							case TOPIC_ID_HIGH:
								topic_id_received[1]=uart_rx_buffer[i];
								verify+=topic_id_received[1];
								if(information_length==0)
								{
									cmd_state=OVERALL_CHECKSUM;
								}
								else
								{
									cmd_state=DATA_GET;
								}	
#ifdef RS232_DEBUG_ON								
								rt_kprintf("topic_id_receive=%x\r\n",(topic_id_received[1]<<8)+topic_id_received[0]);
#endif								
								break;
							case DATA_GET:
								data[n++]=uart_rx_buffer[i];
								verify+=uart_rx_buffer[i];
								if(n==information_length)
								{
									cmd_state=OVERALL_CHECKSUM;
								}
								break;
							case OVERALL_CHECKSUM:
								if((0xff-verify)==uart_rx_buffer[i])
								{
									//这里将信息放到结构体，然后发送
//									receive_data=rt_malloc(sizeof(RECEIVE_DATA));
//									rt_memcpy((*receive_data).topic_id,topic_id_received,2);
//									(*receive_data).cmd_length=information_length;
//									if(information_length!=0)
//									{
//										rt_memcpy((*receive_data).cmd_data,data,information_length);
//									}
//									//然后起一个解析线程，挂起在邮箱接收
//									rt_mb_send(&mb_for_cmd_analysis, (rt_uint32_t)receive_data);
//									memset(topic_id_received,0,2);
//									memset(data,0,information_length);
//									information_length=0;
#ifdef RS232_DEBUG_ON								
									rt_kprintf("FINISH~~~~\r\n");
#endif	
									g_ros_test_flag=1;
								}
								cmd_state=SYNC_FLAG;
								
								verify=0;
								break;		
							default:
								break;
						}																		
					}

				} 
			}
		}
	} 
} 

/**
	*	@brief	using for what
	*	@note		description
	*	@param	
	*	@retval	
	*/
//uint8_t head_reflect=0;
//ALIGN(RT_ALIGN_SIZE)
//static char collectback_thread_stack[1024];
//struct rt_thread  collectback_thread;
//static void  collectback_thread_entry(void *param)
//{
//		rt_time_t	next_delay;
//		rt_uint32_t e;
//		uint8_t head_data[4]={0};
//		while(1)
//		{							
//			//head_data_analysis(data_from_head,2);
//			next_delay=10;//
//			rt_thread_delay(next_delay);
//			//lower_data_analysis(data_from_lower,2);
//			next_delay=10;//
//			rt_thread_delay(next_delay);
//		}
//}

extern uint16_t package_id;
REGEDIT regedit2[99]={0xFF};
CMD_DATA_STRUCT mb_receive_to_send_old[30];
ALIGN(RT_ALIGN_SIZE)
static char data_sendback_thread_stack[1024];
struct rt_thread  data_sendback_thread;
static void  data_sendback_thread_entry(void *param)
{
	CMD_DATA_STRUCT *mb_receive_to_send;
	uint8_t c,send_ok_flag=0;
	
	while(1)
	{				
//		if (rt_mb_recv(&mb_for_data_sendback,(rt_uint32_t *)&mb_receive_to_send, 1)== RT_EOK)
//		{
//			for(c=0;c<(*mb_receive_to_send).cmd_l;c++)
//			{
//				if((*mb_receive_to_send).cmd_data[c]!=mb_receive_to_send_old[(*mb_receive_to_send).struct_id].cmd_data[c])
//				{
//					send_ok_flag=1;
//				}
//			}
//			//if(send_ok_flag==1)
//			{
//				back_data_send(*mb_receive_to_send,package_id++);
//				mb_receive_to_send_old[(*mb_receive_to_send).struct_id]=*mb_receive_to_send;
//				send_ok_flag=0;
//				if(package_id>60000)
//					package_id=0;
//			}
//			if(mb_receive_to_send!=RT_NULL)
//			{
//				rt_free(mb_receive_to_send);
//			}
//		}
		
//		if (rt_mb_recv(&mb_for_data_sendback_special,(rt_uint32_t *)&mb_receive_to_send, 1)== RT_EOK)
//		{
//			back_data_send(*mb_receive_to_send,package_id);
//			
//			if(mb_receive_to_send!=RT_NULL)
//			{
//				rt_free(mb_receive_to_send);
//			}
//		}
	}
}

/**
	*	@name		cmb_hw_rs232_init
	*	@brief		rs232 thread init
	*	@param		None
	*	@retval		None
	*/
static struct rt_messagequeue s_your_mq;//消息队列控制块对象
static char s_msg_pool[1024];//消息池
int cmb_hw_rs232_init(void)
{
	rt_thread_t rs232_thread;
	rt_err_t result;
	//新建命令解析缓存邮箱								
	rt_mb_init(&mb_for_cmd_analysis,
	"cmd_analysis_mb", 
	&mb_for_cmd_analysis_pool[0], 
	sizeof(mb_for_cmd_analysis_pool)/4, 
	RT_IPC_FLAG_FIFO);
	//新建数据上传缓存邮箱
	rt_mb_init(&mb_for_data_sendback,
	"data_sendback_mb", 
	&mb_for_data_sendback_pool[0], 
	sizeof(mb_for_data_sendback_pool)/4, 
	RT_IPC_FLAG_FIFO);
	//新建数据上传特殊缓存邮箱，这个邮箱不判定是否重复，有了就发送
	rt_mb_init(&mb_for_data_sendback_special,
	"data_sendback_special_mb", 
	&mb_for_data_sendback_special_pool[0], 
	sizeof(mb_for_data_sendback_special_pool)/4, 
	RT_IPC_FLAG_FIFO);
	
	result=rt_event_init(&backdate_event, "backdate_event",RT_IPC_FLAG_FIFO);
    if (result!= RT_EOK)
    {
        rt_kprintf("init backdate_event failed.\n");
        return -1;
    }
		
	//初始化消息队列控制块
	rt_mq_init(&s_your_mq,"test",&s_msg_pool[0],\
							sizeof(struct rx_msg_uart3),sizeof(s_msg_pool),\
							RT_IPC_FLAG_FIFO);

	rx_mq =&s_your_mq;//将rx_mq再指向s_your_mq
	
//		/**/rt_thread_init(&data_sendback_thread,"data_sendback",
//                    data_sendback_thread_entry,RT_NULL,
//                    &data_sendback_thread_stack[0],
//                    sizeof(data_sendback_thread_stack),4,10);
//    rt_thread_startup(&data_sendback_thread);
	/* 创建devt线程*/
	rs232_thread = rt_thread_create("rs232",
									rs232_thread_entry, RT_NULL,
									1024, 6, 10);
	
	
	/* 创建成功则启动线程*/
	if (rs232_thread!= RT_NULL)
	{
			rt_thread_startup(rs232_thread);  
	}
	
		
		return 0;
}

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
