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
//���и����¼�
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
/* UART������Ϣ�ṹ*/ 
struct rx_msg_uart3 
{ 
  rt_device_t dev; 
  rt_size_t size; 
}; 
/* ���ڽ�����Ϣ����Ϣ����*/ 
static rt_mq_t rx_mq; 
/* �����̵߳Ľ��ջ�����*/ 
static uint8_t uart_rx_buffer[20]={0}; 
static uint8_t uart_tx_buffer[20]={0};
//static char uart_tx_buffer_ack[10]={0};

/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/


//����״̬����
void body_online(uint8_t structure_id,uint8_t content)
{
	rt_uint32_t tx_length;
	uint8_t uart_tx_buffer_ack[10]={0};
	uint8_t package_length=0,package[30]={0};

	//uart_tx_buffer_ack[0]=0x08;//ȥ��������
	uart_tx_buffer_ack[0]=0x18;//��ģ������
	uart_tx_buffer_ack[1]=0x03;//����ҪӦ��,0x13
	uart_tx_buffer_ack[2]=0x06;//package length
	uart_tx_buffer_ack[3]=structure_id;
	uart_tx_buffer_ack[4]=0x00;
	uart_tx_buffer_ack[5]=content;
	uart_tx_buffer_ack[6]=0xFF;
	uart_tx_buffer_ack[7]=0xFE;

	tx_length = 8;

	/* д��д�豸�� */
	if (write_device_232 != RT_NULL) 
	{
		rt_device_write(write_device_232, 0, 
						&package[0], package_length);
	}
}
/* ���ݵ���ص�����*/ 
rt_err_t uart3_input(rt_device_t dev, rt_size_t size) 
{ 
	struct rx_msg_uart3 msg_uart3; 
	msg_uart3.dev = dev; 
	msg_uart3.size = size; 
 
	/* ������Ϣ����Ϣ������*/ 
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
				/* ����ϵͳ�еĴ����豸 */ 
				device_232 = rt_device_find("uart3"); 
				if (device_232!= RT_NULL) 
				{ 
					/* ���ûص����������豸*/ 
					rt_device_set_rx_indicate(device_232, uart3_input); 
					rt_device_open(device_232, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_INT_RX); 
					
				} 
				/* ����д�豸Ϊuart�豸 */ 
				
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
			//�ڴ˼��Ͻ�������wifi_socket������ص��¼������㣬�����߳����ȼ�Ҫ�ȷ��͵����ȼ���
			/* ����Ϣ�����ж�ȡ��Ϣ*/ 
			result = rt_mq_recv(rx_mq, &msg_rs232,  
				sizeof(struct rx_msg_uart3), 50); 
			/*if (result == -RT_ETIMEOUT) 
			{ 
				// ���ճ�ʱ
				rt_kprintf("timeout count:%d\n", ++count); 
			} */
	 
			/* �ɹ��յ���Ϣ*/ 
			if (result == RT_EOK) 
			{ 
				
				rt_uint32_t rx_length,tx_length; 
				rx_length = (sizeof(uart_rx_buffer) - 1) > 
							msg_rs232.size ? msg_rs232.size:sizeof(uart_rx_buffer)-1; 
				//rt_kprintf("1st: rx_length=%d",rx_length);
//				rt_kprintf("get data\r\n");
				/* ��ȡ��Ϣ */ 
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
					/*����������뿪ʼ*/
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
								if(uart_rx_buffer[i]==0xFE)//command��0x11
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
							case LENGTH_CHECKSUM://�յ����������ʼ����
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
							case TOPIC_ID_LOW://�յ��������ݣ���ʼ����
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
									//���ｫ��Ϣ�ŵ��ṹ�壬Ȼ����
//									receive_data=rt_malloc(sizeof(RECEIVE_DATA));
//									rt_memcpy((*receive_data).topic_id,topic_id_received,2);
//									(*receive_data).cmd_length=information_length;
//									if(information_length!=0)
//									{
//										rt_memcpy((*receive_data).cmd_data,data,information_length);
//									}
//									//Ȼ����һ�������̣߳��������������
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
static struct rt_messagequeue s_your_mq;//��Ϣ���п��ƿ����
static char s_msg_pool[1024];//��Ϣ��
int cmb_hw_rs232_init(void)
{
	rt_thread_t rs232_thread;
	rt_err_t result;
	//�½����������������								
	rt_mb_init(&mb_for_cmd_analysis,
	"cmd_analysis_mb", 
	&mb_for_cmd_analysis_pool[0], 
	sizeof(mb_for_cmd_analysis_pool)/4, 
	RT_IPC_FLAG_FIFO);
	//�½������ϴ���������
	rt_mb_init(&mb_for_data_sendback,
	"data_sendback_mb", 
	&mb_for_data_sendback_pool[0], 
	sizeof(mb_for_data_sendback_pool)/4, 
	RT_IPC_FLAG_FIFO);
	//�½������ϴ����⻺�����䣬������䲻�ж��Ƿ��ظ������˾ͷ���
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
		
	//��ʼ����Ϣ���п��ƿ�
	rt_mq_init(&s_your_mq,"test",&s_msg_pool[0],\
							sizeof(struct rx_msg_uart3),sizeof(s_msg_pool),\
							RT_IPC_FLAG_FIFO);

	rx_mq =&s_your_mq;//��rx_mq��ָ��s_your_mq
	
//		/**/rt_thread_init(&data_sendback_thread,"data_sendback",
//                    data_sendback_thread_entry,RT_NULL,
//                    &data_sendback_thread_stack[0],
//                    sizeof(data_sendback_thread_stack),4,10);
//    rt_thread_startup(&data_sendback_thread);
	/* ����devt�߳�*/
	rs232_thread = rt_thread_create("rs232",
									rs232_thread_entry, RT_NULL,
									1024, 6, 10);
	
	
	/* �����ɹ��������߳�*/
	if (rs232_thread!= RT_NULL)
	{
			rt_thread_startup(rs232_thread);  
	}
	
		
		return 0;
}

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
