/**
	********************************************************************
	*	@file			blooding_ctrl.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* Setup the can module and control it.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2016-2-16			Jun.Lu						first implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>
#include "protocol.h"

/*	Global variables -----------------------------------------------*/
/* 信号量控制块 */
struct rt_semaphore blooding_ctrl_sem;

extern struct rt_mailbox cmd2can_mailbox;
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
extern void cmd4can_push_to_mail(CanTxMsg *TxMessage, 
								rt_mailbox_t cmd4can_buff_mailbox, 
								uint8_t can_fi, uint8_t num_cmd, 
								uint8_t can_si, uint8_t cmd[]);
/*	Private fuctions -----------------------------------------------*/

extern uint8_t g_limbs_blooding_cmd;
ALIGN(RT_ALIGN_SIZE)
static char blooding_ctrl_thread_stack[1024];
struct rt_thread blooding_ctrl_thread;
static void blooding_ctrl_thread_entry(void *param)
{
	uint8_t g_limbs_blooding_cmd_old=0,limbs_blooding_check=0;
	uint8_t cmd[2]={0},valve_la=0,valve_ra=0,valve_b=0;
	CanTxMsg *TxMessage;
	
    while(1)
    {
    	if(rt_sem_take(&blooding_ctrl_sem, RT_WAITING_FOREVER)==RT_EOK)//received a semaphore
		{
			if(g_limbs_blooding_cmd!=g_limbs_blooding_cmd_old)
			{
				limbs_blooding_check=g_limbs_blooding_cmd^g_limbs_blooding_cmd_old;
				g_limbs_blooding_cmd_old=g_limbs_blooding_cmd;
			}
			//左臂
			if(limbs_blooding_check&0x05)//0000 0101,左臂出血有变化
			{
				if(g_limbs_blooding_cmd&0x05)//左臂有出血
				{
					valve_la=1;
					valve_b=1;
					cmd[0]=valve_la;//左A打开
					cmd[1]=valve_b;//B打开
					cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,LEFT_ARM_CAN_ID,2,CAN_LEFT_ARM_BLOODING,cmd);
				}
				else
				{
					valve_la=0;
					valve_b=0|valve_ra;
					cmd[0]=valve_la;//左A关闭
					cmd[1]=valve_b;//只有右侧也不出血，才关闭B
					cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,LEFT_ARM_CAN_ID,2,CAN_LEFT_ARM_BLOODING,cmd);
				}
			}
			//右臂
			if(limbs_blooding_check&0x0A)//0000 1010,右臂
			{
				if(g_limbs_blooding_cmd&0x0A)//右臂有出血
				{
					valve_ra=1;	
					cmd[0]=valve_ra;//右A打开
					cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_ARM_CAN_ID,1,CAN_RIGHT_ARM_BLOODING,cmd);
					valve_b=1;
					cmd[0]=valve_la;//左A不变
					cmd[1]=valve_b;//B打开
					cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,LEFT_ARM_CAN_ID,2,CAN_LEFT_ARM_BLOODING,cmd);
			
				}
				else
				{
					valve_ra=0;	
					cmd[0]=valve_ra;//右A关闭
					cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_ARM_CAN_ID,1,CAN_RIGHT_ARM_BLOODING,cmd);
					valve_b=0|valve_la;
					cmd[0]=valve_la;//左A不变
					cmd[1]=valve_b;//只有左侧也不出血，才关闭b
					cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,LEFT_ARM_CAN_ID,2,CAN_LEFT_ARM_BLOODING,cmd);
				}
			}
			//左腿
			if(limbs_blooding_check&0x50)//0101 0000,·?á?′ú±í×óíè×′ì?óD±??ˉ
			{
				cmd[0]=((g_limbs_blooding_cmd>>4)&0x01)|((g_limbs_blooding_cmd>>6)&0x01);//é??a
				cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,LEFT_LEG_CAN_ID,1,CAN_LEFT_LEG_BLOODING,cmd);
			}
		}
    }
}

/**
	*	@name		cmb_pulse_ctrl_init
	*	@brief		Init the wifi module thread
	*	@param		None
	*	@retval		None
	*/
int cmd_blooding_ctrl_sem_init(void)
{
	rt_err_t result;
	/* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&blooding_ctrl_sem, "blooding_sem", 0, RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init blooding_sem semaphore failed.\n");
        return -1;
    }
	return 0;
}
int cmb_blooding_ctrl_init(uint8_t prio)
{					
	//init semaphore
	cmd_blooding_ctrl_sem_init();

	//pulse control thread init
    rt_thread_init(&blooding_ctrl_thread,"blooding_ctrl",
                   blooding_ctrl_thread_entry,RT_NULL,
                   &blooding_ctrl_thread_stack[0],
                   sizeof(blooding_ctrl_thread_stack),prio,10);
    rt_thread_startup(&blooding_ctrl_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
