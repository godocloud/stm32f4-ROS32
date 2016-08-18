/**
	********************************************************************
	*	@file			sound_ctrl.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* Manage the heartbeat sound and breath sound.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2016-2-22			Jun.Lu						first implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>
#include "protocol.h"

/*	Global variables -----------------------------------------------*/
/* 信号量控制块 */
struct rt_semaphore heart_sound_ctrl_sem;
struct rt_semaphore breath_sound_ctrl_sem;

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

extern uint8_t g_heart_sound_set[2],g_l_lung_sound_set[2],g_r_lung_sound_set[2];
extern uint8_t g_breath_in_out_flag;
ALIGN(RT_ALIGN_SIZE)
static char sound_ctrl_thread_stack[1024];
struct rt_thread sound_ctrl_thread;
static void sound_ctrl_thread_entry(void *param)
{
	uint8_t g_limbs_blooding_cmd_old=0,limbs_blooding_check=0;
	uint8_t cmd[2]={0},valve_la=0,valve_ra=0,valve_b=0;
	CanTxMsg *TxMessage;
	
    while(1)
    {
    	if(rt_sem_take(&heart_sound_ctrl_sem, 5)==RT_EOK)//received a heart beat semaphore
		{
			cmd[0]=g_heart_sound_set[0];
			cmd[1]=0xFF;//g_heart_sound_set[1];
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_HEART_SOUND,cmd);
		}
		if(rt_sem_take(&breath_sound_ctrl_sem, 5)==RT_EOK)//received a heart beat semaphore
		{
			if(g_breath_in_out_flag==1)//xi
			{
				cmd[0]=g_l_lung_sound_set[0];
				cmd[1]=g_l_lung_sound_set[1];
				cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_L_BREATH_SOUND,cmd);
				cmd[0]=g_r_lung_sound_set[0];
				cmd[1]=g_r_lung_sound_set[1];
				cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_R_BREATH_SOUND,cmd);
			}
			else if(g_breath_in_out_flag==2)
			{
				cmd[0]=g_l_lung_sound_set[0]+1;//吸音的后面序号文件是呼音
				cmd[1]=g_l_lung_sound_set[1];
				cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_L_BREATH_SOUND,cmd);
				cmd[0]=g_r_lung_sound_set[0]+1;
				cmd[1]=g_r_lung_sound_set[1];
				cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_R_BREATH_SOUND,cmd);
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
int cmd_heart_sound_ctrl_sem_init(void)
{
	rt_err_t result;
	/* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&heart_sound_ctrl_sem, "heart_sound_sem", 0, RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init heart_sound_sem semaphore failed.\n");
        return -1;
    }
	return 0;
}
int cmd_breath_sound_ctrl_sem_init(void)
{
	rt_err_t result;
	/* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&breath_sound_ctrl_sem, "breath_sound_sem", 0, RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init breath_sound_sem semaphore failed.\n");
        return -1;
    }
	return 0;
}
int cmb_sound_ctrl_init(uint8_t prio)
{					
	//init semaphore
	cmd_heart_sound_ctrl_sem_init();
	cmd_breath_sound_ctrl_sem_init();
	
	//pulse control thread init
    rt_thread_init(&sound_ctrl_thread,"sound_ctrl",
                   sound_ctrl_thread_entry,RT_NULL,
                   &sound_ctrl_thread_stack[0],
                   sizeof(sound_ctrl_thread_stack),prio,10);
    rt_thread_startup(&sound_ctrl_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
