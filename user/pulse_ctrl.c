/**
	********************************************************************
	*	@file			pulse_ctrl.c
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
struct rt_semaphore pulse_ctrl_sem;
extern uint8_t g_l_femoral_cmd,g_r_femoral_cmd;
extern uint8_t g_l_femoral_power_cmd,g_r_femoral_power_cmd;
extern uint8_t g_l_pulse_cmd,g_r_pulse_cmd;
extern uint8_t g_pulse_level[3];
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
/**
	*	@name		GPIO_Configuration
	*	@brief		wifi module GPIO Configuration
	*	@param		None
	*	@retval		None
	*/
typedef struct{
	uint8_t power_return;
	uint8_t update_flag;
}POWER_SET;

typedef struct{
	uint8_t cmd_now;
	uint8_t cmd_before;
	uint8_t power_now;
	uint8_t power_before;
	uint8_t power_return_before;
}PULSE_SET;

enum pulse{
	LEFT_NECK=0x00,
	RIGHT_NECK,
	LEFT_BRACHIAL,
	LEFT_RADIAL,
	RIGHT_BRACHIAL,
	RIGHT_RADIAL,
	LEFT_FEMORAL,
	RIGHT_FEMORAL,
	LEFT_POPLITEAL,
	LEFT_DORSAL,
	RIGHT_POPLITEAL,
	RIGHT_DORSAL
};



POWER_SET pulse_power(PULSE_SET *pulse)
{
	POWER_SET power;
	
	power.power_return=0;
	power.update_flag=0;
	
	if(pulse->cmd_now!=pulse->cmd_before||pulse->power_now!=pulse->power_before)//开关或通电时间有变化
	{
		power.update_flag=1;
		if(pulse->cmd_now)//打开
		{
			if(pulse->power_now==0)//是零
			{
				power.power_return=0x01;//10ms
			}
			else//非零
			{
				power.power_return=pulse->power_now;
			}
		}
		else//关闭
		{
			power.power_return=0x00;//置零
		}
		pulse->power_now=power.power_return;
	}
	else
	{
		power.update_flag=0;
		power.power_return=pulse->power_before;
	}
	
	
	
	return (power);
}

void pulse_set(PULSE_SET *pulse_near, PULSE_SET *pulse_far,uint8_t can_fi,uint8_t can_si)
{
	POWER_SET power_set_near, power_set_far;
	uint8_t cmd[2]={0};
	CanTxMsg *TxMessage;
	
	power_set_near=pulse_power(pulse_near);
	power_set_far=pulse_power(pulse_far);
	if(power_set_near.update_flag||power_set_far.update_flag)
	{
		cmd[0]=power_set_near.power_return;
		cmd[1]=power_set_far.power_return;
		cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,can_fi,2,can_si,cmd);
	}
}

void pulse_cmd_update_init(PULSE_SET pulse[])
{
	//近心端力度更新
	pulse[LEFT_NECK].power_before=0;
	pulse[RIGHT_NECK].power_before=0;
	pulse[LEFT_FEMORAL].power_before=0;
	pulse[RIGHT_FEMORAL].power_before=0;
	//中间端力度更新
	pulse[LEFT_BRACHIAL].power_before=0;
	pulse[RIGHT_BRACHIAL].power_before=0;
	pulse[LEFT_POPLITEAL].power_before=0;
	pulse[RIGHT_POPLITEAL].power_before=0;
	//远心端力度更新
	pulse[LEFT_RADIAL].power_before=0;
	pulse[RIGHT_RADIAL].power_before=0;
	pulse[LEFT_DORSAL].power_before=0;
	pulse[RIGHT_DORSAL].power_before=0;

	//开关更新
	pulse[LEFT_NECK].cmd_before=0;
	pulse[RIGHT_NECK].cmd_before=0;
	pulse[LEFT_FEMORAL].cmd_before=0;
	pulse[RIGHT_FEMORAL].cmd_before=0;
	pulse[LEFT_BRACHIAL].cmd_before=0;
	pulse[RIGHT_BRACHIAL].cmd_before=0;
	pulse[LEFT_POPLITEAL].cmd_before=0;
	pulse[RIGHT_POPLITEAL].cmd_before=0;
	pulse[LEFT_RADIAL].cmd_before=0;
	pulse[RIGHT_RADIAL].cmd_before=0;
	pulse[LEFT_DORSAL].cmd_before=0;
	pulse[RIGHT_DORSAL].cmd_before=0;
}

void pulse_cmd_update_head(PULSE_SET pulse[], uint8_t power_level[], uint8_t left_cmd,uint8_t right_cmd)
{
	//近心端力度更新
	pulse[LEFT_NECK].power_now=power_level[0];
	pulse[RIGHT_NECK].power_now=power_level[0];
	pulse[LEFT_FEMORAL].power_now=power_level[0];
	pulse[RIGHT_FEMORAL].power_now=power_level[0];
	//中间端力度更新
	pulse[LEFT_BRACHIAL].power_now=power_level[1];
	pulse[RIGHT_BRACHIAL].power_now=power_level[1];
	pulse[LEFT_POPLITEAL].power_now=power_level[1];
	pulse[RIGHT_POPLITEAL].power_now=power_level[1];
	//远心端力度更新
	pulse[LEFT_RADIAL].power_now=power_level[2];
	pulse[RIGHT_RADIAL].power_now=power_level[2];
	pulse[LEFT_DORSAL].power_now=power_level[2];
	pulse[RIGHT_DORSAL].power_now=power_level[2];

	//开关更新
	pulse[LEFT_NECK].cmd_now=left_cmd&0x01;
	pulse[RIGHT_NECK].cmd_now=right_cmd&0x01;
	pulse[LEFT_FEMORAL].cmd_now=left_cmd>>3&0x01;
	pulse[RIGHT_FEMORAL].cmd_now=right_cmd>>3&0x01;
	pulse[LEFT_BRACHIAL].cmd_now=left_cmd>>1&0x01;
	pulse[RIGHT_BRACHIAL].cmd_now=right_cmd>>1&0x01;
	pulse[LEFT_POPLITEAL].cmd_now=left_cmd>>4&0x01;
	pulse[RIGHT_POPLITEAL].cmd_now=right_cmd>>4&0x01;
	pulse[LEFT_RADIAL].cmd_now=left_cmd>>2&0x01;
	pulse[RIGHT_RADIAL].cmd_now=right_cmd>>2&0x01;
	pulse[LEFT_DORSAL].cmd_now=left_cmd>>5&0x01;
	pulse[RIGHT_DORSAL].cmd_now=right_cmd>>5&0x01;
}

void pulse_cmd_update_tail(PULSE_SET pulse[])
{
	uint8_t i;
	for(i=0;i<12;i++)
	{
		pulse[i].cmd_before=pulse[i].cmd_now;
		pulse[i].power_before=pulse[i].power_now;
	}
}

ALIGN(RT_ALIGN_SIZE)
static char pulse_ctrl_thread_stack[1024];
struct rt_thread pulse_ctrl_thread;
static void pulse_ctrl_thread_entry(void *param)
{
	PULSE_SET pulse[12];
	
	pulse_cmd_update_init(pulse);
	
    while(1)
    {
    	if(rt_sem_take(&pulse_ctrl_sem, RT_WAITING_FOREVER)==RT_EOK)//received a semaphore
		{
			//update power level and pulse on/off cmd at first
			pulse_cmd_update_head(pulse,g_pulse_level,g_l_pulse_cmd,g_r_pulse_cmd);
			
			//update pulse on/off cmd
			//颈动脉
			pulse_set(&pulse[LEFT_NECK], &pulse[RIGHT_NECK],HEAD_CAN_ID,CAN_NECK_PULSE);
			
			//左臂
			pulse_set(&pulse[LEFT_BRACHIAL], &pulse[LEFT_RADIAL],LEFT_ARM_CAN_ID,CAN_LEFT_ARM_PULSE);

			//右臂
			pulse_set(&pulse[RIGHT_BRACHIAL], &pulse[RIGHT_RADIAL],RIGHT_ARM_CAN_ID,CAN_RIGHT_ARM_PULSE);
			
			//左腿
			pulse_set(&pulse[LEFT_POPLITEAL], &pulse[LEFT_DORSAL],LEFT_LEG_CAN_ID,CAN_LEFT_LEG_PULSE);
			
			//右腿
			pulse_set(&pulse[RIGHT_POPLITEAL], &pulse[RIGHT_DORSAL],RIGHT_LEG_CAN_ID,CAN_RIGHT_LEG_PULSE);
			
			//股动脉
			g_l_femoral_power_cmd=pulse[LEFT_FEMORAL].power_now;
			g_r_femoral_power_cmd=pulse[RIGHT_FEMORAL].power_now;
			g_l_femoral_cmd=pulse[LEFT_FEMORAL].cmd_now;
			g_r_femoral_cmd=pulse[RIGHT_FEMORAL].cmd_now;
			
			//update data_before
			pulse_cmd_update_tail(pulse);
		}
    }
}

/**
	*	@name		cmb_pulse_ctrl_init
	*	@brief		Init the wifi module thread
	*	@param		None
	*	@retval		None
	*/
int cmd_pulse_ctrl_sem_init(void)
{
	rt_err_t result;
	/* 初始化静态信号量，初始值是0 */
    result = rt_sem_init(&pulse_ctrl_sem, "pulse_sem", 0, RT_IPC_FLAG_FIFO);

    if (result != RT_EOK)
    {
        rt_kprintf("init pulse_sem semaphore failed.\n");
        return -1;
    }
	return 0;
}
int cmb_pulse_ctrl_init(uint8_t prio)
{					
	//init semaphore
	cmd_pulse_ctrl_sem_init();

	//pulse control thread init
    rt_thread_init(&pulse_ctrl_thread,"pulse_ctrl",
                   pulse_ctrl_thread_entry,RT_NULL,
                   &pulse_ctrl_thread_stack[0],
                   sizeof(pulse_ctrl_thread_stack),prio,10);
    rt_thread_startup(&pulse_ctrl_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
