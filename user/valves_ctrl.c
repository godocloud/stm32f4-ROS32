/**
	********************************************************************
	*	@file			valves_ctrl.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* control valves and send events to valves hw thread.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2016-2-17			Jun.Lu						first implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>
#include "protocol.h"

/*	Global variables -----------------------------------------------*/
struct rt_event chest_valves_event;
extern struct rt_semaphore breath_sound_ctrl_sem;

extern uint8_t g_breathrate_cmd,g_pneu_cmd,g_hemo_cmd,g_tongu_cmd;
extern uint8_t g_pneu_puncture_state,g_hemo_puncture_state,g_pneu_press;
extern uint16_t g_l_tv,g_r_tv;
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
#define VALVES_TICK				20//200ms
#define PNEU_PRESS_MAX			60//60kpa
#define HEMO_TIME_MAX			20//20s
#define TONGU_TIME_MAX			2//2s
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/

/*	Private fuctions -----------------------------------------------*/
uint8_t g_breath_in_out_flag=0;
//自主呼吸
typedef struct
{
	uint16_t set_counter;
	uint16_t reset_counter;
	uint8_t set_flag;
	uint8_t reset_flag;
	uint16_t Uptime;
	uint16_t Downtime;
}BREATH_CTRL;

void breath_init(BREATH_CTRL *breath)
{
	breath->set_counter=0;
	breath->reset_counter=0;
	breath->set_flag=0;
	breath->reset_flag=0;
	breath->Uptime=0;
	breath->Downtime=0;
}

void breathe(rt_event_t event,BREATH_CTRL *breath,uint8_t tick, uint8_t breath_cmd)
{
	uint32_t next_delay=0;
	
	if(breath_cmd)
	{
		next_delay=100*(60/breath_cmd);
		breath->Uptime=((next_delay/4)/tick);
		breath->Downtime=(((next_delay*3)/4)/tick);

		breath->set_counter++;
		if(breath->set_flag==0)
		{
			breath->set_flag=1;
			rt_event_send(event,LEFT_BREATH_SET | RIGHT_BREATH_SET);
			g_breath_in_out_flag=1;
			rt_sem_release(&breath_sound_ctrl_sem);//release a semaphore after update
			//还需要发送给胸板“吸”
			rt_kprintf("xixixi\r\n");
			breath->reset_flag=0;
			breath->reset_counter=0;
		}
		if(breath->set_counter > breath->Uptime)//定时到
		{
			
			breath->reset_counter++;
			breath->set_counter=breath->Uptime;
			if(breath->reset_flag==0)
			{
				breath->reset_flag=1;
				rt_event_send(event,LEFT_BREATH_RESET | RIGHT_BREATH_RESET);
				g_breath_in_out_flag=2;
				rt_sem_release(&breath_sound_ctrl_sem);//release a semaphore after update
				//还需要发送给胸板“呼”
				rt_kprintf("huhuhu\r\n");
			}
			if(breath->reset_counter > breath->Downtime)
			{
				breath->set_counter=0;
				breath->set_flag=0;
			}
		}
	}
	else
	{
		if(breath->set_flag==1)//had set breath yet
		{
			rt_event_send(event,LEFT_BREATH_RESET | RIGHT_BREATH_RESET);
			breath->set_flag=0;
			breath->reset_flag=1;
		}
	}
}
//气胸
typedef struct{
	uint8_t press;
	uint8_t set_flag;
	uint8_t reset_flag;
}PNEU_CTRL;
	
void pneu_init(PNEU_CTRL *pneu)
{
	pneu->press=0;
	pneu->set_flag=0;
	pneu->reset_flag=0;
}

void pneu_set(rt_event_t event,PNEU_CTRL *pneu, uint8_t pneu_cmd)
{
	if(pneu_cmd&0x80)//有气胸
	{
		pneu->press=(pneu_cmd&0x7F)/2;//打气压力设定值
		if(pneu->press>PNEU_PRESS_MAX)//打气最大设定上限
		{
			pneu->press=PNEU_PRESS_MAX;
		}
		if(pneu->press>(g_pneu_press+5))//设定值与当前压力比较, 当前压力值增加5Kpa，为了掩盖气密性
		{
			if(g_pneu_puncture_state)//导电布有穿刺
			{
				if(pneu->reset_flag==0)
				{
					pneu->set_flag=0;
					pneu->reset_flag=1;
					rt_event_send(event,PNEU_RESET);
				}
			}
			else{
				if(pneu->set_flag==0)
				{
					pneu->set_flag=1;
					pneu->reset_flag=0;
					rt_event_send(event,PNEU_SET);
				}
			}
		}
		else{
			if(pneu->reset_flag==0)
			{
				pneu->set_flag=0;
				pneu->reset_flag=1;
				rt_event_send(event,PNEU_RESET);
			}
		}
	}
	else{
		if(pneu->reset_flag==0)
		{
			pneu->set_flag=0;
			pneu->reset_flag=1;
			rt_event_send(event,PNEU_RESET);
		}
	}
}

//血胸
typedef struct
{
	uint16_t set_counter;
	uint8_t set_flag;
	uint8_t reset_flag;
}HEMO_CTRL;

void hemo_init(HEMO_CTRL *hemo)
{
	hemo->set_counter=0;
	hemo->set_flag=0;
	hemo->reset_flag=0;
}

void hemo_set(rt_event_t event,HEMO_CTRL *hemo, uint8_t hemo_cmd)//没有已补液就不再补的逻辑
{
	uint8_t hemo_inject_time=0;
	
	if(hemo_cmd&0x80)//有血胸
	{
		hemo_inject_time=(hemo_cmd&0x7F)/2;//单位：秒
		if(hemo_inject_time>HEMO_TIME_MAX)
		{
			hemo_inject_time=HEMO_TIME_MAX;
		}
		if((hemo->set_counter)<(hemo_inject_time*5))
		{
			hemo->set_counter++;
			if(hemo->set_flag==0)
			{
				hemo->set_flag=1;
				hemo->reset_flag=0;
				rt_event_send(event,HEMO_SET);
			}
		}
		else{
			if(hemo->reset_flag==0)
			{
				hemo->set_flag=0;
				hemo->reset_flag=1;
				rt_event_send(event,HEMO_RESET);
			}
		}
		
	}
	else
	{
		hemo->set_counter=0;
		if(hemo->reset_flag==0)
		{
			hemo->set_flag=0;
			hemo->reset_flag=1;
			rt_event_send(event,HEMO_RESET);
		}
	}
}

//舌水肿
typedef struct{
	uint8_t set_counter;
	uint8_t set_flag;
	uint8_t reset_flag;
}TONGU_CTRL;

void tongu_init(TONGU_CTRL *tongu)
{
	tongu->set_counter=0;
	tongu->set_flag=0;
	tongu->reset_flag=0;
}

void tongu_set(rt_event_t event,TONGU_CTRL *tongu, uint8_t tongu_cmd)
{
	if(tongu_cmd)//有舌水肿
	{
		if(tongu->set_counter<(TONGU_TIME_MAX*5))
		{
			tongu->set_counter++;
			if(tongu->set_flag==0)
			{
				tongu->set_flag=1;
				tongu->reset_flag=0;
				rt_event_send(event,TONGU_SET);
			}
		}
		else{
			if(tongu->reset_flag==0)
			{
				tongu->set_flag=0;
				tongu->reset_flag=1;
				rt_event_send(event,TONGU_RESET);
			}
		}
	}
	else{
		tongu->set_counter=0;
		if(tongu->reset_flag==0)
		{
			tongu->set_flag=0;
			tongu->reset_flag=1;
			rt_event_send(event,TONGU_RESET);
		}
	}
}

enum CPR_BLOW_SET
{
	L_SIDE_SET=0x01,
	R_SIDE_SET,
	BOTH_SIDE_SET,
	L_SIDE_RESET,
	R_SIDE_RESET,
	BOTH_SIDE_RESET
};

//cpr吹气
uint16_t l_tv_old=0,r_tv_old=0,ltv_rtv_change_flag=0,tv_set_count;
void cpr_blow(rt_event_t event,BREATH_CTRL *l_lung, BREATH_CTRL *r_lung,uint16_t l_tv,uint16_t r_tv,uint8_t breath_cmd)
{
	uint8_t cpr_blow_state=0;
	if(!breath_cmd)//没有自主呼吸，才可用吹气
	{
		if(l_tv!=l_tv_old||r_tv!=r_tv_old)//有变化
		{
			l_tv_old=l_tv;
			r_tv_old=r_tv;
			ltv_rtv_change_flag=1;//
			tv_set_count=0;
			
		}
		else{
			ltv_rtv_change_flag=0;
		}
		
		if(ltv_rtv_change_flag==0&&(l_tv!=0||r_tv!=0))
		{
			tv_set_count++;
			if(tv_set_count>2)//600毫秒没变化，且有一个tv非零
			{
				l_tv=0;
				r_tv=0;//手动清零
				tv_set_count=11;
			}
		}
		
		//为了同步
		if(l_tv)
		{
			l_lung->set_counter++;
			if(l_lung->set_counter>0)
			{
				l_lung->set_counter=0;
				if(r_tv)
				{
					cpr_blow_state=BOTH_SIDE_SET;
				}
				else
				{
					cpr_blow_state=L_SIDE_SET;
				}
			}
		}
		else
		{
			l_lung->reset_counter++;
			if(l_lung->reset_counter>0)
			{
				l_lung->reset_counter=0;
				if(!r_tv)
				{
					cpr_blow_state=BOTH_SIDE_RESET;
				}
				else
				{
					cpr_blow_state=L_SIDE_RESET;
				}
			}	
		}
		
		if(r_tv)
		{
			r_lung->set_counter++;
			if(r_lung->set_counter>0)
			{
				r_lung->set_counter=0;
				if(l_tv)
				{
					cpr_blow_state=BOTH_SIDE_SET;
				}
				else
				{
					cpr_blow_state=R_SIDE_SET;
				}
			}
		}
		else
		{
			r_lung->reset_counter++;
			if(r_lung->reset_counter>0)
			{
				r_lung->reset_counter=0;
				if(!l_tv)
				{
					cpr_blow_state=BOTH_SIDE_RESET;
				}
				else
				{
					cpr_blow_state=R_SIDE_RESET;
				}
			}	
		}
		
		switch(cpr_blow_state)
		{
			case L_SIDE_SET:
				if(l_lung->set_flag==0)
				{
					l_lung->set_flag=1;
					l_lung->reset_flag=0;
					rt_event_send(event,LEFT_BREATH_SET);
				}
				break;
			case R_SIDE_SET:
				if(r_lung->set_flag==0)
				{
					r_lung->set_flag=1;
					r_lung->reset_flag=0;
					rt_event_send(event,RIGHT_BREATH_SET);
				}
				break;
			case BOTH_SIDE_SET:
				if(l_lung->set_flag==0)
				{
					l_lung->set_flag=1;
					l_lung->reset_flag=0;
					rt_event_send(event,LEFT_BREATH_SET);
				}
				if(r_lung->set_flag==0)
				{
					r_lung->set_flag=1;
					r_lung->reset_flag=0;
					rt_event_send(event,RIGHT_BREATH_SET);
				}
				break;
			case L_SIDE_RESET:
				if(l_lung->reset_flag==0)
				{
					l_lung->set_flag=0;
					l_lung->reset_flag=1;
					rt_event_send(event,LEFT_BREATH_RESET);
				}
				break;
			case R_SIDE_RESET:
				if(r_lung->reset_flag==0)
				{
					r_lung->set_flag=0;
					r_lung->reset_flag=1;
					rt_event_send(event,RIGHT_BREATH_RESET);
				}
				break;
			case BOTH_SIDE_RESET:
				if(l_lung->reset_flag==0)
				{
					l_lung->set_flag=0;
					l_lung->reset_flag=1;
					rt_event_send(event,LEFT_BREATH_RESET);
				}
				if(r_lung->reset_flag==0)
				{
					r_lung->set_flag=0;
					r_lung->reset_flag=1;
					rt_event_send(event,RIGHT_BREATH_RESET);
				}
				break;
		}
		
//		if(l_tv&&r_tv)//双侧
//		{
//			if(l_lung->set_flag==0)
//			{
//				l_lung->set_flag=1;
//				l_lung->reset_flag=0;
//				rt_event_send(event,LEFT_BREATH_SET);
//			}
//			if(r_lung->set_flag==0)
//			{
//				r_lung->set_flag=1;
//				r_lung->reset_flag=0;
//				rt_event_send(event,RIGHT_BREATH_SET);
//			}
//		}
////		else{
////			if(l_tv)//单左侧
////			{
////				if(l_lung->set_flag==0)
////				{
////					l_lung->set_flag=1;
////					l_lung->reset_flag=0;
////					rt_event_send(event,LEFT_BREATH_SET);
////				}
////				if(r_lung->reset_flag==0)
////				{
////					r_lung->set_flag=0;
////					r_lung->reset_flag=1;
////					rt_event_send(event,RIGHT_BREATH_RESET);
////				}
////			}
////			else if(r_tv)
////			{
////				if(r_lung->set_flag==0)
////				{
////					r_lung->set_flag=1;
////					r_lung->reset_flag=0;
////					rt_event_send(event,RIGHT_BREATH_SET);
////				}
////				if(l_lung->reset_flag==0)
////				{
////					l_lung->set_flag=0;
////					l_lung->reset_flag=1;
////					rt_event_send(event,LEFT_BREATH_RESET);
////				}
////			}
////			else{
////				if(l_lung->reset_flag==0)
////				{
////					l_lung->set_flag=0;
////					l_lung->reset_flag=1;
////					rt_event_send(event,LEFT_BREATH_RESET);
////				}
////				if(r_lung->reset_flag==0)
////				{
////					r_lung->set_flag=0;
////					r_lung->reset_flag=1;
////					rt_event_send(event,RIGHT_BREATH_RESET);
////				}
////			}
////			
////		}
	}
}

ALIGN(RT_ALIGN_SIZE)
static char valves_ctrl_thread_stack[1024];
struct rt_thread valves_ctrl_thread;
static void valves_ctrl_thread_entry(void *param)
{
	
	BREATH_CTRL breath,l_lung,r_lung;
	PNEU_CTRL pneu;
	HEMO_CTRL hemo;
	TONGU_CTRL tongu;

	breath_init(&breath);
	breath_init(&l_lung);
	breath_init(&r_lung);
	pneu_init(&pneu);
	hemo_init(&hemo);
	tongu_init(&tongu);
	
    while(1)
    {
    	//自主呼吸
		breathe(&chest_valves_event,&breath,VALVES_TICK,g_breathrate_cmd);
		
		//气胸
		pneu_set(&chest_valves_event, &pneu, g_pneu_cmd);
		
		//血胸
		hemo_set(&chest_valves_event, &hemo, g_hemo_cmd);
		
		//舌水肿
		tongu_set(&chest_valves_event, &tongu, g_tongu_cmd);

		//cpr吹气
		cpr_blow(&chest_valves_event,&l_lung, &r_lung,g_l_tv,g_r_tv,g_breathrate_cmd);
		
		rt_thread_delay(VALVES_TICK);	//200ms循环一次
		
    }
}

/**
	*	@name		cmb_valves_ctrl_init
	*	@brief		Init the valves ctrl thread
	*	@param		None
	*	@retval		None
	*/


int cmb_valves_ctrl_init(uint8_t prio)
{					


	//valves control thread init
    rt_thread_init(&valves_ctrl_thread,"valves_ctrl",
                   valves_ctrl_thread_entry,RT_NULL,
                   &valves_ctrl_thread_stack[0],
                   sizeof(valves_ctrl_thread_stack),prio,10);
    rt_thread_startup(&valves_ctrl_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
