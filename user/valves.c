/**
	********************************************************************
	*	@file			valves.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* Setup the valves and control it.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2016-2-3			Jun.Lu						first implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>
#include "protocol.h"
#include "can_driver.h"
#include "valves.h"


/*	Global variables -----------------------------------------------*/
extern struct rt_event chest_valves_event;
extern struct rt_mailbox mb_for_data_sendback;
extern struct rt_mailbox cmd2can_mailbox;
extern void back_data_push_to_mail(CMD_DATA_STRUCT *sendback_struct, 
									rt_mailbox_t sendback_buff_mailbox, 
									uint8_t sendback_structure_id, 
									uint8_t back_data[], uint8_t dlc);
extern void cmd4can_push_to_mail(CanTxMsg *TxMessage, 
								rt_mailbox_t cmd4can_buff_mailbox, 
								uint8_t can_fi, uint8_t num_cmd, 
								uint8_t can_si, uint8_t cmd[]);

/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
extern void* GpioNew();
static void hw_init(void*t, uint8_t Gpiox, uint8_t enPinx, uint8_t nfaultPinx, 
					uint8_t dinPinx, uint8_t latchPinx, uint8_t clkPinx,
					uint8_t doutPinx)
{
	DRV8860* cthis=(DRV8860*)t;

	cthis->EnPin=(Gpio *)GpioNew();
	cthis->EnPin->GPIOx=Gpiox;
	cthis->EnPin->Pinx=enPinx;
	cthis->EnPin->init(Gpiox,enPinx,OUT,PP,PULLUP);

	cthis->nFaultPin=(Gpio *)GpioNew();
	cthis->nFaultPin->GPIOx=Gpiox;
	cthis->nFaultPin->Pinx=nfaultPinx;
	cthis->nFaultPin->init(Gpiox,nfaultPinx,IN,PP,PULLUP);

	cthis->DinPin=(Gpio *)GpioNew();
	cthis->DinPin->GPIOx=Gpiox;
	cthis->DinPin->Pinx=dinPinx;
	cthis->DinPin->init(Gpiox,dinPinx,IN,PP,PULLUP);

	cthis->LatchPin=(Gpio *)GpioNew();
	cthis->LatchPin->GPIOx=Gpiox;
	cthis->LatchPin->Pinx=latchPinx;
	cthis->LatchPin->init(Gpiox,latchPinx,OUT,PP,PULLUP);

	cthis->ClkPin=(Gpio *)GpioNew();
	cthis->ClkPin->GPIOx=Gpiox;
	cthis->ClkPin->Pinx=clkPinx;
	cthis->ClkPin->init(Gpiox,clkPinx,OUT,PP,PULLUP);

	cthis->DoutPin=(Gpio *)GpioNew();
	cthis->DoutPin->GPIOx=Gpiox;
	cthis->DoutPin->Pinx=doutPinx;
	cthis->DoutPin->init(Gpiox,doutPinx,OUT,PP,PULLUP);
}

static void init(void *t)
{
	DRV8860* cthis=(DRV8860*)t;

	cthis->faultValue=0;
	cthis->cmd=0;
}

void DRV8860_DataWrite(void *t, uint8_t data)
{
	DRV8860* cthis=(DRV8860*)t;
	uint8_t i;
	Gpio* clkpin, *doutpin, *latchpin;
	clkpin=cthis->ClkPin;
	doutpin=cthis->DoutPin;
	latchpin=cthis->LatchPin;
	
	rt_enter_critical();
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	
	for(i=8;i!=0;i--)
	{
		(data & 0x80)?doutpin->pinWrite(doutpin->GPIOx,doutpin->Pinx,1):doutpin->pinWrite(doutpin->GPIOx,doutpin->Pinx,0);
		rt_thread_delay(1);
		clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
		data=(data<<1);
		rt_thread_delay(1);
		if(i>1)
			clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);

	}
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	
	rt_exit_critical();
}

void DRV8860_FaultReset(void *t)
{
	DRV8860* cthis=(DRV8860*)t;
	Gpio* clkpin, *latchpin;
	clkpin=cthis->ClkPin;
	latchpin=cthis->LatchPin;

	rt_enter_critical();
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	rt_thread_delay(1);
	
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	rt_thread_delay(1);
	
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	rt_thread_delay(1);
	
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	rt_thread_delay(1);
	
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
	rt_thread_delay(1);
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	
	rt_exit_critical();
}

rt_uint16_t DRV8860_FaultRead(void *t)
{
	DRV8860* cthis=(DRV8860*)t;
	uint8_t i;
	uint16_t fault=0x00;
	Gpio* clkpin, *dinpin, *doutpin, *latchpin;
	clkpin=cthis->ClkPin;
	dinpin=cthis->DinPin;
	doutpin=cthis->DoutPin;
	latchpin=cthis->LatchPin;

	rt_enter_critical();
	
	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
	rt_thread_delay(1);
	doutpin->pinWrite(doutpin->GPIOx,doutpin->Pinx,0);
	rt_thread_delay(1);
	
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	rt_thread_delay(1);
	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,1);
	rt_thread_delay(1);

	for(i=0;i<16;i++)
	{
		clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,1);
		rt_thread_delay(1);
		clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
		rt_thread_delay(1);
		if(dinpin->pinRead(dinpin->GPIOx, dinpin->Pinx))
		{
			fault|=(0x8000>>(i+1));
		}
	}
//	
//	clkpin->pinWrite(clkpin->GPIOx,clkpin->Pinx,0);
//	rt_thread_delay(2);
//	latchpin->pinWrite(latchpin->GPIOx,latchpin->Pinx,0);
	rt_thread_delay(1);

	rt_exit_critical();
	
	return fault;
}

void en(void *t, uint8_t state)
{
	DRV8860* cthis=(DRV8860*)t;
	Gpio* enpin;
	
	enpin=cthis->EnPin;
	
	enpin->pinWrite(enpin->GPIOx, enpin->Pinx, state);
}

//构造DRV8860类
CTOR(DRV8860)
	FUNCTION_SETTING(hw_init, hw_init);
	FUNCTION_SETTING(init, init);
	FUNCTION_SETTING(writeData, DRV8860_DataWrite);
	FUNCTION_SETTING(readData, DRV8860_FaultRead);
	FUNCTION_SETTING(reset, DRV8860_FaultReset);
	FUNCTION_SETTING(en, en);
END_CTOR


char  _crol_(char temp,char num)
{
      _Bool FFlag;
    while(num--)
    {
        FFlag=temp&0x80;
        temp<<=1;
        temp|=FFlag;
    }
    return(temp);
}

extern uint8_t g_chest_valves_states,g_body_blooding_cmd,g_tongu_cmd;
uint16_t g_v_states=0;
uint8_t g_v_cmd=0;
extern uint8_t g_leg_pump_cmd;
ALIGN(RT_ALIGN_SIZE)
static char valves_thread_stack[1024];
struct rt_thread valves_thread;
static void valves_thread_entry(void *param)
{

	uint8_t valves_cmd_old=0,i=0,blooding_flag=0;
	rt_uint32_t e;
	uint8_t cmd[2]={0};
	CanTxMsg *TxMessage;
	
	//实例化胸部电磁阀对象
	DRV8860 *chest_valves=(DRV8860 *)DRV8860New();
	//初始化电磁阀对象
	chest_valves->hw_init(chest_valves, D, 8, 9, 10, 11, 12, 13);
	chest_valves->init(chest_valves);

	chest_valves->en(chest_valves, 1);
	chest_valves->writeData(chest_valves, 0x00);
    while(1)
    {

		//电磁阀开关控制
		//头血
		if(g_body_blooding_cmd&0x03)
		{
			chest_valves->cmd|=0x10;
		}
		else{
			chest_valves->cmd&=0xEF;
		}
		//胸血
		if(g_body_blooding_cmd&0x0C)
		{
			chest_valves->cmd|=0x40;
		}
		else{
			chest_valves->cmd&=0xBF;
		}
		//腹血
		if(g_body_blooding_cmd&0x30)
		{
			chest_valves->cmd|=0x80;
		}
		else{
			chest_valves->cmd&=0x7F;
		}

		//呼吸
		//气胸
		//血胸
		//舌水肿
		if (rt_event_recv(&chest_valves_event, TONGU_SET | LEFT_BREATH_SET | PNEU_SET |
			RIGHT_BREATH_SET | HEMO_SET | TONGU_RESET | LEFT_BREATH_RESET | 
			PNEU_RESET | RIGHT_BREATH_RESET | HEMO_RESET,
			RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
			1, &e) == RT_EOK)
		{
			switch(e)
			{
				case TONGU_SET:
					chest_valves->cmd|=TONGU_SET;
					break;
				case LEFT_BREATH_SET:
					chest_valves->cmd|=LEFT_BREATH_SET;
					break;
				case PNEU_SET:
					chest_valves->cmd|=PNEU_SET;
					break;
				case RIGHT_BREATH_SET:
					chest_valves->cmd|=RIGHT_BREATH_SET;
					break;
				case HEMO_SET:
					chest_valves->cmd|=HEMO_SET;
					break;
				case LEFT_BREATH_SET|RIGHT_BREATH_SET:
					chest_valves->cmd|=LEFT_BREATH_SET;
					chest_valves->cmd|=RIGHT_BREATH_SET;
					break;
				
				case TONGU_RESET:
					chest_valves->cmd&=(~TONGU_SET);
					break;
				case LEFT_BREATH_RESET:
					chest_valves->cmd&=(~LEFT_BREATH_SET);
					break;
				case PNEU_RESET:
					chest_valves->cmd&=(~PNEU_SET);
					break;
				case RIGHT_BREATH_RESET:
					chest_valves->cmd&=(~RIGHT_BREATH_SET);
					break;
				case HEMO_RESET:
					chest_valves->cmd&=(~HEMO_SET);
					break;
				case LEFT_BREATH_RESET|RIGHT_BREATH_RESET:
					chest_valves->cmd&=(~LEFT_BREATH_SET);
					chest_valves->cmd&=(~RIGHT_BREATH_SET);
					break;
				
				default:break;
				
			}
		}
		if((chest_valves->cmd&0xF0)&&blooding_flag==0)//有出血
		{
			cmd[0]=0x01;
			cmd[1]=g_leg_pump_cmd&0x7F;
			blooding_flag=1;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_LEG_CAN_ID,2,CAN_RIGHT_LEG_LIQUID_PUMP,cmd);
		}
		else if(((chest_valves->cmd&0xF0)==0)&&blooding_flag==1)//没有出血
		{
			cmd[0]=0x00;
			blooding_flag=0;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_LEG_CAN_ID,1,CAN_RIGHT_LEG_LIQUID_PUMP,cmd);
		}
		
		//更新电磁阀工作指令
		if(chest_valves->cmd!=valves_cmd_old)
		{
			chest_valves->writeData(chest_valves, chest_valves->cmd);
			rt_kprintf("breath=%d\r\n",chest_valves->cmd);
			chest_valves->faultValue=chest_valves->readData(chest_valves);
			valves_cmd_old=chest_valves->cmd;
		}
		rt_thread_delay(1);

		//chest_valves->writeData(chest_valves, valves_cmd_old);
		chest_valves->reset(chest_valves);
		chest_valves->faultValue=chest_valves->readData(chest_valves);
		g_v_states=chest_valves->faultValue;
//		rt_kprintf("fault1=0x%x\r\n",chest_valves->faultValue);
//		chest_valves->reset(chest_valves);

		rt_thread_delay(1);

    }
}

ALIGN(RT_ALIGN_SIZE)
static char valves_states_thread_stack[1024];
struct rt_thread valves_states_thread;
static void valves_states_thread_entry(void *param)
{
	uint8_t i;
	uint16_t valves_states=0;
	uint8_t valves_OCP=0,valves_OLD=0;
	uint16_t faultValue_old=0xFFFF;
	uint8_t valve_state[8]={0xFF};//each valve state
	uint8_t valves_backstate[2]={0};//data for send back
	CMD_DATA_STRUCT *data_back2send;
	
	while(1)
    {
		for(i=0;i<8;i++)
		{
			if(((g_v_cmd>>i)&0x01)==0)//OFF will clear high 8bits of faultValue
			{
				valves_OLD&=_crol_(0xFE,i);//clear corresponding lower bit
				valves_OLD|=(g_v_states&(0x01<<i));
			}
			else//ON will clear lower 8bits of faultValue
			{
				valves_OCP&=_crol_(0xFE,i);//clear corresponding higher bit
				valves_OCP|=((g_v_states>>8)&(0x01<<i));
			}
		}
		valves_states=(valves_OCP<<8)|(valves_OLD);
//		rt_kprintf("fault_MAIN=0x%x\r\n",valves_states);
		if(valves_states!=faultValue_old)//valves states changes
		{
			for(i=0;i<8;i++)
			{
				if((valves_states>>i)&&0x01==1)//low 8 bits means open load fault
				{
					valve_state[i]=0x02;
				}
				else if(valves_states>>(i+8)&&0x01==1)//high 8 bits means over current fault
				{
					valve_state[i]=0x00;
				}
				else{
					valve_state[i]=0x01;
				}
			}
			//处理上报值
			valves_backstate[1]=0x00;
			valves_backstate[0]=0x00;//先清零
			valves_backstate[1]=(valve_state[AIR_HEAD]<<6)+(valve_state[AIR_R_LUNG]<<4)+(valve_state[AIR_L_LUNG]<<2)+valve_state[AIR_PNEU];
			valves_backstate[0]=(valve_state[BLOOD_HEAD]<<6)+(valve_state[BLOOD_HEMO]<<4)+(valve_state[BLOOD_1]<<2)+valve_state[BLOOD_2];
			//上报状态
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_BODY_VALVE,valves_backstate,2);
			//更新值
			faultValue_old=valves_states;
		}
		
		rt_thread_delay(100);
	}
}


int cmd_valves_ctrl_event_init(void)
{
	rt_err_t result;

    /* 初始化事件对象 */
    rt_event_init(&chest_valves_event, "event", RT_IPC_FLAG_FIFO);
    if (result != RT_EOK)
    {
        rt_kprintf("init event failed.\n");
        return -1;
    }
	return 0;
}
/**
	*	@name		cmb_hw_valves_init
	*	@brief		Init the valves thread
	*	@param		None
	*	@retval		None
	*/

int cmb_valves_init(uint8_t prio)
{						
	//init event
	cmd_valves_ctrl_event_init();
	
	//can send thread init
    rt_thread_init(&valves_thread,"valves",
                   valves_thread_entry,RT_NULL,
                   &valves_thread_stack[0],
                   sizeof(valves_thread_stack),prio,10);
    rt_thread_startup(&valves_thread);
	
	rt_thread_init(&valves_states_thread,"valves_states",
                   valves_states_thread_entry,RT_NULL,
                   &valves_states_thread_stack[0],
                   sizeof(valves_states_thread_stack),prio+1,10);
    rt_thread_startup(&valves_states_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
