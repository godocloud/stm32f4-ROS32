/**
	******************************************************************
	*	@file			pulse.c
	*	@author			Jun.Lu
	*	@brief
	******************************************************************
	*	@attention
	* Setup the pulse module and control it.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author					Notes
	*	2016-2-1			Jun.Lu					first implement
	*
	******************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>
#include "protocol.h"
#include "can_driver.h"
#include "pulse.h"

/*	Global variables -----------------------------------------------*/
extern struct rt_mailbox mb_for_data_sendback;
extern void back_data_push_to_mail(CMD_DATA_STRUCT *sendback_struct, 
									rt_mailbox_t sendback_buff_mailbox, 
									uint8_t sendback_structure_id, 
									uint8_t back_data[], uint8_t dlc);

/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
/**
	*	@name		GPIO_Configuration
	*	@brief		wifi module GPIO Configuration
	*	@param		None
	*	@retval		None
	*/

static void hw_init(void *t, uint8_t touchGpio, uint8_t touchPin, uint8_t beatGpio, uint8_t beatPin)
{
	Pulse* cthis=(Pulse*)t;
	
	cthis->Touchpin=(Gpio *)GpioNew();
	cthis->Touchpin->GPIOx=touchGpio;
	cthis->Touchpin->Pinx=touchPin;
	cthis->Touchpin->init(touchGpio,touchPin,IN,PP,NO);
	
	cthis->Beatpin=(Gpio *)GpioNew();
	cthis->Beatpin->GPIOx=beatGpio;
	cthis->Beatpin->Pinx=beatPin;
	cthis->Beatpin->init(beatGpio,beatPin,OUT,PP,NO);
	cthis->Beatpin->pinWrite(beatGpio, beatPin, 1);
}

static void init(void *t)
{
	Pulse* cthis=(Pulse*)t;
	cthis->power=1;
	cthis->rate=60;
	cthis->state=0;
}

static void setPulse(void *t, uint8_t power, uint8_t rate)
{
	Pulse* cthis=(Pulse*)t;
	cthis->power=power;
	cthis->rate=rate;
}

static void beat(void *t)
{
	Pulse* cthis=(Pulse*)t;
	Gpio* beatpin;
	beatpin=cthis->Beatpin;
	beatpin->pinWrite(beatpin->GPIOx,beatpin->Pinx,0);
	rt_thread_delay(cthis->power);
	beatpin->pinWrite(beatpin->GPIOx,beatpin->Pinx,1);
	rt_thread_delay(1);
}

static uint8_t getPulseState(void *t)
{
	Pulse* cthis=(Pulse*)t;
	Gpio* touchpin;
	touchpin=cthis->Touchpin;
	touchpin->pinState=touchpin->pinRead(touchpin->GPIOx,touchpin->Pinx);
	cthis->state=touchpin->pinState;
	
	return(touchpin->pinState);
}

//构造Pulse类
CTOR(Pulse)
	FUNCTION_SETTING(hw_init, hw_init);
	FUNCTION_SETTING(init, init);
	FUNCTION_SETTING(setPulse, setPulse);
	FUNCTION_SETTING(beat, beat);
	FUNCTION_SETTING(getPulseState, getPulseState);
END_CTOR

extern uint8_t g_l_femoral_power_cmd,g_r_femoral_power_cmd,g_heartrate_cmd,
				g_heartbeat_cmd,g_pulse2_state,g_l_femoral_cmd,g_r_femoral_cmd;
ALIGN(RT_ALIGN_SIZE)
static char pulse_thread_stack[1024];
struct rt_thread pulse_thread;
static void pulse_thread_entry(void *param)
{
	uint8_t backstate[2]={0};
	uint8_t pulse_touch_state_old=0;
	CMD_DATA_STRUCT *data_back2send;
	
	//实例化两个脉搏对象
	Pulse *left_femoral=(Pulse *)PulseNew();
	Pulse *right_femoral=(Pulse *)PulseNew();
	//初始化两个脉搏对象
	left_femoral->hw_init(left_femoral,C,9,C,8);
	left_femoral->init(left_femoral);
	right_femoral->hw_init(right_femoral,C,7,C,6);
	right_femoral->init(right_femoral);
	
    while(1)
    {
    	//设置脉搏参数
		if(g_l_femoral_power_cmd!=left_femoral->power ||
			g_heartrate_cmd!=left_femoral->rate)
		{
			left_femoral->setPulse(left_femoral,g_l_femoral_power_cmd,g_heartrate_cmd);

		}
		if(g_r_femoral_power_cmd!=right_femoral->power ||
			g_heartrate_cmd!=right_femoral->rate)
		{
			right_femoral->setPulse(right_femoral,g_r_femoral_power_cmd,g_heartrate_cmd);
		}
		
		//控制脉搏跳动
		if(g_heartbeat_cmd==1)
		{
			g_heartbeat_cmd=0;
			if(((left_femoral->getPulseState(left_femoral))==0)&&g_l_femoral_cmd)
			{
				left_femoral->beat(left_femoral);
				g_pulse2_state|=0x01;
			}
			else
			{
				g_pulse2_state&=0xFE;
			}
			if(((right_femoral->getPulseState(right_femoral))==0)&&g_r_femoral_cmd)
			{
				right_femoral->beat(right_femoral);
				g_pulse2_state|=0x02;
			}
			else
			{
				g_pulse2_state&=0xFD;
			}
		}
		
		//触摸回发
		if((g_pulse2_state&0x03)!=(pulse_touch_state_old&0x03))//股动脉有变化
		{
			backstate[0]=g_pulse2_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PULSE2,backstate,1);
			pulse_touch_state_old=g_pulse2_state;
		}
		
		rt_thread_delay(1);
    }
}

/**
	*	@name		cmb_hw_wifi_module_init
	*	@brief		Init the wifi module thread
	*	@param		None
	*	@retval		None
	*/

int cmb_pulse_init(uint8_t prio)
{						
	//can send thread init
    rt_thread_init(&pulse_thread,"pulse",
                   pulse_thread_entry,RT_NULL,
                   &pulse_thread_stack[0],
                   sizeof(pulse_thread_stack),prio,10);
    rt_thread_startup(&pulse_thread);
	
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
