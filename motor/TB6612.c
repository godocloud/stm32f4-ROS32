/**
**********************************************************************
*	@file		TB6612.c
*	@author		Jun.Lu
*	@version	V1.0
*	@date		15-Aug-2016
*	@brief		
**********************************************************************
*	@attention
*	This file is the control of motor drvier IC TB6612.
*
*   @note
*   Left_Motor:
*   PWM PIN         PE9             TIM1_CH1
*   IN1 PIN         PE7             GPIO
*   IN2 PIN         PE8             GPIO
*   Right_Motor:
*   PWM PIN         PE11            TIM1_CH2
*   IN1 PIN         PE12            GPIO
*   IN2 PIN         PE13            GPIO
*-----------------------------
*
*	Change Logs:
*	Date					Author					Notes
*	2016-8-15				Jun.Lu					Version V1.0
*
**********************************************************************
*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "TB6612.h"
#include "stm32f4_TIMER_Drv.h"

/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
/**
	*	@name		GPIO_Configuration
	*	@brief		Eyes GPIO Configuration
	*	@param		None
	*	@retval		None
	*/
static void hw_init(void*t, uint8_t pwm_gpiox, uint8_t pwm_pinx,
                       uint8_t in1_gpiox, uint8_t in1_pinx,
                       uint8_t in2_gpiox, uint8_t in2_pinx)
{
    Motor_tb6612* cthis=(Motor_tb6612*)t;
    
    cthis->pwmPin=(Gpio *)GpioNew();
    cthis->pwmPin->GPIOx=pwm_gpiox;
    cthis->pwmPin->Pinx=pwm_pinx;
    cthis->pwmPin->init(pwm_gpiox,pwm_pinx,AF,PP,PULLUP);
    
    cthis->in1Pin=(Gpio *)GpioNew();
    cthis->in1Pin->GPIOx=in1_gpiox;
    cthis->in1Pin->Pinx=in1_pinx;
    cthis->in1Pin->init(in1_gpiox,in1_pinx,OUT,PP,NO);
    cthis->in1Pin->pinWrite(in1_gpiox, in1_pinx, 0);
    
    cthis->in2Pin=(Gpio *)GpioNew();
    cthis->in2Pin->GPIOx=in2_gpiox;
    cthis->in2Pin->Pinx=in2_pinx;
    cthis->in2Pin->init(in2_gpiox,in2_pinx,OUT,PP,NO);
    cthis->in2Pin->pinWrite(in2_gpiox, in2_pinx, 0);
}
static void hw_timer_init(void *t, uint8_t timer_num, uint8_t channel_num,
                     uint32_t timer_freq, uint16_t period,
                     uint16_t pulse)
{
    Motor_tb6612* cthis=(Motor_tb6612*)t;
    
    cthis->ad_timer=(Simple_Timer *)Simple_TimerNew();
	cthis->ad_timer->timer_num=timer_num;
	cthis->ad_timer->channel_num=channel_num;
	cthis->ad_timer->timer_freq=timer_freq;
	cthis->ad_timer->period=period;
	cthis->ad_timer->pulse=pulse;
	
	cthis->ad_timer->simple_init(cthis->ad_timer,timer_num, channel_num, timer_freq, period, pulse);
}
static void init(void *t)
{
    Motor_tb6612* cthis=(Motor_tb6612*)t;
	
    cthis->state=STOP;
	cthis->duty=0;
}

static void set_duty(void *t, uint16_t duty)//duty:0~100%
{
    Motor_tb6612* cthis=(Motor_tb6612*)t;
	
	
    cthis->duty=duty;
	cthis->ad_timer->duty=duty*(cthis->ad_timer->period/100);//ad_timer's duty is the real data may bigger than 100
    cthis->ad_timer->simple_set_duty(cthis->ad_timer, cthis->ad_timer->timer_num,
									cthis->ad_timer->channel_num, cthis->ad_timer->duty);
}

static uint16_t get_duty(void *t)
{
	Motor_tb6612* cthis=(Motor_tb6612*)t;
	
	return(cthis->duty);
}

static void update_state(void *t, uint8_t state)
{
	Motor_tb6612* cthis=(Motor_tb6612*)t;
	
	cthis->state=state;
	
	switch(state)
	{
		case STOP:
			cthis->in1Pin->pinWrite(cthis->in1Pin->GPIOx, cthis->in1Pin->Pinx, 0);
			cthis->in2Pin->pinWrite(cthis->in2Pin->GPIOx, cthis->in2Pin->Pinx, 0);
			break;
		case CW:
			cthis->in1Pin->pinWrite(cthis->in1Pin->GPIOx, cthis->in1Pin->Pinx, 1);
			cthis->in2Pin->pinWrite(cthis->in2Pin->GPIOx, cthis->in2Pin->Pinx, 0);
			break;
		case CCW:
			cthis->in1Pin->pinWrite(cthis->in1Pin->GPIOx, cthis->in1Pin->Pinx, 0);
			cthis->in2Pin->pinWrite(cthis->in2Pin->GPIOx, cthis->in2Pin->Pinx, 1);
			break;
		default:break;
	}
}

static uint8_t get_state(void *t)
{
	Motor_tb6612* cthis=(Motor_tb6612*)t;
	
	return(cthis->state);
}



//
CTOR(Motor_tb6612)
	FUNCTION_SETTING(hw_init, hw_init);
	FUNCTION_SETTING(hw_timer_init, hw_timer_init);
	FUNCTION_SETTING(init, init);
	FUNCTION_SETTING(set_duty, set_duty);
	FUNCTION_SETTING(get_duty, get_duty);
	FUNCTION_SETTING(update_state, update_state);
	FUNCTION_SETTING(get_state, get_state);
END_CTOR



ALIGN(RT_ALIGN_SIZE)
static char thread_hw_motor_stack[512];
struct rt_thread thread_hw_motor;
static void rt_thread_entry_hw_motor(void* parameter)
{
	//new two motor objects
	Motor_tb6612 *left_motor=(Motor_tb6612 *)Motor_tb6612New();
	Motor_tb6612 *right_motor=(Motor_tb6612 *)Motor_tb6612New();
	//init the two motor objects
	left_motor->hw_init(left_motor,E,9,E,7,E,8);
	//period=200, timer_freq=20KHz*200=4MHz,pulse=100,duty_cycle=50%
	left_motor->hw_timer_init(left_motor,1,1,4000000,200,100);
	left_motor->init(left_motor);
	
	right_motor->hw_init(right_motor,E,11,E,12,E,13);
	//same as left parameters
	right_motor->hw_timer_init(right_motor,1,2,4000000,200,100);
	right_motor->init(right_motor);
	

	while (1)
    {
		//STEP 1st: 20% in both wheels 
		left_motor->update_state(left_motor,CW);
		left_motor->set_duty(left_motor,20);//20%,since period is 200,real duty set is 40
		right_motor->update_state(right_motor,CCW);//wheels in different directions
		right_motor->set_duty(right_motor,20);
		rt_thread_delay(5*RT_TICK_PER_SECOND);
		
		//STEP 2nd: 50%
		left_motor->update_state(left_motor,CW);
		left_motor->set_duty(left_motor,50);//50%,since period is 200,real duty set is 40
		right_motor->update_state(right_motor,CCW);//wheels in different directions
		right_motor->set_duty(right_motor,50);
		rt_thread_delay(5*RT_TICK_PER_SECOND);
		
		//STEP 3rd: stop
		left_motor->update_state(left_motor,STOP);
		left_motor->set_duty(left_motor,0);//20%,since period is 200,real duty set is 40
		right_motor->update_state(right_motor,STOP);//wheels in different directions
		right_motor->set_duty(right_motor,0);
		rt_thread_delay(5*RT_TICK_PER_SECOND);
		
		//SETP 4th: 50% change direction
		left_motor->update_state(left_motor,CCW);
		left_motor->set_duty(left_motor,50);//20%,since period is 200,real duty set is 40
		right_motor->update_state(right_motor,CW);//wheels in different directions
		right_motor->set_duty(right_motor,50);
		rt_thread_delay(5*RT_TICK_PER_SECOND);
		
		
    }
}

int hw_motor_init(uint8_t prio)
{
    rt_thread_init(&thread_hw_motor,
                   "hw_motor",
                   rt_thread_entry_hw_motor,
                   RT_NULL,
                   &thread_hw_motor_stack[0],
                   sizeof(thread_hw_motor_stack),prio,5);
    rt_thread_startup(&thread_hw_motor);

    return 0;
}






/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
