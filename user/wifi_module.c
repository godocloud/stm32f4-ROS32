/**
	********************************************************************
	*	@file			wifi_module.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* Setup the wifi module and contron it.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2014-6-10			Jun.Lu						beta
	*	2016-1-3			Jun.Lu						update for ebase
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "wifi_module.h"
#include <board.h>

/*	Global variables -----------------------------------------------*/
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
static void GPIO_Configuration(void)
{
	GPIO_Pin_Setup(E,13,OUT,PP,NO);//STATUS PIN=PE13
	GPIO_Pin_Setup(E,12,OUT,PP,NO);//RESET_PIN=PE12
	GPIO_Pin_Setup(E,11,IN,PP,NO);//BOOT_CONNCT_PIN=PE11
	GPIO_Pin_Setup(E,14,IN,PP,NO);//SETUP_PIN=PE14
}

/**
	*	@name		setup_wifi_module
	*	@brief		wifi module into setup states
	*	@param		None
	*	@retval		None
	*/
static void setup_wifi_module(void)
{
    WIFI_STATUS_L();
    WIFI_RESET_L();
    WIFI_RESET_H();
}

static void direct_tran_mode(void)
{
    WIFI_STATUS_H();
    WIFI_RESET_L();
    WIFI_RESET_H();
}

static _Bool wifi_setup_states(void)
{
    return (GPIO_Pin_Read(E,14));
}

_Bool wifi_is_connected(void)
{
	return (GPIO_Pin_Read(E,11));
}

uint8_t g_communication_start=0;
ALIGN(RT_ALIGN_SIZE)
static char wifi_module_thread_stack[1024];
struct rt_thread wifi_module_thread;
static void wifi_module_thread_entry(void *param)
{
    rt_time_t	next_delay;
    uint8_t setup_flag=0,direct_tran_flag=0;
	uint8_t wifi_state_init=0;
	
    GPIO_Configuration();
    
    while(1)
    {
       //2014.10.19注销以下wifi 连接判断的if
	   if(wifi_state_init==0)
		{
			if(wifi_is_connected()==0)
			{
				
				wifi_state_init=1;
//				rt_hw_usart_init();	
//				#ifdef RT_USING_CONSOLE
//				rt_console_set_device(CONSOLE_DEVICE);
//				#endif
//				rt_show_version();
				rt_thread_delay(300);
				g_communication_start=1;
			}
		}

        if((wifi_setup_states())&&setup_flag==0)//if true is setup state
        {
            setup_wifi_module();
            setup_flag=1;
            direct_tran_flag=0;
        }
        else if(!(wifi_setup_states())&&direct_tran_flag==0)
        {
            direct_tran_mode();
            direct_tran_flag=1;
            setup_flag=0;
        }
		
        next_delay=100;//1 second
        /* wait next key press */
        rt_thread_delay(next_delay);
    }
}

/**
	*	@name		cmb_hw_wifi_module_init
	*	@brief		Init the wifi module thread
	*	@param		None
	*	@retval		None
	*/
int cmb_hw_wifi_module_init(uint8_t prio)
{
    //sexuality thread init
    rt_thread_init(&wifi_module_thread,"wifi_module",
                   wifi_module_thread_entry,RT_NULL,
                   &wifi_module_thread_stack[0],
                   sizeof(wifi_module_thread_stack),prio,10);
    rt_thread_startup(&wifi_module_thread);
    
    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
