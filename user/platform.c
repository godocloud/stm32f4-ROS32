/**
	********************************************************************
	*	@file			chest_board_V1.0.c
	*	@author		Jun.Lu
	*	@version	V1.0.0
	*	@date			25-Feb-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	This file is the master control program of Chest Main Board(CMB).
	*-----------------------------
	*
	*	Change Logs:
	*	Date							Author						Notes
	*	2013-2-25					Jun.Lu						Version V1.0.0
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "stm32f4xx.h"

/*	Global variables -----------------------------------------------*/
extern int nano_board_heartbeat_init(uint8_t prio);
extern int core_test_init(void);
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
enum
{
	WIFI_PRIO=2,
	RS232_PRIO,
	DATABACK_PRIO,
	PROTOCOL_PRIO,
	HW_MOTOR_PRIO,
	ROS_BRIDGE_PRIO,
	ROS_NODE_PRIO,
	LED_PRIO,
	DEBUG_PRIO
}PRIORITY;
/*	Private function prototypes ------------------------------------*/
extern int nano_board_heartbeat_init(uint8_t prio);
extern int cmb_hw_rs232_init(void);
extern int cmb_hw_protocol_init(void);
extern int cmb_hw_comm_overtime_init(void);
extern int cmb_hw_wifi_module_init(uint8_t prio);

extern int rosserial_bridge_init(uint8_t prio);
extern int ros_node_test_init(uint8_t prio);

extern int hw_motor_init(uint8_t prio);
/*	Private fuctions -----------------------------------------------*/

/**
	*	@brief	using for what
	*	@note		description
	*	@param	
	*	@retval	
	*/


/**
	*	@name	2 cmb_platform_init
	*	@brief	Chest Main Board Threads Init Functions Init
	*	@param	None
	*	@retval	None
	*/
void platform_init(void)
{
	
//	cmb_hw_wifi_module_init(WIFI_PRIO);
	cmb_hw_rs232_init();
//	cmb_hw_comm_overtime_init();
//	cmb_hw_protocol_init();
	hw_motor_init(HW_MOTOR_PRIO);
	rosserial_bridge_init(ROS_BRIDGE_PRIO);
//	ros_node_test_init(ROS_NODE_PRIO);
	nano_board_heartbeat_init(LED_PRIO);//25

}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
