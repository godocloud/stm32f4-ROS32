/**
	********************************************************************
	*	@file			protocol.h
	*	@author			Jun.Lu
	*	@version		V1.0.0
	*	@date			14-May-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	The header of protocol.c.
	*-----------------------------
	*
	*	Change Logs:
	*	Date					Author					Notes
	*	2013-5-14				Jun.Lu					Version V1.0.0
	*	2013-12-2 				Jun.Lu 					alpha
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
/*	Private typedef ------------------------------------------------*/
typedef enum
{
		//下行代码标头
		//**********模型人在线*********
		SI_MODELOFPERSON=0x00,
		//**********头部**********
		SI_PUPIL,
		SI_RESPONSE,
		SI_EYELIDS,
		SI_VOICE,
		SI_TONGU,//0X05
		SI_PURPLE,
		SI_PULSE,
		//**********胸部**********
		SI_BREATHRATE,
		SI_HEARTSOUND,
		SI_BREATHSOUND,//0X0A
		SI_PNEUMOTHORAX,
		SI_HAEMOTHORAX,
		SI_BLOODING,
		//**********四肢**********
		SI_ARMTIC,
		SI_ARMPUMP,//0X0F
		SI_PUMPSWITCH,
		SI_HEARTBEAT,
		SI_HEARTRATE,
		SI_SPO2,
		SI_SWITCH,
		SI_BATT_STUDY,
		SI_PULSE_POWER,//0X16
		SI_LEGPUMP,

		//上行代码标头
		SI_AIRWAY=0X20,
		SI_PUPILREFLECT,
		SI_PNEU_PUNCTURE,
		SI_HEMO_PUNCTURE,
		SI_TV,
		SI_CPR_HEIGHT,//0X25
		SI_CPR_POSITION,
		SI_ARM_HEMOSTASIS,
		SI_LEG_HEMOSTASIS,
		SI_BATT_LEVEL,
		SI_PRS_CYLINDER,//0X2A
		SI_PRS_PNEUMOTHORAX,
		SI_RATE_HEMOTHORAX,
		SI_VENTURI,
		SI_LIGHT_ADC,
		SI_ARM_ADC,//0X2F
		SI_LEG_ADC,
		SI_NECK_ANGLE,
		SI_PULSE1,
		SI_PULSE2,
		SI_ARM_VALVE,
		SI_BODY_VALVE,//0X35
		SI_SPO2_SWITCH,
		SI_CPR_ADC,
		SI_CHARGING,
		SI_LEG_VALVE,
		SI_NECK_ADC
		
}STRUCTION_INDEX;

typedef enum
{
	CAN_PUPIL=0x01,
	CAN_RESPONSE,
	CAN_EYELIDS,
	CAN_VOICE,
	CAN_PURPLE,
	CAN_NECK_PULSE,
	CAN_AIRWAY=0x08,
	CAN_PUPILREFLECT,
	CAN_NECK_ANGLE,
	CAN_CAROTID_TOUCH,

	CAN_LEFT_ARM_PULSE=0x11,
	CAN_LEFT_ARM_BLOODING,
	CAN_LEFT_ARMTIC,
	CAN_LEFT_ARM_HEMOSTASIS=0x17,
	CAN_LEFT_ARM_HEMO_ADC=0x19,
	CAN_LEFT_ARM_ARTERY_TOUCH,
	CAN_LEFT_ARM_VALVES_STATES,

	CAN_RIGHT_ARM_PULSE=0x21,
	CAN_RIGHT_ARM_BLOODING,
	CAN_RIGHT_ARMTIC,
	CAN_RIGHT_ARM_LIQUID_PUMP,
	CAN_RIGHT_ARM_HEMOSTASIS=0X27,
	CAN_RIGHT_ARM_HEMO_ADC=0X29,
	CAN_RIGHT_ARM_ARTERY_TOUCH,
	CAN_RIGHT_ARM_VALVES_STATES,

	CAN_LEFT_LEG_PULSE=0x31,
	CAN_LEFT_LEG_BLOODING,
	CAN_LEFT_LEG_LIQUID_PUMP=0X34,
	CAN_LEFT_LEG_HEMOSTASIS=0X37,
	CAN_LEFT_LEG_HEMO_ADC=0X39,
	CAN_LEFT_LEG_ARTERY_TOUCH,
	CAN_LEFT_LEG_VALVES_STATES,

	CAN_RIGHT_LEG_PULSE=0x41,
	CAN_RIGHT_LEG_BLOODING,
	CAN_RIGHT_LEG_LIQUID_PUMP=0X44,
	CAN_RIGHT_LEG_AIR_PUMP,
	CAN_RIGHT_LEG_HEMOSTASIS=0X47,
	CAN_RIGHT_LEG_AIR_PRESS,
	CAN_RIGHT_LEG_HEMO_ADC,
	CAN_RIGHT_LEG_ARTERY_TOUCH,
	CAN_RIGHT_LEG_VALVES_STATES,

	CAN_POWER_SWITCH=0X51,
	CAN_POWER_BATT_STUDY,
	CAN_POWER_LEVEL,
	CAN_CHARGING_ON,

	CAN_HEART_SOUND=0X61,
	CAN_L_BREATH_SOUND,
	CAN_PNEUMOTHORAX,
	CAN_HAEMOTHORAX,
	CAN_PNEU_PUNCTURE,
	CAN_HEMO_PUNCTURE,
	CAN_LEFT_TV,
	CAN_CPR_POSITION,
	CAN_PRS_PNEUMOTHORAX,
	CAN_RATE_HEMOTHORAX,
	CAN_RIGHT_TV,
	CAN_CPR_ADC,
	CAN_R_BREATH_SOUND,

	CAN_HEARTBEAT=0X88

}CAN_FILTER_ID;



typedef enum
{
	HEAD_CAN_ID=0x0a,
	LEFT_ARM_CAN_ID=0x10,
	RIGHT_ARM_CAN_ID=0x20,
	LEFT_LEG_CAN_ID=0x30,
	RIGHT_LEG_CAN_ID=0x40,
	POWER_CAN_ID=0x50,
	CHEST_CAN_ID=0x60,
	MAIN_CAN_ID=0x70,
	BROADCAST_CAN_ID=0x80

}INSIDE_CAN_ID;

typedef union{
	uint8_t cmd_byte;
	uint16_t cmd_2bytes;
	uint32_t cmd_3bytes;
	uint8_t	cmd_5bytes[5];
}CMD;

typedef struct
{	
	uint16_t struct_id;//struct id
	uint8_t can_id;//发给那块子板的can id
	uint8_t cmd_l;
	uint8_t cmd_data[8];//本次收到的命令
}CMD_DATA_STRUCT;

typedef struct
{
	uint8_t structure_id;
	uint16_t data;
	uint8_t bit16_flag;
}DATA_SENDBACK;

typedef struct
{
	uint16_t content_old;//保留上次的包信息
}REGEDIT;

typedef enum 
{
	TONGU=0,
	LEFT_LUNG,
	PNEU,
	RIGHT_LUNG,
	HEAD_BLOOD,
	HEMO,
	BLOOD1,
	BLOOD2
}VALVES_NO;
/*	Private define -------------------------------------------------*/
#define STURCTURE_MAX 		25

#define TONGU_SET   							(1<<0)//对应硬件out0
#define LEFT_BREATH_SET   						(1<<1)//对应硬件out1
#define PNEU_SET   								(1<<2)//对应硬件out2
#define RIGHT_BREATH_SET   						(1<<3)//对应硬件out3
#define HEMO_SET								(1<<5)//对应硬件out5
#define TONGU_RESET								(1<<8)//对应硬件out0
#define LEFT_BREATH_RESET   					(1<<9)//对应硬件out1
#define PNEU_RESET   							(1<<10)//对应硬件out2
#define RIGHT_BREATH_RESET   					(1<<11)//对应硬件out3
#define HEMO_RESET								(1<<13)//对应硬件out5


/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/







/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
