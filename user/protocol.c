/**
 ********************************************************************
 *	@file			protocol.c
 *	@author		Jun.Lu
 *	@version	V1.0.0
 *	@date			14-May-2013
 *	@brief		
 ********************************************************************
 *	@attention
 *	This file is used to parse from cmd or data.
 *-----------------------------
 *
 *	Change Logs:
 *	Date					Author					Notes
 *	2013-5-14				Jun.Lu					Version V1.0.0
 *	2013Äê12ÔÂ27ÈÕÔö¼Ó¾±²¿Ç¿Ö±¸øÍÎ°åÔö¼ÓÁËÒ»¸öÄÚ²¿Í¨Ñ¶
 * 	2014Äê2ÔÂ14ÈÕÔö¼Óliquid_pump_cmd_part1£¬ÎªÁË·ÀÖ¹Á½¸öº¯ÊýÍ¬Ê±²Ù×÷
 *	2016-1-12				Jun.Lu 					CMB_COM
 *
 ********************************************************************
 */

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "protocol.h"
#include "RS232.h"

#include "stm32f4xx.h"

/*	Global variables -----------------------------------------------*/
//CMB_COM
struct rt_mailbox cmd2can_mailbox;//´ý·¢ËÍ¸øcanµÄ»º´æÓÊÏä
static char cmd2can_mailbox_pool[200];

extern struct rt_semaphore pulse_ctrl_sem;
extern struct rt_semaphore blooding_ctrl_sem;
extern struct rt_semaphore heart_sound_ctrl_sem;
extern struct rt_semaphore breath_sound_ctrl_sem;

extern struct rt_mailbox mb_for_data_sendback;//
extern struct rt_mailbox mb_for_data_sendback_special;
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
void cmd_covent2can(CanTxMsg *TxMessage,uint8_t can_fi,uint8_t num_cmd,uint8_t can_si,uint8_t cmd[])
{
	uint8_t i;
	TxMessage->StdId=can_fi;
	TxMessage->DLC=(num_cmd+1);
	TxMessage->Data[0]=can_si;
	for(i=0;i<num_cmd;i++)
	{
		TxMessage->Data[i+1]=cmd[i];
	}
}

void cmd4can_push_to_mail(CanTxMsg *TxMessage, rt_mailbox_t cmd4can_buff_mailbox, uint8_t can_fi, uint8_t num_cmd, uint8_t can_si, uint8_t cmd[])
{
	uint8_t i;
	
	TxMessage=rt_malloc(sizeof(CanTxMsg));
	
	TxMessage->StdId=can_fi;
	TxMessage->IDE=CAN_ID_STD;
	TxMessage->RTR=CAN_RTR_DATA;
	TxMessage->DLC=(num_cmd+1);
	TxMessage->Data[0]=can_si;
	for(i=0;i<num_cmd;i++)
	{
		TxMessage->Data[i+1]=cmd[i];
	}
	rt_mb_send(cmd4can_buff_mailbox, (rt_uint32_t)TxMessage);
}

void back_data_push_to_mail(CMD_DATA_STRUCT *sendback_struct, rt_mailbox_t sendback_buff_mailbox, 
							uint8_t sendback_structure_id, uint8_t back_data[], uint8_t dlc)
{
	uint8_t i;
	sendback_struct=rt_malloc(sizeof(DATA_SENDBACK));
	sendback_struct->struct_id=sendback_structure_id;
	for(i=0;i<dlc;i++)
	{
		(*sendback_struct).cmd_data[i]=back_data[i];
	}
	(*sendback_struct).cmd_l=dlc;
	rt_mb_send(sendback_buff_mailbox, (rt_uint32_t)sendback_struct);
}

/**
 *	@name		cmd_update
 *	@brief		´ÓÊý¾Ý½»»»ÖÐÐÄÊÕµÄÃüÁî£¬½âÎö
 *	@param		None
 *	@retval		None
 */
//Ö÷¿Ø°åÓÃµ½µÄÈ«¾Ö±äÁ¿
uint8_t g_tongu_cmd=0,g_breathrate_cmd=0,g_spo2_cmd=0,g_l_femoral_cmd=0,g_r_femoral_cmd=0,
		g_body_blooding_cmd=0,g_limbs_blooding_cmd=0,g_heartrate_cmd=0,
		g_l_femoral_power_cmd=0,g_r_femoral_power_cmd=0,g_pulse1_state=0,g_pulse2_state=0,
		g_arm_hemostasis_state=0,g_left_arm_valve_state=0,g_right_arm_valve_state=0,
		g_l_arm_hemo_adc_state=0,g_r_arm_hemo_adc_state,g_leg_hemostasis_state=0,
		g_l_leg_hemo_adc_state=0,g_r_leg_hemo_adc_state=0,g_left_leg_valve_state=0,
		g_right_leg_valve_state=0,g_heartbeat_cmd=0,g_l_pulse_cmd=0,g_r_pulse_cmd=0,
		g_chest_valves_states=0,g_heartbeat_sync_flag=0,g_pneu_cmd=0,g_hemo_cmd=0,g_leg_pump_cmd=0;
uint8_t g_pulse_level[3]={0},g_heart_sound_set[2]={0},g_l_lung_sound_set[2]={0},g_r_lung_sound_set[2]={0};
//将从wifi收到的数据转发到can发送线程
void cmd_update(uint8_t structure_name[],
	uint8_t imformation[],uint8_t imformation_length)
{
	uint8_t  content_8,cmd[8]={0},blooding_change_test=0;
	uint16_t order;
	CanTxMsg *TxMessage;

	order=structure_name[1]*256+structure_name[0];
	//order=structure_name[0];

	//rt_kprintf("structure_id=%x\r\n",order);
	switch(imformation_length)
	{
	case 1://Ò»¸ö×Ö½ÚÊý¾Ý
		content_8=imformation[0];
		switch(order)
		{
		case SI_PUPIL://ÉèÖÃÍ«¿×´óÐ¡
		case SI_RESPONSE://ÉèÖÃÍ«¿×¶Ô¹â·´Éä
		case SI_EYELIDS://ÑÛíú´óÐ¡
			cmd[0]=content_8&0x0f;
			cmd[1]=(content_8>>4)&0x0f;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,HEAD_CAN_ID,2,order,cmd);
			break;
		case SI_PURPLE://×Ïç¤¿ª¹Ø
			cmd[0]=content_8;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,HEAD_CAN_ID,1,CAN_PURPLE,cmd);
			break;
		case SI_PNEUMOTHORAX://ÆøÐØÉèÖÃ
			cmd[0]=content_8;
			g_pneu_cmd=content_8;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,1,CAN_PNEUMOTHORAX,cmd);
			break;
		case SI_HAEMOTHORAX://ÑªÐØÉèÖÃ
			cmd[0]=content_8;
			g_hemo_cmd=content_8;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,1,CAN_HAEMOTHORAX,cmd);
			break;
		case SI_ARMPUMP://ÊÖ±ÛÒº±Ã¹¦ÂÊ
			cmd[0]=(content_8>>7)&0x01;
			cmd[1]=content_8&0x7F;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_ARM_CAN_ID,2,CAN_RIGHT_ARM_LIQUID_PUMP,cmd);
			break;
		case SI_HEARTBEAT://ÐÄÌø°ü
			g_heartbeat_cmd=1;
			g_heartbeat_sync_flag=1;
			rt_sem_release(&heart_sound_ctrl_sem);
			cmd[0]=0x00;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,BROADCAST_CAN_ID,1,CAN_HEARTBEAT,cmd);
			break;
		case SI_TONGU://ÉàË®Ö×ÓÐÎÞ
			g_tongu_cmd=content_8;
			break;
		case SI_BREATHRATE://ºôÎüÆµÂÊ
			g_breathrate_cmd=content_8;
			break;
		case SI_SPO2://ÑªÑõÅ¨¶È
			g_spo2_cmd=content_8;
			break;
		case SI_SWITCH://Èí¼þ¿ª¹Ø»ú
			cmd[0]=content_8;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,POWER_CAN_ID,1,CAN_POWER_SWITCH,cmd);
			break;
		case SI_BATT_STUDY://µç³ØÑ§Ï°¿ª¹Ø
			cmd[0]=content_8;
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,POWER_CAN_ID,1,CAN_POWER_BATT_STUDY,cmd);
			break;
		case SI_LEGPUMP:
			g_leg_pump_cmd=content_8;
			break;
		default:break;
		}
		break;
	/**/case 2://Á½¸ö×Ö½ÚÊý¾Ý
		//content_16=imformation[1]*256+imformation[0];
		switch(order)
		{
		case SI_VOICE://Í·²¿ÓïÒô
			cmd[0]=imformation[0];
			cmd[1]=imformation[1];
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,HEAD_CAN_ID,2,CAN_VOICE,cmd);
			break;
		case SI_HEARTSOUND://ÐÄÒô
			cmd[0]=imformation[0];
			cmd[1]=imformation[1];
			g_heart_sound_set[0]=cmd[0];
			g_heart_sound_set[1]=cmd[1];
			//cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_HEART_SOUND,cmd);
			break;
		case SI_PULSE://脉搏开关
			g_l_pulse_cmd=imformation[0];
			g_r_pulse_cmd=imformation[1];
			rt_sem_release(&pulse_ctrl_sem);//release a semaphore after update
			break;
		case SI_BLOODING://³öÑª£¬ËÄÖ«µÄ·Åµ½Ö÷°åÅÐ¶ÏÔÙ·¢ËÍ
			g_body_blooding_cmd=imformation[0];
			g_limbs_blooding_cmd=imformation[1];
			rt_sem_release(&blooding_ctrl_sem);//release a semaphore after update
			break;
		case SI_HEARTRATE://ÐÄÂÊ
			g_heartrate_cmd=imformation[1]*256+imformation[0];
			break;
		case SI_PUMPSWITCH://Æø±Ã¿ØÖÆ
			cmd[0]=imformation[0];
			cmd[1]=imformation[1];
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_LEG_CAN_ID,2,CAN_RIGHT_LEG_AIR_PUMP,cmd);
			break;
		default:break;
		}
		break;
	/**/case 3://Èý¸ö×Ö½ÚÊý¾Ý£¬Í·Ò»¸ö×Ö½Ú²¹Áã
		//content_32=(imformation[2]<<16)+(imformation[1]<<8)+imformation[0];
		switch(order)
		{
		case SI_BREATHSOUND://呼吸音
			cmd[0]=imformation[1];
			cmd[1]=imformation[2];
			g_l_lung_sound_set[0]=cmd[0];
			g_l_lung_sound_set[1]=cmd[1];
			//cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_L_BREATH_SOUND,cmd);
			cmd[0]=imformation[0];
			cmd[1]=imformation[2];
			g_r_lung_sound_set[0]=cmd[0];
			g_r_lung_sound_set[1]=cmd[1];
			//cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,CHEST_CAN_ID,2,CAN_R_BREATH_SOUND,cmd);
			break;
		case SI_ARMTIC://ÊÖ±Û³é´¤£¬·Åµ½Ö÷°åÅÐ¶ÏÔÙ·¢ËÍ
			cmd[0]=imformation[0];
			cmd[1]=imformation[1];
			cmd[2]=imformation[2];
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,LEFT_ARM_CAN_ID,3,CAN_LEFT_ARMTIC,cmd);
			cmd4can_push_to_mail(TxMessage,&cmd2can_mailbox,RIGHT_ARM_CAN_ID,3,CAN_RIGHT_ARMTIC,cmd);
			break;
		case SI_PULSE_POWER://¶¯ÂöÇ¿¶È
			g_pulse_level[0]=imformation[0];
			g_pulse_level[1]=imformation[1];
			g_pulse_level[2]=imformation[2];
			rt_sem_release(&pulse_ctrl_sem);//release a semaphore after update
			break;
		default:break;
		}
		break;
	default:break;
	}

}
void can_data_push(CMD_DATA_STRUCT *data_back2send,CanRxMsg can_receive_buff)
{
	data_back2send->can_id=can_receive_buff.StdId;
}

uint8_t g_pneu_puncture_state=0,g_hemo_puncture_state=0;
uint8_t g_pneu_press=0;
uint16_t g_l_tv=0,g_r_tv=0;
//将从can收到的数据推到rs232发送线程
void can_data_update(CanRxMsg can_receive_buff)
{
	CMD_DATA_STRUCT *data_back2send;
	uint8_t data_back[5]={0};//
	
	//can_data_push(data_back2send,can_receive_buff);

	switch(can_receive_buff.Data[0])
	{
		case CAN_AIRWAY:
			data_back[0]=can_receive_buff.Data[1];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_AIRWAY,data_back,1);
			break;

		case CAN_PUPILREFLECT:
			data_back[0]=((can_receive_buff.Data[2]&0x01)<<4)+(can_receive_buff.Data[1]&0x01);
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PUPILREFLECT,data_back,1);
			break;

		case CAN_NECK_ANGLE:
			if(can_receive_buff.Data[2])//仰头时才更新角度
			{
				data_back[0]=can_receive_buff.Data[1];
				back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_NECK_ANGLE,data_back,1);
			}
			break;

		case CAN_CAROTID_TOUCH:
			/****按压力度代码开始****/
			data_back[0]=can_receive_buff.Data[1];
			data_back[1]=can_receive_buff.Data[2];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_NECK_ADC,data_back,2);
			/****按压力度代码结束****/
			/******2016年4月份更新为压力传感器方案，以下代码注销******
			if((can_receive_buff.Data[2]&0x01)==1)
			{
				g_pulse1_state|=(can_receive_buff.Data[2]&0x01)<<1;
			}
			else{
				g_pulse1_state&=0xFD;
			}
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_pulse1_state|=can_receive_buff.Data[1]&0x01;
			}
			else{
				g_pulse1_state&=0xFE;
			}
			data_back[0]=g_pulse1_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PULSE1,data_back,1);
			*****2016年4月份更新为压力传感器方案，以上代码注销******/
			break;

		case CAN_LEFT_ARM_HEMOSTASIS:
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_arm_hemostasis_state|=can_receive_buff.Data[1]&0x01;
			}
			else{
				g_arm_hemostasis_state&=0xFE;
			}
			data_back[0]=g_arm_hemostasis_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_ARM_HEMOSTASIS,data_back,1);
			break;

		case CAN_LEFT_ARM_HEMO_ADC:
			g_l_arm_hemo_adc_state=can_receive_buff.Data[1];
			data_back[1]=g_l_arm_hemo_adc_state;
			data_back[0]=g_r_arm_hemo_adc_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_ARM_ADC,data_back,2);
			break;

		case CAN_LEFT_ARM_ARTERY_TOUCH:
			if((can_receive_buff.Data[2]&0x01)==1)
			{
				g_pulse1_state|=(can_receive_buff.Data[2]&0x01)<<4;
			}
			else{
				g_pulse1_state&=0xEF;
			}
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_pulse1_state|=(can_receive_buff.Data[1]&0x01)<<2;
			}
			else{
				g_pulse1_state&=0xFB;
			}
			data_back[0]=g_pulse1_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PULSE1,data_back,1);
			break;

		case CAN_LEFT_ARM_VALVES_STATES:
			g_left_arm_valve_state=(can_receive_buff.Data[3]<<6)+(can_receive_buff.Data[2]<<4)+(can_receive_buff.Data[1]<<2);
			data_back[0]=g_right_arm_valve_state;
			data_back[1]=g_left_arm_valve_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_ARM_VALVE,data_back,2);
			break;
		case CAN_RIGHT_ARM_HEMOSTASIS:
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_arm_hemostasis_state|=(can_receive_buff.Data[1]&0x01)<<4;
			}
			else{
				g_arm_hemostasis_state|=0xEF;
			}
			data_back[0]=g_arm_hemostasis_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_ARM_HEMOSTASIS,data_back,1);
			break;
		case CAN_RIGHT_ARM_HEMO_ADC:
			g_r_arm_hemo_adc_state=can_receive_buff.Data[1];
			data_back[1]=g_l_arm_hemo_adc_state;
			data_back[0]=g_r_arm_hemo_adc_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_ARM_ADC,data_back,2);
			break;
		case CAN_RIGHT_ARM_ARTERY_TOUCH:
			if((can_receive_buff.Data[2]&0x01)==1)
			{
				g_pulse1_state|=(can_receive_buff.Data[2]&0x01)<<5;
			}
			else{
				g_pulse1_state&=0xDF;
			}
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_pulse1_state|=(can_receive_buff.Data[1]&0x01)<<3;
			}
			else{
				g_pulse1_state&=0xF7;
			}
			data_back[0]=g_pulse1_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PULSE1,data_back,1);
			break;
		case CAN_RIGHT_ARM_VALVES_STATES:
			g_right_arm_valve_state=(can_receive_buff.Data[3]<<6)+(can_receive_buff.Data[2]<<4)+(can_receive_buff.Data[1]<<2);
			data_back[0]=g_right_arm_valve_state;//右侧是靠前字节
			data_back[1]=g_left_arm_valve_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_ARM_VALVE,data_back,2);
			break;
		case CAN_LEFT_LEG_HEMOSTASIS:
			g_leg_hemostasis_state|=can_receive_buff.Data[1]&0x01;
			data_back[0]=g_leg_hemostasis_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_LEG_HEMOSTASIS,data_back,1);
			break;
		case CAN_LEFT_LEG_HEMO_ADC:
			g_l_leg_hemo_adc_state=can_receive_buff.Data[1];
			data_back[1]=g_l_leg_hemo_adc_state;
			data_back[0]=g_r_leg_hemo_adc_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_LEG_ADC,data_back,2);
			break;
		case CAN_LEFT_LEG_ARTERY_TOUCH:
			if((can_receive_buff.Data[2]&0x01)==1)
			{
				g_pulse2_state|=(can_receive_buff.Data[2]&0x01)<<4;
			}
			else{
				g_pulse2_state&=0xEF;
			}
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_pulse2_state|=(can_receive_buff.Data[1]&0x01)<<2;
			}
			else{
				g_pulse2_state&=0xFB;
			}
			data_back[0]=g_pulse2_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PULSE2,data_back,1);
			break;
		case CAN_LEFT_LEG_VALVES_STATES:
			g_left_leg_valve_state=(can_receive_buff.Data[3]<<6)+(can_receive_buff.Data[2]<<4)+(can_receive_buff.Data[1]<<2);
			data_back[0]=g_right_leg_valve_state;
			data_back[1]=g_left_leg_valve_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_LEG_VALVE,data_back,2);
			break;

		case CAN_RIGHT_LEG_HEMOSTASIS:
			g_leg_hemostasis_state|=(can_receive_buff.Data[1]&0x01)<<4;
			data_back[0]=g_leg_hemostasis_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_LEG_HEMOSTASIS,data_back,1);
			break;
		case CAN_RIGHT_LEG_AIR_PRESS:
			data_back[0]=can_receive_buff.Data[1];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PRS_CYLINDER,data_back,1);
			break;
		case CAN_RIGHT_LEG_HEMO_ADC:
			g_r_leg_hemo_adc_state=can_receive_buff.Data[1];
			data_back[1]=g_l_leg_hemo_adc_state;
			data_back[0]=g_r_leg_hemo_adc_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_LEG_ADC,data_back,2);
			break;
		case CAN_RIGHT_LEG_ARTERY_TOUCH:
			if((can_receive_buff.Data[2]&0x01)==1)
			{
				g_pulse2_state|=((can_receive_buff.Data[2]&0x01)<<5);
			}
			else{
				g_pulse2_state&=0xDF;
			}
			if((can_receive_buff.Data[1]&0x01)==1)
			{
				g_pulse2_state|=((can_receive_buff.Data[1]&0x01)<<3);
			}
			else{
				g_pulse2_state&=0xF7;
			}
			data_back[0]=g_pulse2_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PULSE2,data_back,1);
			break;
		case CAN_RIGHT_LEG_VALVES_STATES:
			g_right_leg_valve_state=(can_receive_buff.Data[3]<<6)+(can_receive_buff.Data[2]<<4)+(can_receive_buff.Data[1]<<2);
			data_back[0]=g_right_leg_valve_state;
			data_back[1]=g_left_leg_valve_state;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_LEG_VALVE,data_back,2);
			break;

		case CAN_POWER_LEVEL:
			data_back[0]=can_receive_buff.Data[1];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_BATT_LEVEL,data_back,1);
			break;
		case CAN_CHARGING_ON:
			data_back[0]=can_receive_buff.Data[1];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_CHARGING,data_back,1);
			break;

		case CAN_PNEU_PUNCTURE://气胸穿刺操作
			data_back[0]=can_receive_buff.Data[1];
			g_pneu_puncture_state=data_back[0];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PNEU_PUNCTURE,data_back,1);
			break;
		case CAN_HEMO_PUNCTURE://血胸穿刺操作
			data_back[0]=can_receive_buff.Data[1];
			g_hemo_puncture_state=data_back[0];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_HEMO_PUNCTURE,data_back,1);
			break;
		case CAN_LEFT_TV:
			data_back[0]=can_receive_buff.Data[2];//低位
			data_back[1]=can_receive_buff.Data[1];//高位
			g_l_tv=(data_back[1]<<8)+data_back[0];
			g_l_tv/=5;
			data_back[0]=g_l_tv%256;
			data_back[1]=g_l_tv/256;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_TV,data_back,2);
			break;
		case CAN_CPR_POSITION:
			data_back[0]=can_receive_buff.Data[1];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_CPR_POSITION,data_back,1);
			break;
		case CAN_PRS_PNEUMOTHORAX://气胸压力
			data_back[0]=can_receive_buff.Data[1];
			g_pneu_press=data_back[0];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_PRS_PNEUMOTHORAX,data_back,1);
			break;
		case CAN_RATE_HEMOTHORAX:
			data_back[0]=can_receive_buff.Data[1];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_RATE_HEMOTHORAX,data_back,1);
			break;
		case CAN_RIGHT_TV:
			data_back[0]=can_receive_buff.Data[2];//低位
			data_back[1]=can_receive_buff.Data[1];//高位
			g_r_tv=(data_back[1]<<8)+data_back[0];
			g_r_tv/=5;
			data_back[0]=g_r_tv%256;
			data_back[1]=g_r_tv/256;
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_VENTURI,data_back,2);
			break;
		case CAN_CPR_ADC:
			data_back[0]=can_receive_buff.Data[1];//要什么循环？？！
			data_back[1]=can_receive_buff.Data[2];
			data_back[2]=can_receive_buff.Data[3];
			data_back[3]=can_receive_buff.Data[4];
			data_back[4]=can_receive_buff.Data[5];
			back_data_push_to_mail(data_back2send,&mb_for_data_sendback,SI_CPR_ADC,data_back,5);
			break;
	}
}



/**
 *	@name		data_update
 *	@brief		´ÓÊý¾Ý½»»»ÖÐÐÄÊÕµÄÊý¾Ý£¬½âÎö
 *	@param		None
 *	@retval		None
 */
void data_update(uint8_t structure_name[],
	uint8_t imformation[],uint8_t imformation_length)
{
	uint8_t content_8=0;
	uint16_t order=0;
	order=structure_name[1]*256+structure_name[0];
	content_8=imformation[0];
	switch(order)
	{

	}
}


/**
 *	@name		protocol_thread_entry
 *	@brief		The Thread of Protocol 
 *	@param		*parameter
 *	@retval		None
 */
extern uint8_t rs485_init_flag;
extern struct rt_mailbox mb_for_cmd_analysis;
ALIGN(RT_ALIGN_SIZE)
static char protocol_thread_stack[1024];
struct rt_thread protocol_thread;
static void protocol_thread_entry(void *parameter)
{
	rt_time_t	next_delay;
	CMD_DATA *cmd_received_to_analysis;
	next_delay=100;
	rt_thread_delay(next_delay); //delay for init
	
	//ÓÉÔ­À´¼ì²âÃüÁî±ä»¯ÔÙ·¢ËÍ¸ÄÎª°ëÃë¼ä¸ô·¢ËÍÁ½¸ö°åµÄ485ÃüÁî
	while(1)
	{
		if (rt_mb_recv(&mb_for_cmd_analysis,(rt_uint32_t *)&cmd_received_to_analysis, 1)
			== RT_EOK)
		{
			//rt_kprintf("temp_test=%d\r\n",temp_test);
			cmd_update((*cmd_received_to_analysis).structure_id,\
						(*cmd_received_to_analysis).cmd_data_value,\
						(*cmd_received_to_analysis).cmd_length);
			if(cmd_received_to_analysis!=RT_NULL)
			{
				rt_free(cmd_received_to_analysis);
			}
		}
		//next_delay=1;//500ms update
		//rt_thread_delay(next_delay);
	}
}

/**
 *	@name		cmb_hw_adc_init
 *	@brief	Init The ADC Thread
 *	@param	None
 *	@retval	None
 */
int cmb_hw_protocol_init(void)
{ 
	//CAN发送缓存邮箱
	rt_mb_init(&cmd2can_mailbox,
	"cmd2can", 
	&cmd2can_mailbox_pool[0], 
	sizeof(cmd2can_mailbox_pool)/4, 
	RT_IPC_FLAG_FIFO);


	rt_thread_init(&protocol_thread,
		"protocol",
		protocol_thread_entry,
		RT_NULL,
		&protocol_thread_stack[0],
		sizeof(protocol_thread_stack),5,10);
	rt_thread_startup(&protocol_thread);

	return 0;
}

/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
