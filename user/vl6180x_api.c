/*******************************************************************************
Copyright © 2014, STMicroelectronics International N.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of STMicroelectronics nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED. 
IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

/*
 * $Date: 2015-01-07 16:12:48 +0100 (Wed, 07 Jan 2015) $
 * $Revision: 2023 $
 */
#include "vl6180x_api.h"
#include "usart.h"

#include "i2c_bit.h"
#include <rthw.h>
#include <rtthread.h>

/* ÐÅºÅÁ¿¿ØÖÆ¿é */
struct rt_semaphore static_sem; 

void VL6180x_ReadModelID(void)
{
	 uint8_t MODEL_ID;
	 do
	 {
	    MODEL_ID=VL6180x_ReadByte(IDENTIFICATION_MODEL_ID);
		  rt_kprintf("MODEL_ID=%x\n",MODEL_ID);
	 } 
	 while(MODEL_ID!=0xB4);
	 
}

int VL6180x_WaitDeviceBooted(void)
{
	uint8_t FreshOutReset;
	int status;

	do
	{
		FreshOutReset = VL6180x_ReadByte(SYSTEM_FRESH_OUT_OF_RESET);
	}
	while( FreshOutReset!=1 );

	return status;
}

/*Help host determine if settings have been loaded.*/
int VL6180x_WriteZeroToResetFlag(void)
{
	int status;

	status = VL6180x_WriteByte(SYSTEM_FRESH_OUT_OF_RESET,0x00);

	return status;
}

int VL6180x_RangeStaticInit(void)
{
    int status;

    /* REGISTER_TUNING_SR03_270514_CustomerView.txt */
    VL6180x_WriteByte( 0x0207, 0x01);
    VL6180x_WriteByte( 0x0208, 0x01);
    VL6180x_WriteByte( 0x0096, 0x00);
    VL6180x_WriteByte( 0x0097, 0xfd);
    VL6180x_WriteByte( 0x00e3, 0x00);
    VL6180x_WriteByte( 0x00e4, 0x04);
    VL6180x_WriteByte( 0x00e5, 0x02);
    VL6180x_WriteByte( 0x00e6, 0x01);
    VL6180x_WriteByte( 0x00e7, 0x03);
    VL6180x_WriteByte( 0x00f5, 0x02);
    VL6180x_WriteByte( 0x00d9, 0x05);
    VL6180x_WriteByte( 0x00db, 0xce);
    VL6180x_WriteByte( 0x00dc, 0x03);
    VL6180x_WriteByte( 0x00dd, 0xf8);
    VL6180x_WriteByte( 0x009f, 0x00);
    VL6180x_WriteByte( 0x00a3, 0x3c);
    VL6180x_WriteByte( 0x00b7, 0x00);
    VL6180x_WriteByte( 0x00bb, 0x3c);
    VL6180x_WriteByte( 0x00b2, 0x09);
    VL6180x_WriteByte( 0x00ca, 0x09);
    VL6180x_WriteByte( 0x0198, 0x01);
    VL6180x_WriteByte( 0x01b0, 0x17);
    VL6180x_WriteByte( 0x01ad, 0x00);
    VL6180x_WriteByte( 0x00ff, 0x05);
    VL6180x_WriteByte( 0x0100, 0x05);
    VL6180x_WriteByte( 0x0199, 0x05);
    VL6180x_WriteByte( 0x01a6, 0x1b);
    VL6180x_WriteByte( 0x01ac, 0x3e);
    VL6180x_WriteByte( 0x01a7, 0x1f);
    VL6180x_WriteByte( 0x0030, 0x00);

    /* Recommended : Public registers - See data sheet for more detail */
    VL6180x_WriteByte( 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
    VL6180x_WriteByte( 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
    VL6180x_WriteByte( 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
    VL6180x_WriteByte( 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
    VL6180x_WriteByte( 0x0040, 0x63); /* Set ALS integration time to 100ms */
    VL6180x_WriteByte( 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

    /* Optional: Public registers - See data sheet for more detail */
    VL6180x_WriteByte( 0x001b, 0x09); /* Set default ranging inter-measurement period to 100ms */
    VL6180x_WriteByte( 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
    VL6180x_WriteByte( 0x0014, 0x24); /* Configures interrupt on New sample ready */

//    status=VL6180x_RangeSetMaxConvergenceTime(dev, 50); /*  Calculate ece value on initialization (use max conv) */

    return status;
}

/*starts a single shot range measurement*/
int VL6180x_Start_Range(void)
{
	 int status;
	
	 status=VL6180x_WriteByte( SYSRANGE_START , 0x01);
	
	 return status;
}
///////////////////////////////////////////////////////////////////
// poll for new sample ready ready
///////////////////////////////////////////////////////////////////
int VL6180x_Poll_Range(void) 
{
		char status;
		char range_status;
		// check the status
		status = VL6180x_ReadByte(RESULT_INTERRUPT_STATUS_GPIO);
		range_status = status & 0x07;
		// wait for new measurement ready status
		while (range_status != 0x04) 
		{
				status = VL6180x_ReadByte(RESULT_INTERRUPT_STATUS_GPIO);
				range_status = status & 0x07;
				rt_thread_delay(10);// (can be removed)
		}
		return 0;
}

///////////////////////////////////////////////////////////////////
// Read range result (mm)
///////////////////////////////////////////////////////////////////
int VL6180x_Read_Range(void) 
{
	 int range;
	 range=VL6180x_ReadByte(RESULT_RANGE_VAL);
	 return range;
}

///////////////////////////////////////////////////////////////////
// clear interrupts
///////////////////////////////////////////////////////////////////
int VL6180x_Clear_Interrupts(void) 
{
	 char status;
   status=VL6180x_WriteByte(SYSTEM_INTERRUPT_CLEAR,0x07);
   return status;
}

unsigned int Index=0; 
void VL6180x_Save_Range_Data(int data,int *buf)
{
	 buf[Index]=data;
	 if(++Index>=10) Index=0;
}

int VL6180x_Get_Range_Average(int *buf)
{
	uint32_t data_sum=0;  
	uint16_t t,i,j,temp, average;  //temp ÁÙÊ±Êý¾Ý  average Æ½¾ùÖµ

	for(j=0;j<9;j++)   //ÅÅÐò
  {
     for(i=0;i<9-j;i++)
		 {
			   if(buf[i]>buf[i+1])
				 {
					   temp=buf[i];
					   buf[i]=buf[i+1];
					   buf[i+1]=temp;
				 }
		 }
	}		

	for(t=1;t<9;t++)  //È¥µô×îÐ¡ÖµºÍ×î´óÖµ£¬ÇóÆ½¾ù	
	{
		 data_sum+=buf[t];
	}
	average=data_sum/8;
	
	return average;
} 	 

//void  VL6180x_GPIO_Init(void)
//{
//	 GPIO_InitTypeDef  GPIO_InitStructure;
// 	
//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//Ê¹ÄÜGPIOBÊ±ÖÓ	 //Ê¹ÄÜPA,PD¶Ë¿ÚÊ±ÖÓ
//	
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;				 //GPIO0(CE) ¶Ë¿ÚÅäÖÃ 
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//ÆÕÍ¨Êä³öÄ£Ê½
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//ÍÆÍìÊä³ö
//	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 //IO¿ÚËÙ¶ÈÎª50MHz
//	 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIOA.8
//	 GPIO_ResetBits(GPIOB,GPIO_Pin_10);	

//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	    	//GPIO1(INT) ¶Ë¿ÚÅäÖÃ, ÍÆÍìÊä³ö
//	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//ÆÕÍ¨ÊäÈëÄ£Ê½
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//ÉÏÀ­
//	 GPIO_Init(GPIOB, &GPIO_InitStructure);	  				 //IO¿ÚËÙ¶ÈÎª50MHz
//}

//void VL6180x_Enable(void)
//{
//	 GPIO_SetBits(GPIOB,GPIO_Pin_10);	
//}

extern void TIM3_Int_Init(u16 arr,u16 psc);
//¹âÑ§²â¾à´«¸ÐÆ÷Ïß³Ì´¦Àíº¯ÊýÈë¿Ú
void VL6180x_SampEntry(void *parameter)
{	
	 int range,range_average;
	 int range_buf[10]; 
		
	
	IIC_Init();
	//	 VL6180x_GPIO_Init();
	//	 rt_thread_delay(10);
	//	 VL6180x_Enable();
	//	 rt_thread_delay(10);
	//	 VL6180x_WaitDeviceBooted();  //Ö»ÓÐÔÚCE½ÅÊ¹ÓÃÊ±£¬´Ë´¦²Å´ò¿ª¡£CE½ÅºÍAVDDÉÏµç¾Í±»Í¬Ê±À­¸ß£¬Ê¡ÂÔdeviceboot¡£
	
	
	
	VL6180x_ReadModelID();
	VL6180x_RangeStaticInit();
	VL6180x_WriteZeroToResetFlag();
	VL6180x_ReadModelID();
	 
	 while(1)
	 {
		// start single range measurement
		VL6180x_Start_Range();
		// poll the VL6180X till new sample ready
		VL6180x_Poll_Range();
		// read range result
		range = VL6180x_Read_Range();
		// clear the interrupt on VL6180X
//		VL6180x_Clear_Interrupts();

//		VL6180x_Save_Range_Data(range , &range_buf[0]);

//		range_average=VL6180x_Get_Range_Average(&range_buf[0]);

		rt_kprintf("range = %d\n\r\n",range);

		rt_thread_delay(5);
		  		   
	 }	
}

int cmb_vl6180_init(uint8_t prio)
{						
	rt_err_t result;
	/* ³õÊ¼»¯¾²Ì¬ÐÅºÅÁ¿£¬³õÊ¼ÖµÊÇ0 */
    result = rt_sem_init(&static_sem, "ssem", 0, RT_IPC_FLAG_FIFO);
	
	VL6180x_thread_creat(prio, 10);
	

    if (result != RT_EOK)
    {
        rt_kprintf("init static semaphore failed.\n");
        return -1;
    }
	return 0;
}
//´´½¨¹âÑ§²â¾à´«¸ÐÆ÷¿ØÖÆÏß³Ì£»
void VL6180x_thread_creat(char priority , char tick)
{ 
	rt_thread_t VL6180x_Samp_thread;				//vl6180xÏß³Ì¶¨Òå
	VL6180x_Samp_thread = rt_thread_create("vl6180x",VL6180x_SampEntry,
										 RT_NULL,1024,priority,tick);
	if(VL6180x_Samp_thread != RT_NULL)
	{
			rt_thread_startup(VL6180x_Samp_thread);
	} 
}



//´´½¨ÐÅºÅÁ¿  Í¨¹ý¶¨Ê±Æ÷3×÷ÎªÑÓÊ±º¯Êý ÊµÏÖus¼¶ÑÓÊ±
/* Ö¸ÏòÐÅºÅÁ¿µÄÖ¸Õë */
//static rt_sem_t delay_sem = RT_NULL;
















