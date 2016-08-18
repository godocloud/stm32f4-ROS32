/**
	********************************************************************
	*	@file			cpr.c
	*	@author			Jun.Lu
	*	@brief
	********************************************************************
	*	@attention
	* 	cpr thread
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author						Notes
	*	2016-2-11			Jun.Lu						first implement
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include <board.h>
#include "protocol.h"
#include "can_driver.h"
#include "cpr.h"
#include <rtdevice.h>
#include "vl6180x_def.h"

/*	Global variables -----------------------------------------------*/
extern struct rt_mailbox mb_for_data_sendback;
extern void back_data_push_to_mail(CMD_DATA_STRUCT *sendback_struct, 
									rt_mailbox_t sendback_buff_mailbox, 
									uint8_t sendback_structure_id, 
									uint8_t back_data[], uint8_t dlc);
extern uint8_t back_data_8bit_noid(uint8_t structure_id,uint16_t content);
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/

//#define CPR_DEBUG_ON
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
struct IIC_device cpr_device;   /*CPR设备结构体*/

/*向BH1772发送参数*/
static rt_uint8_t CprDeviceSend(rt_uint16_t s_reg,rt_uint8_t s_data)
{
    struct rt_i2c_msg msg;
    rt_uint8_t send_buffer[3];

    RT_ASSERT(cpr_device.i2c_device != RT_NULL);

    send_buffer[0] = (rt_uint8_t)(s_reg>>8);
    send_buffer[1] = (rt_uint8_t)(s_reg);
	send_buffer[2] = (rt_uint8_t)(s_data); 
    
	msg.addr = theVL6180xDev;
    msg.flags = RT_I2C_WR;
    msg.len = 3;
    msg.buf = send_buffer;
    if(rt_i2c_transfer(cpr_device.i2c_device, &msg, 1) != 1)
		 return 1;        /*操作失败*/
	else
		 return 0;        /*操作成功*/
}

static rt_err_t CprDeviceInit(rt_device_t dev)
{  
// 	 cpr_device_send((rt_uint16_t)(PS_CONTROL<<9)|PS_Stand_alone_mode);
// 	 cpr_device_send((rt_uint16_t)(I_LED<<9)|I_LED_50MA);
// 	 cpr_device_send((rt_uint16_t)(PS_MEAS_RATE<<9)|PS_meas_rate_10ms);
}



static rt_err_t CprDeviceOpen(rt_device_t dev, rt_uint16_t oflag)
{  
    
	 return RT_EOK;
}

static rt_err_t CprDeviceClose(rt_device_t dev)
{
}

static rt_err_t CprDeviceControl(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
	  switch (cmd)
    {
	   //case CODEC_CMD_3D:
       // eq3d(*((uint8_t*) args));
        break;

    default:
        result = RT_ERROR;
    }
    return result;
}

/*从BH1772读取数据*/
static rt_uint8_t CprDeviceRec(rt_uint16_t s_reg,rt_uint8_t*buf)
{
	 struct rt_i2c_msg msg[2];
    rt_uint8_t send_buffer[2];
    
	  RT_ASSERT(cpr_device.i2c_device != RT_NULL);
	
    send_buffer[0] = (rt_uint8_t)(s_reg>>8);
		send_buffer[1] = (rt_uint8_t)(s_reg);

	  msg[0].addr = theVL6180xDev;
    msg[0].flags = RT_I2C_WR;
    msg[0].len = 2;
    msg[0].buf = send_buffer;
	
	  msg[1].addr = theVL6180xDev;
	  msg[1].flags = RT_I2C_RD;
	  msg[1].len = 1;
	  msg[1].buf = buf;
    if(rt_i2c_transfer(cpr_device.i2c_device, msg, 2) != 2)
      return 1;            /*操作失败*/
    else
		  return 0;            /*操作成功*/

}
/*CPR设备总线硬件注册*/
rt_err_t CprDeviceHWInit(const char * i2c_bus_device_name)
{ 
	struct rt_i2c_bus_device * i2c_device;
	rt_thread_delay(100);  //上电后延时；
  i2c_device = rt_i2c_bus_device_find(i2c_bus_device_name);
  if(i2c_device == RT_NULL)
  {
     rt_kprintf("i2c bus device %s not found!\r\n", i2c_bus_device_name);
     return -RT_ENOSYS;
  }
  cpr_device.i2c_device = i2c_device;
  cpr_device.parent.type = RT_Device_Class_I2CBUS;
  cpr_device.parent.rx_indicate = RT_NULL;
  cpr_device.parent.tx_complete = RT_NULL;
  cpr_device.parent.user_data   = RT_NULL;

  cpr_device.parent.control = CprDeviceControl;
  cpr_device.parent.init    = CprDeviceInit;
  cpr_device.parent.open    = CprDeviceOpen;
  cpr_device.parent.close   = CprDeviceClose;
  cpr_device.parent.read    = RT_NULL;
  cpr_device.parent.write   = RT_NULL;
	/* register the device */
  return rt_device_register(&cpr_device.parent, "cpr_device", RT_DEVICE_FLAG_RDWR);
}

void VL6180x_RangeStaticInit(void)
{
    /* REGISTER_TUNING_SR03_270514_CustomerView.txt */
    CprDeviceSend( 0x0207, 0x01);
    CprDeviceSend( 0x0208, 0x01);
    CprDeviceSend( 0x0096, 0x00);
    CprDeviceSend( 0x0097, 0xfd);
    CprDeviceSend( 0x00e3, 0x00);
    CprDeviceSend( 0x00e4, 0x04);
    CprDeviceSend( 0x00e5, 0x02);
    CprDeviceSend( 0x00e6, 0x01);
    CprDeviceSend( 0x00e7, 0x03);
    CprDeviceSend( 0x00f5, 0x02);
    CprDeviceSend( 0x00d9, 0x05);
    CprDeviceSend( 0x00db, 0xce);
    CprDeviceSend( 0x00dc, 0x03);
    CprDeviceSend( 0x00dd, 0xf8);
    CprDeviceSend( 0x009f, 0x00);
    CprDeviceSend( 0x00a3, 0x3c);
    CprDeviceSend( 0x00b7, 0x00);
    CprDeviceSend( 0x00bb, 0x3c);
    CprDeviceSend( 0x00b2, 0x09);
    CprDeviceSend( 0x00ca, 0x09);
    CprDeviceSend( 0x0198, 0x01);
    CprDeviceSend( 0x01b0, 0x17);
    CprDeviceSend( 0x01ad, 0x00);
    CprDeviceSend( 0x00ff, 0x05);
    CprDeviceSend( 0x0100, 0x05);
    CprDeviceSend( 0x0199, 0x05);
    CprDeviceSend( 0x01a6, 0x1b);
    CprDeviceSend( 0x01ac, 0x3e);
    CprDeviceSend( 0x01a7, 0x1f);
    CprDeviceSend( 0x0030, 0x00);

    /* Recommended : Public registers - See data sheet for more detail */
    CprDeviceSend( 0x0011, 0x10); /* Enables polling for New Sample ready when measurement completes */
    CprDeviceSend( 0x010a, 0x30); /* Set the averaging sample period (compromise between lower noise and increased execution time) */
    CprDeviceSend( 0x003f, 0x46); /* Sets the light and dark gain (upper nibble). Dark gain should not be changed.*/
    CprDeviceSend( 0x0031, 0xFF); /* sets the # of range measurements after which auto calibration of system is performed */
    CprDeviceSend( 0x0040, 0x63); /* Set ALS integration time to 100ms */
    CprDeviceSend( 0x002e, 0x01); /* perform a single temperature calibration of the ranging sensor */

    /* Optional: Public registers - See data sheet for more detail */
    CprDeviceSend( 0x001b, 0x00); /* Set default ranging inter-measurement period to 100ms */
    CprDeviceSend( 0x003e, 0x31); /* Set default ALS inter-measurement period to 500ms */
    CprDeviceSend( 0x0014, 0x24); /* Configures interrupt on New sample ready */


}

int VL6180x_InitData(void)         //设备初始化
{
	int status, dmax_status ;
	int8_t offset;
	uint8_t FreshOutReset;


	do {

		/* backup offset initial value from nvm these must be done prior any over call that use offset */
		status = CprDeviceRec(SYSRANGE_PART_TO_PART_RANGE_OFFSET, (uint8_t *)&offset);
		if (status) {
			rt_kprintf("SYSRANGE_PART_TO_PART_RANGE_OFFSET rd fail");
			break;
		}


		/* Read or wait for fresh out of reset  */
		status = CprDeviceRec(SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
		if (status) {
			rt_kprintf("SYSTEM_FRESH_OUT_OF_RESET rd fail");
			break;
		}
		if (FreshOutReset != 1 || dmax_status)
			status = CALIBRATION_WARNING;

	} while (0);

	return status;
}

int VL6180x_StaticInit(void)
{

	VL6180x_RangeStaticInit();
	return 0;
	
}

int VL6180x_RangeSetRawThresholds(uint8_t low, uint8_t high)   //设置Range 原始测量值的高低阈值，范围均为0-255
{
	int status;
	/* TODO we can optimize here grouping high/low in a word but that's cpu endianness dependent */
	status = CprDeviceSend(SYSRANGE_THRESH_HIGH, high);
	if (!status) {
		status = CprDeviceSend(SYSRANGE_THRESH_LOW, low);
	}

	return status;
}

int VL6180x_ClearInterrupt(uint8_t IntClear)  //清除中断标志
{
	int status;
	if (IntClear <= 7) {
		status = CprDeviceSend(SYSTEM_INTERRUPT_CLEAR, IntClear);
	} else {
		status = INVALID_PARAMS;
	}
	return status;
}

int VL6180x_Prepare(void)    //开启GPIO中断，设置Rang模式的阈值和ALS模式的测量周期、增益、阈值等
{
	int status;

	do {
		status = VL6180x_StaticInit();
		if (status < 0)
			break;


		/* set default threshold */
		status = VL6180x_RangeSetRawThresholds(10, 200);
		if (status) {
			rt_kprintf("VL6180x_RangeSetRawThresholds fail");
			break;
		}
		/* make sure to reset any left previous condition that can hangs first poll */
		status = VL6180x_ClearInterrupt(INTERRUPT_CLEAR_ERROR|INTERRUPT_CLEAR_RANGING|INTERRUPT_CLEAR_ALS);
	} while (0);

	return status;
}

int VL6180x_WaitDeviceBooted(void)    //等待设备重启
{
	uint8_t FreshOutReset;
	int status;
	do {
		status = CprDeviceRec(SYSTEM_FRESH_OUT_OF_RESET, &FreshOutReset);
	} while (FreshOutReset != 1 && status == 0);

	return status;
}

int VL6180x_RangeSetSystemMode(uint8_t  mode)   //设置Range模式下的系统模式
{
	int status;
	/* FIXME we are not checking device is ready via @a VL6180x_RangeWaitDeviceReady
	 * so if called back to back real fast we are not checking
	 * if previous mode "set" got absorbed => bit 0 must be 0 so that it work
	 */
	if (mode <= 3) {
		status = CprDeviceSend(SYSRANGE_START, mode);
		if (status) {
		    rt_kprintf("SYSRANGE_START wr fail");
		}
	} else {
		status = INVALID_PARAMS;
	}
	return status;
}

rt_uint8_t VL6180x_Poll_Range(void) 
{
		rt_uint8_t status,i=0;
		rt_uint8_t range_status;
		// check the status
		range_status=CprDeviceRec(RESULT_INTERRUPT_STATUS_GPIO,&status);
		range_status = status & 0x07;
		// wait for new measurement ready status
		while ((range_status != 0x04)&&i++<200) 
		{
				rt_thread_delay(1);
				CprDeviceRec(RESULT_INTERRUPT_STATUS_GPIO,&status);
				range_status = status & 0x07;//这里过不去
		}
		return 0;
}

rt_uint8_t VL6180x_Read_Range(void) 
{
	 rt_uint8_t range;
	 CprDeviceRec(RESULT_RANGE_VAL,&range);
	 return range;
}

int VL6180x_RangePollMeasurement(void)    //设置系统模式为单次测距，并进行测量
{
	int status;


	/* if device get stopped with left interrupt uncleared , it is required to clear them now or poll for new condition will never occur*/
	status = VL6180x_ClearInterrupt(INTERRUPT_CLEAR_ERROR|INTERRUPT_CLEAR_RANGING|INTERRUPT_CLEAR_ALS);
	if (status) {
		rt_kprintf("VL6180x_RangeClearInterrupt fail");
		return 0;
	}

	/* //![single_shot_snipet] */
	status = VL6180x_RangeSetSystemMode(MODE_START_STOP | MODE_SINGLESHOT);   //设置为单次测量
	if (status) {
		rt_kprintf("VL6180x_RangeSetSystemMode fail");
		return 0;
	}


	/* poll for new sample ready */
//	while (1) {
//		status = VL6180x_RangeGetInterruptStatus(&IntStatus.val);
//		if (status) {
//			break;
//		}
//		if (IntStatus.status.Range == RES_INT_STAT_GPIO_NEW_SAMPLE_READY || IntStatus.status.Error != 0) {
//			break;
//		}

//		rt_thread_delay(10);
//	}
	VL6180x_Poll_Range();
	/* //![single_shot_snipet] */

	status=VL6180x_Read_Range();
}

extern uint8_t g_distance_val;
ALIGN(RT_ALIGN_SIZE)
static char cpr_thread_stack[1024];
struct rt_thread cpr_thread;
static void cpr_thread_entry(void *param)
{
	uint8_t distance_now=0,distance_delta=0,distance_val_init=0,init_flag=0,i;
	uint16_t distance_sum=0;
	uint8_t cpr_send_flag=0,cpr_send_count=0;
	rt_device_t cpr_device;
	
	cpr_device = rt_device_find("cpr_device");
	if (cpr_device != RT_NULL)
	{
		rt_device_open(cpr_device, RT_DEVICE_FLAG_RDWR);
	}
	
	VL6180x_InitData();
	VL6180x_Prepare();
    while(1)
    {
		distance_now = VL6180x_RangePollMeasurement();

		//if(distance_now>70)distance_now=70;
#ifdef CPR_DEBUG_ON
		rt_kprintf("distance_now=%d\r\n",distance_now);
#endif
		if(init_flag==0)
		{
			for(i=0;i<10;i++)
			{
				distance_sum+=distance_now;
			}
			distance_val_init=distance_sum/10;
			init_flag=1;
#ifdef CPR_DEBUG_ON
			rt_kprintf("distance_init=%d\r\n",distance_val_init);
#endif
			
		}
		else{
			if(distance_now<distance_val_init)
			{
				if((distance_val_init-distance_now)>=15)//有效按压
				{
					distance_delta=distance_val_init-distance_now;
					
					cpr_send_flag=1;
				}
				else{
					cpr_send_flag=2;
				}
			}
			else{
				cpr_send_flag=2;
			}
			
			
			if(cpr_send_flag==1)
			{		
				if(distance_delta>70)distance_delta=70;
				distance_now=70-distance_delta;//反一下
				back_data_8bit_noid(SI_CPR_HEIGHT,distance_now);
				cpr_send_count=0;
#ifdef CPR_DEBUG_ON
					rt_kprintf("distance_send=%d\r\n",distance_now);
#endif
			}
			else if(cpr_send_flag==2)
			{
				distance_now=70;
				if(cpr_send_count<100)
				{
					back_data_8bit_noid(SI_CPR_HEIGHT,distance_now);
					cpr_send_count++;
				}
			}
		}
		
		rt_thread_delay(3);
    }
}


/**
	*	@name		cmb_hw_cpr_init
	*	@brief		Init the valves thread
	*	@param		None
	*	@retval		None
	*/

int cmb_cpr_init(uint8_t prio)
{						
	//can send thread init
    rt_thread_init(&cpr_thread,"cpr",
                   cpr_thread_entry,RT_NULL,
                   &cpr_thread_stack[0],
                   sizeof(cpr_thread_stack),prio,10);
    rt_thread_startup(&cpr_thread);

    return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
