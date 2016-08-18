#include "module_drv8860.h"

void DRV8860_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(SOLE_CTRL_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = EN_OUT_PIN | LATCH_OUT_PIN | CLK_OUT_PIN | DATA_OUT_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SOLE_CTRL_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = FAULT_IN_PIN | DATA_IN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SOLE_CTRL_PORT,&GPIO_InitStructure);
	
}

void send_byte(uint8_t data)
{
	uint8_t i;
	
	CLK_SET(0);
	for(i=8;i!=0;i--)
	{
		(data & 0x80)?DATA_SET(1):DATA_SET(0);
		rt_thread_delay(1);
		CLK_SET(1);
		data=(data<<1);
		rt_thread_delay(1);
		if(i>1)
		{
			CLK_SET(0);
		}
	}	
}

void DRV8860_DataWrite2(rt_uint8_t data)
{
	rt_uint8_t i;
	
	rt_enter_critical();
	
	LATCH_SET(1);
	CLK_SET(1);
	rt_thread_delay(1);
	LATCH_SET(0);
//	rt_thread_delay(1);
	send_byte(data);
	rt_thread_delay(1);
	LATCH_SET(1);
	
	rt_exit_critical();
	
}

void DRV8860_DataWrite(rt_uint8_t data)
{
	rt_uint8_t i;
	
	rt_enter_critical();
	
	CLK_SET(0);
	rt_thread_delay(1);
	DATA_SET(0);
	rt_thread_delay(1);
	LATCH_SET(0);
	rt_thread_delay(1);
	
	
	for(i=0;i<8;i++)
	{
		CLK_SET(1);
		rt_thread_delay(1);
		DATA_SET((data>>(7-i))&0x01);
		CLK_SET(0);
		rt_thread_delay(1);
	}
	
	CLK_SET(1);
	rt_thread_delay(1);
	LATCH_SET(1);
	rt_thread_delay(1);
	
	rt_exit_critical();
	
}

void DRV8860_FaultReset(void)
{
	
	rt_enter_critical();
	
	LATCH_SET(1);
	CLK_SET(0);
	rt_thread_delay(1);
	
	LATCH_SET(0);
	rt_thread_delay(1);
	
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	
	LATCH_SET(1);
	rt_thread_delay(1);
	LATCH_SET(0);
	rt_thread_delay(1);
	
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	
	LATCH_SET(1);
	rt_thread_delay(1);
	LATCH_SET(0);
	rt_thread_delay(1);
	
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	
	LATCH_SET(1);
	rt_thread_delay(1);
	LATCH_SET(0);
	rt_thread_delay(1);
	
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	CLK_SET(1);
	rt_thread_delay(1);
	CLK_SET(0);
	rt_thread_delay(1);
	
	LATCH_SET(1);
	
	rt_exit_critical();

}

rt_uint16_t DRV8860_FaultRead(void)
{
	rt_uint8_t i;
	rt_uint16_t fault=0x00;
	
	rt_enter_critical();
	
	CLK_SET(0);
	rt_thread_delay(1);
	DATA_SET(0);
	rt_thread_delay(1);
	
	LATCH_SET(1);
	rt_thread_delay(1);
	LATCH_SET(0);
	rt_thread_delay(1);
	LATCH_SET(1);
	rt_thread_delay(1);
	

	
	for(i=0;i<16;i++)
	{
		CLK_SET(1);
		rt_thread_delay(1);
		CLK_SET(0);
		rt_thread_delay(1);
		if(DATA_READ())
		{
			fault|=(0x8000>>(i+1));
		}
		
	}
	
//	CLK_SET(1);
	rt_thread_delay(1);
	
	rt_exit_critical();
	
	return fault;

}


ALIGN(RT_ALIGN_SIZE)
static rt_uint8_t drv8860_thread_stack[512];
static struct rt_thread drv8860_thread;
void drv8860_thread_entry(void* parameter)
{
	rt_uint16_t fault;
	DRV8860_GPIO_Init();
	
	EN_SET(1);
	
	DRV8860_DataWrite(0xFF);
	
	while(1)
	{
		DRV8860_DataWrite2(0xFF);
		fault=DRV8860_FaultRead();
		rt_kprintf("OPEN_Fault:0x%x\r\n",fault);
		rt_thread_delay(100);
		
		DRV8860_FaultReset();
		
		DRV8860_DataWrite2(0x00);
		fault=DRV8860_FaultRead();
		rt_kprintf("CLOSE_Fault:0x%x\r\n",fault);
		rt_thread_delay(100);
		
	}
}


int cmb_valves_init(uint8_t prio)
{						
	//can send thread init
    rt_thread_init(&drv8860_thread,"drv8860",
					drv8860_thread_entry,RT_NULL,
                   &drv8860_thread_stack[0],
                   sizeof(drv8860_thread_stack),prio,10);
    rt_thread_startup(&drv8860_thread);
	
    return 0;
}
