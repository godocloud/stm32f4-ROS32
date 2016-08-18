/**
	********************************************************************
	*	@file		nrf24l01.c
	*	@author		Jun.Lu
	*	@version	alpha
	*	@date		28-Oct-2013
	*	@brief		
	********************************************************************
	*	@attention
	*	The driver of NRF24L01+ module which is used to connect sim 
	*	thermometer.
	*-----------------------------
	*
	*	Change Logs:
	*	Date				Author					Notes
	*	2013-10-28			Jun.Lu					Version alpha
	*
	********************************************************************
	*/

/*	Includes -------------------------------------------------------*/
#include <rtthread.h>
#include "nrf24l01.h"
#include "stm32f4xx.h"
/*	Global variables -----------------------------------------------*/
uint8_t RX_BUF[RX_PLOAD_WIDTH];//receive buffer
uint8_t TX_BUF[TX_PLOAD_WIDTH];//send buffer
uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0x20,0x16,0x02,0x15,0x01};//{0x65,0xff,0xc6,0x60,0x94};
uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0x20,0x16,0x02,0x15,0x01};
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/
/**
	*	@name		Delay
	*	@brief		hold a short time delay
	*	@param		n
	*	@retval		None
	*/
static void Delay(__IO uint16_t n)
{
	rt_time_t	next_delay;
	next_delay=n;//n*10ms
	rt_thread_delay(next_delay); 
}
//********************第一层封装开始************************
/**
	*	@name		GPIO_Configuration
	*	@brief		SPI GPIO Configuration
	*	@param		None
	*	@retval		None
	*/
static void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	//Clock enable 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIO_CE_IRQ|RCC_AHB1Periph_GPIO_SPI,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APBPeriph_SPI,ENABLE);
	
	
	//Configure SPI pins:SCK,MOSI,MISO
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_SPI_SCK | GPIO_Pin_SPI_MOSI | GPIO_Pin_SPI_MISO;
    GPIO_Init(GPIO_SPI, &GPIO_InitStructure);
	//Connect SPI pins to AF
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_SCK_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MOSI_SOURCE,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIO_SPI, GPIO_Pin_SPI_MISO_SOURCE,GPIO_AF_SPI1);
	//CS and CE gpio set
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CE;
    GPIO_Init(GPIO_CE, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_CS;
    GPIO_Init(GPIO_CS, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_IRQ;
    GPIO_Init(GPIO_IRQ, &GPIO_InitStructure);
	
	//SPI configuration
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 84000kHz/8=9MHz模式
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI1, &SPI_InitStructure);

	//SPI_CalculateCRC(SPI1, DISABLE);

	/* SPI1 enable */
	SPI_Cmd(SPI, ENABLE);
	NRF_CE_LOW();
	NRF_CSN_HIGH();
}

/**
	*	@name		SPI_NRF_RW
	*	@brief		向NRF读或写一个字节数据
	*	@param		dat
	*	@retval		None
	*/
static uint8_t SPI_NRF_RW(uint8_t dat)
{  	
	/* 当 SPI发送缓冲器非空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
  
	/* 通过 SPI2发送一字节数据 */
	SPI_I2S_SendData(SPI, dat);		
 
	/* 当SPI接收缓冲器为空时等待 */
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI);
}
//********************第一层封装结束************************	

//********************第二层封装开始************************
/**
	*	@name		SPI_NRF_WriteReg
	*	@brief		把数据写入特定寄存器
	*	@param		reg：命令及寄存器地址
	*				dat：写入寄存器的数据
	*	@retval		status：返回status寄存器的状态
	*/
static uint8_t SPI_NRF_WriteReg(uint8_t reg,uint8_t dat)
{
	uint8_t status;
	NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
    NRF_CSN_LOW();	
	/*发送命令及寄存器号 */
	status = SPI_NRF_RW(reg); 
	 /*向寄存器写入数据*/
    SPI_NRF_RW(dat);          
	/*CSN拉高，完成*/	   
  	NRF_CSN_HIGH();		
	/*返回状态寄存器的值*/
   	return(status);
}

/**
	*	@name		SPI_NRF_ReadReg
	*	@brief		读特定寄存器的数据
	*	@param		reg：命令及寄存器地址
	*	@retval		reg_val:寄存器中的数据
	*/
static uint8_t SPI_NRF_ReadReg(uint8_t reg)
{
 	uint8_t reg_val;

	NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
 	NRF_CSN_LOW();			
  	 /*发送寄存器号*/
	SPI_NRF_RW(reg); 
	 /*读取寄存器的值 */
	reg_val = SPI_NRF_RW(NOP);	            
   	/*CSN拉高，完成*/
	NRF_CSN_HIGH();		
   	
	return reg_val;
}	

/**
	*	@name		SPI_NRF_WriteBuf
	*	@brief		往特定寄存器中写入一串数据
	*	@param		reg：命令及寄存器地址
	*				*pBuf：存储着将要写入寄存器数据的存储数组
					bytes：pBuf的长度
	*	@retval		status:status寄存器的状态
	*/
static uint8_t SPI_NRF_WriteBuf(uint8_t reg ,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_cnt;
	NRF_CE_LOW();
   	 /*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();			
	 /*发送寄存器号*/	
	status = SPI_NRF_RW(reg); 
  	  /*向缓冲区写入数据*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
	{
		SPI_NRF_RW(*pBuf++);	//写数据到缓冲区 	 
	}
	/*CSN拉高，完成*/
	NRF_CSN_HIGH();			
  
  	return (status);	//返回NRF24L01的状态 		
}

/**
	*	@name		SPI_NRF_ReadBuf
	*	@brief		读特定寄存器的一串数据
	*	@param		reg：命令及寄存器地址
	*				*pBuf：读出数据的存储数组
					bytes：pBuf的长度
	*	@retval		status:status寄存器的状态
	*/
static uint8_t SPI_NRF_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
 	uint8_t status, byte_cnt;
	NRF_CE_LOW();
	/*置低CSN，使能SPI传输*/
	NRF_CSN_LOW();	
	/*发送寄存器号*/		
	status = SPI_NRF_RW(reg); 
 	/*读取缓冲区数据*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
	{
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //从NRF24L01读取数据  
	}
	 /*CSN拉高，完成*/
	NRF_CSN_HIGH();	
		
 	return status;		//返回寄存器状态值
}
//********************第二层封装结束************************

//********************第三层封装开始************************
/**
	*	@name		NRF_TX_Mode
	*	@brief		配置为发送模式
	*	@param		None
	*	@retval		None
	*/
static void NRF_TX_Mode(uint8_t*COMM_ADDRESS)
{  
	NRF_CE_LOW();		

	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 

	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK   

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  

	SPI_NRF_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANNEL);       //设置RF通道为CHANAL

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0F);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   

	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,发射模式,开启所有中断

	/*CE拉高，进入发送模式*/	
	NRF_CE_HIGH();
	//Delay(1); //CE要拉高10ms才进入发送模式
}

/**
	*	@name		NRF_RX_Mode
	*	@brief		配置为接收送模式
	*	@param		None
	*	@retval		None
	*/
static void NRF_RX_Mode(void)

{
	NRF_CE_LOW();	

	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址    

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANNEL);      //设置RF通信频率    

	SPI_NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度      

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   

	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 

	/*CE拉高，进入接收模式*/	
	NRF_CE_HIGH();
} 

/**
	*	@name		NRF_Check
	*	@brief		配置为接收送模式
	*	@param		None
	*	@retval		None
	*/
static uint8_t NRF_Check(void)
{
	uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2};
	uint8_t buf1[5];
	uint8_t i; 
	 
	/*写入5个字节的地址.  */  
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);

	/*读出写入的地址 */
	SPI_NRF_ReadBuf(TX_ADDR,buf1,5); 
	 
	/*比较*/               
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0xC2)
		break;
	} 
	       
	if(i==5)
		return 0 ;        //MCU与NRF成功连接 
	else
		return 1 ;        //MCU与NRF不正常连接
}

/**
	*	@name		NRF_Tx_Dat
	*	@brief		向NRF发送缓冲区写入数据
	*	@param		txbuf：存储着将发送数据的数组
	*	@retval		成功或失败
	*/
static uint8_t NRF_Tx_Dat(uint8_t *txbuf)
{
	uint8_t state;  
//	uint16_t timeout_counter=0xFFFF;
	/*ce为低，进入待机模式1*/
	NRF_CE_LOW();
	/*写数据到TX BUF 最大 32个字节*/						
	SPI_NRF_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	/*CE为高，txbuf非空，发送数据包 */   
	NRF_CE_HIGH();
	/*等待发送完成中断 */                            
	while((NRF_Read_IRQ()!=0));//&&timeout_counter--); 	
	/*读取状态寄存器的值 */                              
	state = SPI_NRF_ReadReg(0x07);
	/*清除TX_DS或MAX_RT中断标志*/                  
	SPI_NRF_WriteReg(NRF_WRITE_REG+0x07,state); 	
	
	/*判断中断类型*/    
	if(state&MAX_RT)                     //达到最大重发次数
	{
		SPI_NRF_WriteReg(FLUSH_TX,NOP);    //清除TX FIFO寄存器 
		return MAX_RT; 
	}
	else if(state&TX_DS)                  //发送完成
		return TX_DS;
	else						  
		return ERROR;                 //其他原因发送失败
} 

/**
	*	@name		NRF_Rx_Dat
	*	@brief		从NRF接收缓冲区读出数据
	*	@param		rxbuf：存储接收数据的数组
	*	@retval		成功或失败
	*/
static uint8_t NRF_Rx_Dat(uint8_t *rxbuf)
{
	uint8_t state; 
	NRF_CE_HIGH();	 //进入接收状态
	 /*等待接收中断*/
	while(NRF_Read_IRQ()!=0); 

	NRF_CE_LOW();  	 //进入待机状态
	/*读取status寄存器的值  */               
	state=SPI_NRF_ReadReg(STATUS);
	 
	/* 清除中断标志*/      
	SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,state);

	/*判断是否接收到数据*/
	if(state&RX_DR)                                 //接收到数据
	{
		SPI_NRF_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		SPI_NRF_WriteReg(FLUSH_RX,NOP);          //清除RX FIFO寄存器
		return RX_DR; 
	}
	else    
		return ERROR;                    //没收到任何数据
}
//********************第三层封装结束************************

//**********************其他函数****************************
/**
	*	@name		Cpu_Id_Get
	*	@brief		获取cpu的id
	*	@param		None
	*	@retval		成功或失败
	*/
uint32_t Cpu_Id[3]={0};
static void Cpu_Id_Get(void)
{
    Cpu_Id[0] = *(__IO u32*)(0x1FFF7A10);
    Cpu_Id[1] = *(__IO u32*)(0x1FFF7A14);
    Cpu_Id[2] = *(__IO u32*)(0x1FFF7A18);
}	

/**
	*	@name		Dynamic_Comm_Address
	*	@brief		根据Cpu_Id生成动态通讯地址
	*	@param		None
	*	@retval		None
	*/
uint8_t Comm_Address[TX_ADR_WIDTH]={0};
static void Dynamic_Comm_Address(void)
{
	Cpu_Id_Get();
	Comm_Address[0]=(uint8_t)(Cpu_Id[0]&0x000000FF);
	Comm_Address[1]=(uint8_t)(Cpu_Id[1]&0x000000FF);
	Comm_Address[2]=(uint8_t)(Cpu_Id[2]&0x000000FF);
	Comm_Address[3]=(uint8_t)(((Cpu_Id[0]&Cpu_Id[1])&0x000FF000)>>12);
	Comm_Address[4]=(uint8_t)(((Cpu_Id[1]&Cpu_Id[2])&0x000FF000)>>12);
}
//**********************线程开始****************************
/**
	*	@name		motor_thread_entry
	*	@brief	motor thread
	*	@param	*param
	*	@retval	None
	*/
uint8_t nrf24l01_status=NORMAL;
extern uint16_t body_temperature_cmd;
uint8_t temp[3]={0x02,0x03,0x18};
extern uint8_t g_heartbeat_sync_flag,g_spo2_cmd;
extern uint16_t g_heartrate_cmd;
ALIGN(RT_ALIGN_SIZE)
static char nrf24l01_thread_stack[1024];
struct rt_thread nrf24l01_thread;
static void nrf24l01_thread_entry(void *param)
{
	rt_time_t	next_delay;
	uint8_t send_temp_char[10],status;//把温度拆成4个char发送
	GPIO_Configuration();
	Dynamic_Comm_Address();
	while(NRF_Check());//不能这么写，要加超时判断
    while(1)
    {
			if(nrf24l01_status==NORMAL&&g_heartbeat_sync_flag==1)
			{
				g_heartbeat_sync_flag=0;
				NRF_TX_Mode(TX_ADDRESS);//发送固定地址
				send_temp_char[0]=0x05;
				send_temp_char[1]=0x38;
				send_temp_char[2]=0x18;
				send_temp_char[3]=g_spo2_cmd;
				send_temp_char[4]=g_heartrate_cmd/256;
				send_temp_char[5]=g_heartrate_cmd%256;//脉搏低8位
				NRF_Tx_Dat(send_temp_char);
				//rt_kprintf("nrf24l01_status=NORMAL\r\n");
			}
			else if(nrf24l01_status==PAIRING)
			{
				NRF_TX_Mode((uint8_t*)TX_ADDRESS);//发送初始地址
				//if(NRF_Tx_Dat(send_temp_char)==TX_DS)
				if(NRF_Tx_Dat(temp)==TX_DS)//发送成功
				{
					nrf24l01_status=NORMAL;
				}
				//rt_kprintf("nrf24l01_status=PAIRING\r\n");
				status=SPI_NRF_ReadReg(0x07 );
				if((status & 0x20)==0x20)
				//rt_kprintf("SPI Send OK\r\n");
				status=SPI_NRF_ReadReg(0x17);
				//rt_kprintf(" fifo is: %d",status);
				status=SPI_NRF_ReadReg(0x06 );
				//rt_kprintf(" rf setup is: %d",status);
				status=SPI_NRF_ReadReg(0x05);
				//rt_kprintf(" rf chanel is: %d",status);
				status=SPI_NRF_ReadReg(0x00);
				//rt_kprintf(" config is: %d",status);
				SPI_NRF_WriteReg(0x20  + 0x00, 0x3B); // enable power up and prx
			}
		
		//rt_kprintf("temp=%c%c.%c%c\r\n",send_temp_char[0],send_temp_char[1],send_temp_char[2],send_temp_char[3]);
		next_delay=10;//1000ms				
        rt_thread_delay(next_delay); 
    }
}

/**
	*	@name		cmb_hw_motor_init
	*	@brief	Init the Water Pumps(including the keys )
	*	@param	None
	*	@retval	None
	*/
int cmb_hw_nrf24l01_init(uint8_t prio)
{
    //water pump thread init
    rt_thread_init(&nrf24l01_thread,"nrf24l01",
                    nrf24l01_thread_entry,RT_NULL,
                    &nrf24l01_thread_stack[0],
                    sizeof(nrf24l01_thread_stack),prio,20);
    rt_thread_startup(&nrf24l01_thread);
    
	return 0;
}




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
