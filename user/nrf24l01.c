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
//********************��һ���װ��ʼ************************
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
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 84000kHz/8=9MHzģʽ
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
	*	@brief		��NRF����дһ���ֽ�����
	*	@param		dat
	*	@retval		None
	*/
static uint8_t SPI_NRF_RW(uint8_t dat)
{  	
	/* �� SPI���ͻ������ǿ�ʱ�ȴ� */
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_TXE) == RESET);
  
	/* ͨ�� SPI2����һ�ֽ����� */
	SPI_I2S_SendData(SPI, dat);		
 
	/* ��SPI���ջ�����Ϊ��ʱ�ȴ� */
	while (SPI_I2S_GetFlagStatus(SPI, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI);
}
//********************��һ���װ����************************	

//********************�ڶ����װ��ʼ************************
/**
	*	@name		SPI_NRF_WriteReg
	*	@brief		������д���ض��Ĵ���
	*	@param		reg������Ĵ�����ַ
	*				dat��д��Ĵ���������
	*	@retval		status������status�Ĵ�����״̬
	*/
static uint8_t SPI_NRF_WriteReg(uint8_t reg,uint8_t dat)
{
	uint8_t status;
	NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
    NRF_CSN_LOW();	
	/*��������Ĵ����� */
	status = SPI_NRF_RW(reg); 
	 /*��Ĵ���д������*/
    SPI_NRF_RW(dat);          
	/*CSN���ߣ����*/	   
  	NRF_CSN_HIGH();		
	/*����״̬�Ĵ�����ֵ*/
   	return(status);
}

/**
	*	@name		SPI_NRF_ReadReg
	*	@brief		���ض��Ĵ���������
	*	@param		reg������Ĵ�����ַ
	*	@retval		reg_val:�Ĵ����е�����
	*/
static uint8_t SPI_NRF_ReadReg(uint8_t reg)
{
 	uint8_t reg_val;

	NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
 	NRF_CSN_LOW();			
  	 /*���ͼĴ�����*/
	SPI_NRF_RW(reg); 
	 /*��ȡ�Ĵ�����ֵ */
	reg_val = SPI_NRF_RW(NOP);	            
   	/*CSN���ߣ����*/
	NRF_CSN_HIGH();		
   	
	return reg_val;
}	

/**
	*	@name		SPI_NRF_WriteBuf
	*	@brief		���ض��Ĵ�����д��һ������
	*	@param		reg������Ĵ�����ַ
	*				*pBuf���洢�Ž�Ҫд��Ĵ������ݵĴ洢����
					bytes��pBuf�ĳ���
	*	@retval		status:status�Ĵ�����״̬
	*/
static uint8_t SPI_NRF_WriteBuf(uint8_t reg ,uint8_t *pBuf,uint8_t bytes)
{
	uint8_t status,byte_cnt;
	NRF_CE_LOW();
   	 /*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();			
	 /*���ͼĴ�����*/	
	status = SPI_NRF_RW(reg); 
  	  /*�򻺳���д������*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)
	{
		SPI_NRF_RW(*pBuf++);	//д���ݵ������� 	 
	}
	/*CSN���ߣ����*/
	NRF_CSN_HIGH();			
  
  	return (status);	//����NRF24L01��״̬ 		
}

/**
	*	@name		SPI_NRF_ReadBuf
	*	@brief		���ض��Ĵ�����һ������
	*	@param		reg������Ĵ�����ַ
	*				*pBuf���������ݵĴ洢����
					bytes��pBuf�ĳ���
	*	@retval		status:status�Ĵ�����״̬
	*/
static uint8_t SPI_NRF_ReadBuf(uint8_t reg,uint8_t *pBuf,uint8_t bytes)
{
 	uint8_t status, byte_cnt;
	NRF_CE_LOW();
	/*�õ�CSN��ʹ��SPI����*/
	NRF_CSN_LOW();	
	/*���ͼĴ�����*/		
	status = SPI_NRF_RW(reg); 
 	/*��ȡ����������*/
	for(byte_cnt=0;byte_cnt<bytes;byte_cnt++)		  
	{
		pBuf[byte_cnt] = SPI_NRF_RW(NOP); //��NRF24L01��ȡ����  
	}
	 /*CSN���ߣ����*/
	NRF_CSN_HIGH();	
		
 	return status;		//���ؼĴ���״ֵ̬
}
//********************�ڶ����װ����************************

//********************�������װ��ʼ************************
/**
	*	@name		NRF_TX_Mode
	*	@brief		����Ϊ����ģʽ
	*	@param		None
	*	@retval		None
	*/
static void NRF_TX_Mode(uint8_t*COMM_ADDRESS)
{  
	NRF_CE_LOW();		

	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 

	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK   

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  

	SPI_NRF_WriteReg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANNEL);       //����RFͨ��ΪCHANAL

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0F);  //����TX�������,0db����,2Mbps,���������濪��   

	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�

	/*CE���ߣ����뷢��ģʽ*/	
	NRF_CE_HIGH();
	//Delay(1); //CEҪ����10ms�Ž��뷢��ģʽ
}

/**
	*	@name		NRF_RX_Mode
	*	@brief		����Ϊ������ģʽ
	*	@param		None
	*	@retval		None
	*/
static void NRF_RX_Mode(void)

{
	NRF_CE_LOW();	

	SPI_NRF_WriteBuf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    

	SPI_NRF_WriteReg(NRF_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ    

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_CH,CHANNEL);      //����RFͨ��Ƶ��    

	SPI_NRF_WriteReg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��      

	SPI_NRF_WriteReg(NRF_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,���������濪��   

	SPI_NRF_WriteReg(NRF_WRITE_REG+CONFIG, 0x0f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 

	/*CE���ߣ��������ģʽ*/	
	NRF_CE_HIGH();
} 

/**
	*	@name		NRF_Check
	*	@brief		����Ϊ������ģʽ
	*	@param		None
	*	@retval		None
	*/
static uint8_t NRF_Check(void)
{
	uint8_t buf[5]={0xC2,0xC2,0xC2,0xC2,0xC2};
	uint8_t buf1[5];
	uint8_t i; 
	 
	/*д��5���ֽڵĵ�ַ.  */  
	SPI_NRF_WriteBuf(NRF_WRITE_REG+TX_ADDR,buf,5);

	/*����д��ĵ�ַ */
	SPI_NRF_ReadBuf(TX_ADDR,buf1,5); 
	 
	/*�Ƚ�*/               
	for(i=0;i<5;i++)
	{
		if(buf1[i]!=0xC2)
		break;
	} 
	       
	if(i==5)
		return 0 ;        //MCU��NRF�ɹ����� 
	else
		return 1 ;        //MCU��NRF����������
}

/**
	*	@name		NRF_Tx_Dat
	*	@brief		��NRF���ͻ�����д������
	*	@param		txbuf���洢�Ž��������ݵ�����
	*	@retval		�ɹ���ʧ��
	*/
static uint8_t NRF_Tx_Dat(uint8_t *txbuf)
{
	uint8_t state;  
//	uint16_t timeout_counter=0xFFFF;
	/*ceΪ�ͣ��������ģʽ1*/
	NRF_CE_LOW();
	/*д���ݵ�TX BUF ��� 32���ֽ�*/						
	SPI_NRF_WriteBuf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);
	/*CEΪ�ߣ�txbuf�ǿգ��������ݰ� */   
	NRF_CE_HIGH();
	/*�ȴ���������ж� */                            
	while((NRF_Read_IRQ()!=0));//&&timeout_counter--); 	
	/*��ȡ״̬�Ĵ�����ֵ */                              
	state = SPI_NRF_ReadReg(0x07);
	/*���TX_DS��MAX_RT�жϱ�־*/                  
	SPI_NRF_WriteReg(NRF_WRITE_REG+0x07,state); 	
	
	/*�ж��ж�����*/    
	if(state&MAX_RT)                     //�ﵽ����ط�����
	{
		SPI_NRF_WriteReg(FLUSH_TX,NOP);    //���TX FIFO�Ĵ��� 
		return MAX_RT; 
	}
	else if(state&TX_DS)                  //�������
		return TX_DS;
	else						  
		return ERROR;                 //����ԭ����ʧ��
} 

/**
	*	@name		NRF_Rx_Dat
	*	@brief		��NRF���ջ�������������
	*	@param		rxbuf���洢�������ݵ�����
	*	@retval		�ɹ���ʧ��
	*/
static uint8_t NRF_Rx_Dat(uint8_t *rxbuf)
{
	uint8_t state; 
	NRF_CE_HIGH();	 //�������״̬
	 /*�ȴ������ж�*/
	while(NRF_Read_IRQ()!=0); 

	NRF_CE_LOW();  	 //�������״̬
	/*��ȡstatus�Ĵ�����ֵ  */               
	state=SPI_NRF_ReadReg(STATUS);
	 
	/* ����жϱ�־*/      
	SPI_NRF_WriteReg(NRF_WRITE_REG+STATUS,state);

	/*�ж��Ƿ���յ�����*/
	if(state&RX_DR)                                 //���յ�����
	{
		SPI_NRF_ReadBuf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		SPI_NRF_WriteReg(FLUSH_RX,NOP);          //���RX FIFO�Ĵ���
		return RX_DR; 
	}
	else    
		return ERROR;                    //û�յ��κ�����
}
//********************�������װ����************************

//**********************��������****************************
/**
	*	@name		Cpu_Id_Get
	*	@brief		��ȡcpu��id
	*	@param		None
	*	@retval		�ɹ���ʧ��
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
	*	@brief		����Cpu_Id���ɶ�̬ͨѶ��ַ
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
//**********************�߳̿�ʼ****************************
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
	uint8_t send_temp_char[10],status;//���¶Ȳ��4��char����
	GPIO_Configuration();
	Dynamic_Comm_Address();
	while(NRF_Check());//������ôд��Ҫ�ӳ�ʱ�ж�
    while(1)
    {
			if(nrf24l01_status==NORMAL&&g_heartbeat_sync_flag==1)
			{
				g_heartbeat_sync_flag=0;
				NRF_TX_Mode(TX_ADDRESS);//���͹̶���ַ
				send_temp_char[0]=0x05;
				send_temp_char[1]=0x38;
				send_temp_char[2]=0x18;
				send_temp_char[3]=g_spo2_cmd;
				send_temp_char[4]=g_heartrate_cmd/256;
				send_temp_char[5]=g_heartrate_cmd%256;//������8λ
				NRF_Tx_Dat(send_temp_char);
				//rt_kprintf("nrf24l01_status=NORMAL\r\n");
			}
			else if(nrf24l01_status==PAIRING)
			{
				NRF_TX_Mode((uint8_t*)TX_ADDRESS);//���ͳ�ʼ��ַ
				//if(NRF_Tx_Dat(send_temp_char)==TX_DS)
				if(NRF_Tx_Dat(temp)==TX_DS)//���ͳɹ�
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
