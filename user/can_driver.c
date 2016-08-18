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
#include "can_driver.h"
#include <board.h>
/*	Global variables -----------------------------------------------*/
CAN_InitTypeDef        CAN_InitStructure;
CAN_FilterInitTypeDef  CAN_FilterInitStructure;
CanTxMsg TxMessage;
CanRxMsg RxMessage;

extern struct rt_mailbox can_received_mailbox;
/*	Private typedef ------------------------------------------------*/
/*	Private define -------------------------------------------------*/
/*	Private macro --------------------------------------------------*/
/*	Private variables ----------------------------------------------*/
/*	Private function prototypes ------------------------------------*/
/*	Private fuctions -----------------------------------------------*/


/**
  * @brief  Configures the CAN.
  * @param  None
  * @retval None
  */
void CAN_Config(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CANx);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = ENABLE;
  CAN_InitStructure.CAN_AWUM = DISABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 0.5 MBps (CAN clocked at 42 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_7tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq;
  CAN_InitStructure.CAN_Prescaler = 6;
  CAN_Init(CANx, &CAN_InitStructure);

  /* CAN filter init */
#ifdef  USE_CAN1
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
#endif  /* USE_CAN1 */
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;//CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;//CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = (0x0070)<<5;//0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0xFFFF;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

/**
  * @brief  Configures the NVIC for CAN.
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
  NVIC_InitTypeDef  NVIC_InitStructure;

#ifdef  USE_CAN1 
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
#endif /* USE_CAN1 */

  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



uint8_t CAN_PushMessage(CanTxMsg  *pTransmitBuf )
{
  uint8_t transmit_mailbox = 0;
	CanTxMsg TxMessage;
  if(pTransmitBuf -> DLC > 8)
  {
       return  1;
  }
  /* transmit */
  TxMessage=*pTransmitBuf;
//  TxMessage.StdId=pTransmitBuf ->StdId;//用来设定标准标识符（0-0x7ff，11位）
//  TxMessage.ExtId=0x01;//随便设一个，因为IDE设为标志模式，这里用不到的
//  TxMessage.RTR=  CAN_RTR_DATA;//设置RTR位为数据帧
//  TxMessage.IDE=  CAN_ID_STD;//标识符设置为标准帧
//  TxMessage.DLC=  pTransmitBuf ->LEN;//设置数据长度
//  //根据DLC字段的值，将有效数据拷贝到发送数据寄存器
//  rt_memcpy(TxMessage.Data, pTransmitBuf ->BUF,pTransmitBuf ->LEN);
  //发送
  transmit_mailbox=CAN_Transmit(CANx,&TxMessage);

  return (transmit_mailbox);
}


/**
  * @brief  Initializes the Rx Message.
  * @param  RxMessage: pointer to the message to initialize
  * @retval None
  */
void Init_RxMes(CanRxMsg *RxMessage)
{
  uint8_t i = 0;

  RxMessage->StdId = 0x00;
  RxMessage->ExtId = 0x00;
  RxMessage->IDE = CAN_ID_STD;
  RxMessage->DLC = 0;
  RxMessage->FMI = 0;
  for (i = 0;i < 8;i++)
  {
    RxMessage->Data[i] = 0x00;
  }
}


#ifdef USE_CAN1
/**
  * @brief  This function handles CAN1 RX0 request.
  * @param  None
  * @retval None
  */
extern struct rt_messagequeue can_rx_mq;
void CAN1_RX0_IRQHandler(void)
{
	/* enter interrupt */
    rt_interrupt_enter();

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	/* 发送消息到消息队列中*/ 
	rt_mq_send(&can_rx_mq, &RxMessage, sizeof(CanRxMsg)); 
	
	/* leave interrupt */
    rt_interrupt_leave();
}
#endif  /* USE_CAN1 */




/*******************(C)	COPYRIGHT TELLYES *****END OF FILE***********/
