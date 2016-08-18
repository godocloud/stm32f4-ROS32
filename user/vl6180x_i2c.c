
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
 * $Date: 2015-01-08 14:30:24 +0100 (Thu, 08 Jan 2015) $
 * $Revision: 2039 $
 */

/**
 * @file vl6180x_i2c.c
 *
 * Copyright (C) 2014 ST MicroElectronics
 *
 * provide variable word size byte/Word/dword VL6180x register access via i2c
 *
 */
#include "i2c_bit.h"

#include "vl6180x_def.h"

int VL6180x_WriteByte( uint16_t reg, uint8_t data)
{
    int  status=0;

	  IIC_Start(); 
	  IIC_Send_Byte((theVL6180xDev<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî
		if(IIC_Wait_Ack())	//µÈ´ýÓ¦´ð
		{
			IIC_Stop();		 
			return 1;		
		}
		IIC_Send_Byte(reg>>8);	//Ð´¼Ä´æÆ÷µØÖ·¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(reg%256);	//Ð´¼Ä´æÆ÷µØÖ·µÍÎ»  IIC_Send_Byte(reg&0xff)Ò²¿ÉÒÔ
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		
		IIC_Send_Byte(data);//·¢ËÍÊý¾Ý
		if(IIC_Wait_Ack())	//µÈ´ýACK
		{
			IIC_Stop();	 
			return 1;		 
		}		 
		IIC_Stop();	

    return status;
}

int VL6180x_WriteWord( uint16_t reg, uint16_t data)
{
    int  status;
	
	  IIC_Start(); 
	  IIC_Send_Byte((theVL6180xDev<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî
		if(IIC_Wait_Ack())	//µÈ´ýÓ¦´ð
		{
			IIC_Stop();		 
			return 1;		
		}
		IIC_Send_Byte(reg>>8);	//Ð´¼Ä´æÆ÷µØÖ·¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(reg%256);	//Ð´¼Ä´æÆ÷µØÖ·µÍÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		
		IIC_Send_Byte(data>>8);//·¢ËÍÊý¾Ý¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(data%256);//·¢ËÍÊý¾ÝµÍÎ»
		if(IIC_Wait_Ack())	//µÈ´ýACK
		{
			IIC_Stop();	 
			return 1;		 
		}		 
		IIC_Stop();	
		
    return status;
}

int VL6180x_WriteDoubleWord( uint16_t reg, uint32_t data)
{
  	int  status;
	
	  IIC_Start(); 
	  IIC_Send_Byte((theVL6180xDev<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî
		if(IIC_Wait_Ack())	//µÈ´ýÓ¦´ð
		{
			IIC_Stop();		 
			return 1;		
		}
		IIC_Send_Byte(reg>>8);	//Ð´¼Ä´æÆ÷µØÖ·¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(reg%256);	//Ð´¼Ä´æÆ÷µØÖ·µÍÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		
		IIC_Send_Byte(data>>24);//·¢ËÍÊý¾Ý¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
		IIC_Send_Byte(data>>16);//·¢ËÍÊý¾Ý¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
		IIC_Send_Byte(data>>8); //·¢ËÍÊý¾Ý¸ßÎ»
		IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(data%256);//·¢ËÍÊý¾ÝµÍÎ»
		if(IIC_Wait_Ack())	//µÈ´ýACK
		{
			IIC_Stop();	 
			return 1;		 
		}		 
		IIC_Stop();	
		
    return status;
}

//int VL6180x_UpdateByte(VL6180xDev_t dev, uint16_t index, uint8_t AndData, uint8_t OrData){
//    VL6180x_I2C_USER_VAR
//    int  status;
//    uint8_t *buffer;
//    DECL_I2C_BUFFER

//    VL6180x_GetI2CAccess(dev);

//    buffer=VL6180x_GetI2cBuffer(dev,3);
//    buffer[0]=index>>8;
//    buffer[1]=index&0xFF;

//    status=VL6180x_I2CWrite(dev, (uint8_t *)buffer,(uint8_t)2);
//    if( !status ){
//        /* read data direct onto buffer */
//        status=VL6180x_I2CRead(dev, &buffer[2],1);
//        if( !status ){
//            buffer[2]=(buffer[2]&AndData)|OrData;
//            status=VL6180x_I2CWrite(dev, buffer, (uint8_t)3);
//        }
//    }

//    VL6180x_DoneI2CAcces(dev);

//    return status;
//}

int VL6180x_ReadByte( uint16_t reg )
{
    uint8_t  res;
	
    IIC_Start(); 
	  IIC_Send_Byte((theVL6180xDev<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	  IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
    IIC_Send_Byte(reg>>8);	//Ð´¼Ä´æÆ÷µØÖ·¸ßÎ»
	  IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(reg%256);	//Ð´¼Ä´æÆ÷µØÖ·µÍÎ»
    IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
	
    IIC_Start();
	  IIC_Send_Byte((theVL6180xDev<<1)|1);//·¢ËÍÆ÷¼þµØÖ·+¶ÁÃüÁî	
    IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
	
	  res=IIC_Read_Byte(0);//¶ÁÈ¡Êý¾Ý,·¢ËÍnACK 
    IIC_Stop();			//²úÉúÒ»¸öÍ£Ö¹Ìõ¼þ 
	
	  return res;	
}

int VL6180x_ReadWord( uint16_t reg )
{
	  uint16_t  res;
	  uint8_t *buffer;
	
    IIC_Start(); 
	  IIC_Send_Byte((theVL6180xDev<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	  IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
    IIC_Send_Byte(reg>>8);	//Ð´¼Ä´æÆ÷µØÖ·¸ßÎ»
	  IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(reg%256);	//Ð´¼Ä´æÆ÷µØÖ·µÍÎ»
    IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
	
    IIC_Start();
	  IIC_Send_Byte((theVL6180xDev<<1)|1);//·¢ËÍÆ÷¼þµØÖ·+¶ÁÃüÁî	
    IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
	
	  buffer[0]=IIC_Read_Byte(1);//¶ÁÈ¡Êý¾Ý,·¢ËÍACK 
	  buffer[1]=IIC_Read_Byte(0);//¶ÁÈ¡Êý¾Ý,·¢ËÍnACK 
    IIC_Stop();			//²úÉúÒ»¸öÍ£Ö¹Ìõ¼þ 
	
	  res=((uint16_t)buffer[0]<<8)|((uint16_t)buffer[1]);
	  return res;	
}

int  VL6180x_ReadDoubleWord( uint16_t reg )
{
	  uint32_t  res;
	  uint8_t *buffer;
	
    IIC_Start(); 
	  IIC_Send_Byte((theVL6180xDev<<1)|0);//·¢ËÍÆ÷¼þµØÖ·+Ð´ÃüÁî	
	  IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
    IIC_Send_Byte(reg>>8);	//Ð´¼Ä´æÆ÷µØÖ·¸ßÎ»
	  IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
		IIC_Send_Byte(reg%256);	//Ð´¼Ä´æÆ÷µØÖ·µÍÎ»
    IIC_Wait_Ack();		//µÈ´ýÓ¦´ð
	
    IIC_Start();
	  IIC_Send_Byte((theVL6180xDev<<1)|1);//·¢ËÍÆ÷¼þµØÖ·+¶ÁÃüÁî	
    IIC_Wait_Ack();		//µÈ´ýÓ¦´ð 
	
	  buffer[0]=IIC_Read_Byte(1);//¶ÁÈ¡Êý¾Ý,·¢ËÍACK 
	  buffer[1]=IIC_Read_Byte(1);//¶ÁÈ¡Êý¾Ý,·¢ËÍACK 
	  buffer[2]=IIC_Read_Byte(1);//¶ÁÈ¡Êý¾Ý,·¢ËÍACK 
	  buffer[3]=IIC_Read_Byte(0);//¶ÁÈ¡Êý¾Ý,·¢ËÍnACK 
    IIC_Stop();			//²úÉúÒ»¸öÍ£Ö¹Ìõ¼þ 
	
	  res=((uint32_t)buffer[0]<<24)|((uint32_t)buffer[1]<<16)|((uint32_t)buffer[2]<<8)|((uint32_t)buffer[3]);
	  return res;	
}
