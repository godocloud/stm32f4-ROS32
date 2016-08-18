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
 * @file VL6180x_api.h
 * $Date: 2015-01-07 12:15:09 +0100 (Wed, 07 Jan 2015) $
 * $Revision: 2020 $
 */



#ifndef VL6180x_API_H_
#define VL6180x_API_H_

#include "vl6180x_def.h"
#include <rthw.h>
#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup api_ll API Low Level Functions
 *  @brief    API Low level functions
 */

/** @defgroup api_hl API High Level Functions
 *  @brief    API High level functions
 */
 
extern	unsigned int Index;
	
/**
 * @brief Wait for device booted after chip enable (hardware standby)
 * @par Function Description
 * After Chip enable Application you can also simply wait at least 1ms to ensure device is ready
 * @warning After device chip enable (gpio0) de-asserted  user must wait gpio1 to get asserted (hardware standby).
 * or wait at least 400usec prior to do any low level access or api call .
 *
 * This function implements polling for standby but you must ensure 400usec from chip enable passed\n
 * @warning if device get prepared @a VL6180x_Prepare() re-using these function can hold indefinitely\n
 *
 * @param dev  The device
 * @return     0 on success
 */ 
//int VL6180x_WaitDeviceBooted(void);
int VL6180x_WriteZeroToResetFlag(void);
void VL6180x_ReadModelID(void);
int VL6180x_RangeStaticInit(void);

int VL6180x_Start_Range(void);
int VL6180x_Poll_Range(void);
int VL6180x_Read_Range(void);
int VL6180x_Clear_Interrupts(void); 
void VL6180x_Save_Range_Data(int data,int *buf);
int VL6180x_Get_Range_Average(int *buf);
/**
 * Write VL6180x single byte register
 * @param reg The register 
 * @param data  8 bit register data
 * @return 0 on success  
 */
int VL6180x_WriteByte( uint16_t reg, uint8_t data);
/**
 * Thread safe VL6180x Update (rd/modify/write) single byte register
 *
 * Final_reg = (Initial_reg & and_data) |or_data
 *
 * @param dev   The device index
 * @param index The register index
 * @param AndData  8 bit and data
 * @param OrData   8 bit or data
 * @return 0 on success
 */
//int VL6180x_UpdateByte(VL6180xDev_t dev, uint16_t index, uint8_t AndData, uint8_t OrData);
/**
 * Write VL6180x word register
 * @param reg The register index
 * @param data  16 bit register data
 * @return  0 on success
 */
int VL6180x_WriteWord( uint16_t reg, uint16_t data);
/**
 * Write VL6180x double word (4 byte) register
 * @param reg The register index
 * @param data  32 bit register data
 * @return  0 on success
 */
int VL6180x_WriteDoubleWord( uint16_t reg, uint32_t data);
/**
 * Read VL6180x single byte register
 * @param reg The register index
 * @return 8 bit receive data 
 */
int VL6180x_ReadByte( uint16_t reg );

/**
 * Read VL6180x word (2byte) register
 * @param reg The register index
 * @return 16 bit receive data 
 */
int VL6180x_ReadWord( uint16_t reg );

/**
 * Read VL6180x dword (4byte) register
 * @param reg The register index
 * @return 32 bit receive data 
 */
int  VL6180x_ReadDoubleWord( uint16_t reg );

/** @}  */

//void  VL6180x_GPIO_Init(void);
//void VL6180x_Enable(void);

void VL6180x_SampEntry(void *parameter);
void VL6180x_thread_creat(char priority , char tick);
rt_err_t demo_thread_creat(void);

#ifdef __cplusplus
}
#endif

#endif /* VL6180x_API_H_ */
