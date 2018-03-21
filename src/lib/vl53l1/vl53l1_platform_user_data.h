/*******************************************************************************
 Copyright (C) 2016, STMicroelectronics International N.V.
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
 ******************************************************************************/


#ifndef _VL53L1_PLATFORM_USER_DATA_H_
#define _VL53L1_PLATFORM_USER_DATA_H_

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#include "vl53l1_def.h"
#include "i2cdev.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @file   vl53l1_platform_user_data.h
 *
 * @brief  All end user OS/platform/application porting
 */

/** @brief  Contains the current state and internal values of the API
 */

typedef struct {

	VL53L1_DevData_t   Data;
	/*!< Low Level Driver data structure */

  uint8_t devAddr;
  /*!< i2c device address user specific field */
  I2C_Dev *I2Cx;

	uint32_t  new_data_ready_poll_duration_ms;
		/*!< New data ready poll duration in ms - for debug */
} VL53L1_Dev_t;


/**
 * @brief   Declare the device Handle as a pointer of the structure @a VL53L1_Dev_t.
 *
 */
typedef VL53L1_Dev_t *VL53L1_DEV;

/**
 * @def PALDevDataGet
 * @brief Get ST private structure @a VL53L1_DevData_t data access
 *
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * It maybe used and as real data "ref" not just as "get" for sub-structure item
 * like PALDevDataGet(FilterData.field)[i] or
 * PALDevDataGet(FilterData.MeasurementIndex)++
 */
#define PALDevDataGet(Dev, field) (Dev->Data.field)


/**
 * @def PALDevDataSet(Dev, field, data)
 * @brief  Set ST private structure @a VL53L1_DevData_t data field
 * @param Dev       Device Handle
 * @param field     ST structure field name
 * @param data      Data to be set
 */
#define PALDevDataSet(Dev, field, data) ((Dev->Data.field) = (data))


/**
 * @def VL53L1DevStructGetLLDriverHandle
 * @brief Get LL Driver handle @a VL53L0_Dev_t data access
 *
 * @param Dev      Device Handle
 */
#define VL53L1DevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)

/**
 * @def VL53L1DevStructGetLLResultsHandle
 * @brief Get LL Results handle @a VL53L0_Dev_t data access
 *
 * @param Dev      Device Handle
 */
#define VL53L1DevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)



#ifdef __cplusplus
}
#endif

#endif

