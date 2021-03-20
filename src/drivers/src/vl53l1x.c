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

/**
 * @file   vl53l1_platform.c
 * @brief  Code function definitions for Crazyflie
 *
 */
#include <stdio.h>      // sprintf(), vsnprintf(), printf()
#include <stdint.h>
#include <string.h>     // strncpy(), strnlen()

#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "i2cdev.h"
#include "vl53l1x.h"


#ifdef PAL_EXTENDED
	#include "vl53l1_register_strings.h"
#else
	#define VL53L1_get_register_name(a,b)
#endif

// Set the start address 8 step after the VL53L0 dynamic addresses
static int nextI2CAddress = VL53L1X_DEFAULT_ADDRESS+8;


bool vl53l1xInit(VL53L1_Dev_t *pdev, I2C_Dev *I2Cx)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;

  pdev->I2Cx = I2Cx;
  pdev->devAddr = VL53L1X_DEFAULT_ADDRESS;

  /* Move initialized sensor to a new I2C address */
  int newAddress;

  taskENTER_CRITICAL();
  newAddress = nextI2CAddress++;
  taskEXIT_CRITICAL();

  vl53l1xSetI2CAddress(pdev, newAddress);

  status = VL53L1_DataInit(pdev);

  if (status == VL53L1_ERROR_NONE)
  {
    status = VL53L1_StaticInit(pdev);
  }

  return status == VL53L1_ERROR_NONE;
}

bool vl53l1xTestConnection(VL53L1_Dev_t* pdev)
{
  VL53L1_DeviceInfo_t info;
  VL53L1_Error status = VL53L1_ERROR_NONE;

  status = VL53L1_GetDeviceInfo(pdev, &info);

  return status == VL53L1_ERROR_NONE;
}

/** Set I2C address
 * Any subsequent communication will be on the new address
 * The address passed is the 7bit I2C address from LSB (ie. without the
 * read/write bit)
 */
VL53L1_Error vl53l1xSetI2CAddress(VL53L1_Dev_t* pdev, uint8_t address)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;

  status = VL53L1_SetDeviceAddress(pdev, address);
  pdev->devAddr = address;
  return  status;
}


/*
 * ----------------- COMMS FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WriteMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

  if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, count, pdata))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}

VL53L1_Error VL53L1_ReadMulti(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, count, pdata))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53L1_Error VL53L1_WrByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, 1, &data))
	{
	  status = VL53L1_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53L1_Error VL53L1_WrWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
  VL53L1_Error status         = VL53L1_ERROR_NONE;

  if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, 2, (uint8_t *)&data))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53L1_Error VL53L1_WrDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;

	if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, 4, (uint8_t *)&data))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53L1_Error VL53L1_RdByte(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
  VL53L1_Error status         = VL53L1_ERROR_NONE;
  static uint8_t r8data;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, 1, &r8data))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
  *pdata = r8data;

  return status;
}


VL53L1_Error VL53L1_RdWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
  VL53L1_Error status         = VL53L1_ERROR_NONE;
  static uint16_t r16data;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, 2, (uint8_t *)&r16data))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
  *pdata = r16data;
  
  return status;
}


VL53L1_Error VL53L1_RdDWord(
	VL53L1_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  static uint32_t r32data;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, 4, (uint8_t *)&r32data))
  {
    status = VL53L1_ERROR_CONTROL_INTERFACE;
  }
  *pdata = r32data;

  return status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_WaitUs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_us)
{
	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t delay_ms = (wait_us + 900) / 1000;

	if(delay_ms == 0)
	{
	  delay_ms = 1;
	}

	vTaskDelay(M2T(delay_ms));

	return status;
}


VL53L1_Error VL53L1_WaitMs(
	VL53L1_Dev_t *pdev,
	int32_t       wait_ms)
{
  vTaskDelay(M2T(wait_ms));

  return VL53L1_ERROR_NONE;
}

/*
 * ----------------- DEVICE TIMING FUNCTIONS -----------------
 */

VL53L1_Error VL53L1_GetTickCount(
	uint32_t *ptick_count_ms)
{
	/* Returns current tick count in [ms] */
	*ptick_count_ms = xTaskGetTickCount();

	return VL53L1_ERROR_NONE;
}


VL53L1_Error VL53L1_WaitValueMaskEx(
	VL53L1_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{
	/*
	 * Platform implementation of WaitValueMaskEx V2WReg script command
	 *
	 * WaitValueMaskEx(
	 *          duration_ms,
	 *          index,
	 *          value,
	 *          mask,
	 *          poll_delay_ms);
	 */

	VL53L1_Error status         = VL53L1_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53L1_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53L1_LOG_ENABLE
	/* look up register name */
	VL53L1_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	VL53L1_GetTickCount(&start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53L1_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53L1_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53L1_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		if (status == VL53L1_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53L1_WaitMs(
							pdev,
							poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53L1_GetTickCount(&current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	if (found == 0 && status == VL53L1_ERROR_NONE)
		status = VL53L1_ERROR_TIME_OUT;

	return status;
}

