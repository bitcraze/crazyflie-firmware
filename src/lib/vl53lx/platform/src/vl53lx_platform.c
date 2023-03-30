
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file   vl53lx_platform.c
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
#include "vl53lx_api.h"
#include "vl53lx_platform.h"

// #ifdef PAL_EXTENDED
// 	#include "vl53l1_register_strings.h"
// #else
// 	#define VL53L1_get_register_name(a,b)
// #endif

// Set the start address 8 step after the VL53L0 dynamic addresses
static int nextI2CAddress = VL53LX_DEFAULT_ADDRESS+8;


bool VL53LX_Init(VL53LX_Dev_t *pdev, I2C_Dev *I2Cx)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;

  pdev->I2Cx = I2Cx;
  pdev->devAddr = VL53LX_DEFAULT_ADDRESS;

  /* Move initialized sensor to a new I2C address */
  int newAddress;

  taskENTER_CRITICAL();
  newAddress = nextI2CAddress++;
  taskEXIT_CRITICAL();

  vl53lxSetI2CAddress(pdev, newAddress);

  status = VL53LX_DataInit(pdev);

//   if (status == VL53LX_ERROR_NONE)
//   {
//     status = VL53LX_StaticInit(pdev);
//   }

  return status == VL53LX_ERROR_NONE;
}


bool vl53l1xTestConnection(VL53LX_Dev_t* pdev)
{
  VL53LX_DeviceInfo_t info;
  VL53LX_Error status = VL53LX_ERROR_NONE;

  status = VL53LX_GetDeviceInfo(pdev, &info);

  return status == VL53LX_ERROR_NONE;
}

/** Set I2C address
 * Any subsequent communication will be on the new address
 * The address passed is the 7bit I2C address from LSB (ie. without the
 * read/write bit)
 */
VL53LX_Error vl53lxSetI2CAddress(VL53LX_Dev_t* pdev, uint8_t address)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;

  status = VL53LX_SetDeviceAddress(pdev, address);
  pdev->devAddr = address;
  return  status;
}


/*
 * ----------------- COMMS FUNCTIONS -----------------
 */

VL53LX_Error VL53LX_WriteMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

  if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, count, pdata))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }

	return status;
}

VL53LX_Error VL53LX_ReadMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, count, pdata))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       data)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, 1, &data))
	{
	  status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      data)
{
  VL53LX_Error status         = VL53LX_ERROR_NONE;

  if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, 2, (uint8_t *)&data))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53LX_Error VL53LX_WrDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t      data)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;

	if (!i2cdevWrite16(pdev->I2Cx, pdev->devAddr, index, 4, (uint8_t *)&data))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }

	return status;
}


VL53LX_Error VL53LX_RdByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
  VL53LX_Error status         = VL53LX_ERROR_NONE;
  static uint8_t r8data;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, 1, &r8data))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }
  *pdata = r8data;

  return status;
}


VL53LX_Error VL53LX_RdWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
  VL53LX_Error status         = VL53LX_ERROR_NONE;
  static uint16_t r16data;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, 2, (uint8_t *)&r16data))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }
  *pdata = r16data;
  
  return status;
}


VL53LX_Error VL53LX_RdDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata)
{
  VL53LX_Error status = VL53LX_ERROR_NONE;
  static uint32_t r32data;

  if (!i2cdevRead16(pdev->I2Cx, pdev->devAddr, index, 4, (uint8_t *)&r32data))
  {
    status = VL53LX_ERROR_CONTROL_INTERFACE;
  }
  *pdata = r32data;

  return status;
}

/*
 * ----------------- HOST TIMING FUNCTIONS -----------------
 */

VL53LX_Error VL53LX_WaitUs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_us)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t delay_ms = (wait_us + 900) / 1000;

	if(delay_ms == 0)
	{
	  delay_ms = 1;
	}

	vTaskDelay(M2T(delay_ms));

	return status;
}


VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms)
{
  vTaskDelay(M2T(wait_ms));

  return VL53LX_ERROR_NONE;
}

/*
 * ----------------- DEVICE TIMING FUNCTIONS -----------------
 */

VL53LX_Error VL53LX_GetTickCount(
	uint32_t *ptick_count_ms)
{
	/* Returns current tick count in [ms] */
	*ptick_count_ms = xTaskGetTickCount();

	return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
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

	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;
#ifdef VL53LX_LOG_ENABLE
	uint32_t     trace_functions = 0;
#endif

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

#ifdef VL53LX_LOG_ENABLE
	/* look up register name */
	VL53LX_get_register_name(
			index,
			register_name);

	/* Output to I2C logger for FMT/DFT  */
	trace_i2c("WaitValueMaskEx(%5d, %s, 0x%02X, 0x%02X, %5d);\n",
		timeout_ms, register_name, value, mask, poll_delay_ms);
#endif // VL53L1_LOG_ENABLE

	/* calculate time limit in absolute time */

	VL53LX_GetTickCount(&start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	/* remember current trace functions and temporarily disable
	 * function logging
	 */

#ifdef VL53LX_LOG_ENABLE
	trace_functions = _LOG_GET_TRACE_FUNCTIONS();
#endif

	/* wait until value is found, timeout reached on error occurred */

	while ((status == VL53LX_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53LX_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		if (status == VL53LX_ERROR_NONE  &&
			found == 0 &&
			poll_delay_ms > 0)
			status = VL53LX_WaitMs(
							pdev,
							poll_delay_ms);

		/* Update polling time (Compare difference rather than absolute to
		negate 32bit wrap around issue) */
		VL53LX_GetTickCount(&current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}

	if (found == 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_TIME_OUT;

	return status;
}

