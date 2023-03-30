
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */
/**
 * @file   vl53lx_platform.h
 *
 * @brief  All end user OS/platform/application porting
 */


#ifndef _VL53LX_PLATFORM_H_
#define _VL53LX_PLATFORM_H_

#include "vl53lx_ll_def.h"
#include "vl53lx_platform_user_data.h"
#include "vl53lx_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define VL53LX_DEFAULT_ADDRESS 0b0101001

#define USE_I2C_2V8


bool VL53LX_Init(VL53LX_Dev_t *pdev, I2C_Dev *I2Cx);

bool vl53lxTestConnection(VL53LX_Dev_t* pdev);

VL53LX_Error vl53lxSetI2CAddress(VL53LX_Dev_t* pdev, uint8_t address);

/**
 * @brief Writes the supplied byte buffer to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   pdata     : pointer to uint8_t (byte) buffer containing the data to be written
 * @param[in]   count     : number of bytes in the supplied byte buffer
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WriteMulti(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Reads the requested number of bytes from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to the uint8_t (byte) buffer to store read data
 * @param[in]   count     : number of bytes to read
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_ReadMulti(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata,
		uint32_t      count);


/**
 * @brief  Writes a single byte to the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint8_t data value to write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrByte(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t       data);


/**
 * @brief  Writes a single word (16-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uin16_t data value write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint16_t      data);


/**
 * @brief  Writes a single dword (32-bit unsigned) to the device
 *
 * Manages the big-endian nature of the device register map
 * (first byte written is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[in]   data      : uint32_t data value to write
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WrDWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint32_t      data);



/**
 * @brief  Reads a single byte from the device
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index
 * @param[out]  pdata     : pointer to uint8_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 *
 */

VL53LX_Error VL53LX_RdByte(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint8_t      *pdata);


/**
 * @brief  Reads a single word (16-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint16_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_RdWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint16_t     *pdata);


/**
 * @brief  Reads a single dword (32-bit unsigned) from the device
 *
 * Manages the big-endian nature of the device (first byte read is the MS byte).
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   index     : uint16_t register index value
 * @param[out]  pdata     : pointer to uint32_t data value
 *
 * @return   VL53LX_ERROR_NONE    Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_RdDWord(
		VL53LX_Dev_t *pdev,
		uint16_t      index,
		uint32_t     *pdata);



/**
 * @brief  Implements a programmable wait in us
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_us   : integer wait in micro seconds
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitUs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_us);


/**
 * @brief  Implements a programmable wait in ms
 *
 * @param[in]   pdev      : pointer to device structure (device handle)
 * @param[in]   wait_ms   : integer wait in milliseconds
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitMs(
		VL53LX_Dev_t *pdev,
		int32_t       wait_ms);

/*
 * @brief Gets current system tick count in [ms]
 *
 * @return  time_ms : current time in [ms]
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_GetTickCount(
	uint32_t *ptime_ms);


/**
 * @brief Register "wait for value" polling routine
 *
 * Port of the V2WReg Script function  WaitValueMaskEx()
 *
 * @param[in]   pdev          : pointer to device structure (device handle)
 * @param[in]   timeout_ms    : timeout in [ms]
 * @param[in]   index         : uint16_t register index value
 * @param[in]   value         : value to wait for
 * @param[in]   mask          : mask to be applied before comparison with value
 * @param[in]   poll_delay_ms : polling delay been each read transaction in [ms]
 *
 * @return  VL53LX_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53LX_Error
 */

VL53LX_Error VL53LX_WaitValueMaskEx(
		VL53LX_Dev_t *pdev,
		uint32_t      timeout_ms,
		uint16_t      index,
		uint8_t       value,
		uint8_t       mask,
		uint32_t      poll_delay_ms);

#ifdef __cplusplus
}
#endif

#endif

