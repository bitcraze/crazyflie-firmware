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
*******************************************************************************/

/**
 * @file  vl53l1_wait.h
 *
 * @brief EwokPlus25 low level Driver wait function definitions
 */

#ifndef _VL53L1_WAIT_H_
#define _VL53L1_WAIT_H_

#include "../../../../drivers/interface/vl53l1x.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Wait for initial firmware boot to finish
 *
 * Calls VL53L1_poll_for_boot_completion()
 *
 * @param[in]   Dev           : Device handle
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_wait_for_boot_completion(
	VL53L1_DEV      Dev);


/**
 * @brief Waits for initial firmware ready
 *
 * Only waits to see if the firmware is ready in timed and
 * single shot modes.
 *
 * Calls VL53L1_poll_for_firmware_ready()
 *
 * @param[in]   Dev           : Device handle
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_wait_for_firmware_ready(
	VL53L1_DEV      Dev);


/**
 * @brief  Waits for the next ranging interrupt
 *
 * Calls VL53L1_poll_for_range_completion()
 *
 * @param[in]   Dev             :  Device handle
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_wait_for_range_completion(
	VL53L1_DEV   Dev);


/**
 * @brief  Waits for a device test mode to complete.

 * Calls VL53L1_poll_for_test_completion()
 *
 * @param[in]   Dev             : Device Handle
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_wait_for_test_completion(
	VL53L1_DEV   Dev);




/**
 * @brief Reads FIRMWARE__SYSTEM_STATUS register to detect if the
 *        firmware was finished booting
 *
 * @param[in]   Dev           : Device handle
 * @param[out]  pready        : pointer to data ready flag \n
 *                                 0 = boot not complete \n
 *                                 1 = boot complete
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_is_boot_complete(
	VL53L1_DEV      Dev,
	uint8_t        *pready);

/**
 * @brief Reads FIRMWARE__SYSTEM_STATUS register to detect if the
 *        firmware is ready for ranging.
 *
 * @param[in]   Dev           : Device handle
 * @param[out]  pready        : pointer to data ready flag \n
 *                                 0 = firmware not ready \n
 *                                 1 = firmware ready
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_is_firmware_ready(
	VL53L1_DEV      Dev,
	uint8_t        *pready);


/**
 * @brief  Reads bit 0 of VL53L1_GPIO__TIO_HV_STATUS register to determine
 *         if new range data is ready (available).
 *
 * Interrupt may be either active high or active low. The behaviour of bit 0
 * of the VL53L1_GPIO__TIO_HV_STATUS register is the same as the interrupt
 * signal generated on the GPIO pin.
 *
 * pdev->stat_cfg.gpio_hv_mux_ctrl bit 4 is used to select required check level
 *
 *
 * @param[in]   Dev           : Device handle
 * @param[out]  pready        : pointer to data ready flag \n
 *                                 0 = data not ready \n
 *                                 1 = new range data available
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_is_new_data_ready(
	VL53L1_DEV      Dev,
	uint8_t        *pready);




/**
 * @brief Waits (polls) for initial firmware boot to finish
 *
 * After power on or a device reset via XSHUTDOWN EwokPlus25 firmware takes
 * about 2ms to boot. During this boot sequence elected NVM data is copied
 * to the device's Host & MCU G02 registers
 *
 * This function polls the FIRMWARE__SYSTEM_STATUS register to detect when
 * firmware is ready.
 *
 * Polling is implemented using VL53L1_WaitValueMaskEx()
 *
 * @param[in]   Dev           : Device handle
 * @param[in]   timeout_ms    : Wait timeout in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_poll_for_boot_completion(
	VL53L1_DEV      Dev,
	uint32_t        timeout_ms);


/**
 * @brief Waits (polls) for initial firmware ready
 *
 * Polling is implemented using VL53L1_WaitValueMaskEx()
 *
 * @param[in]   Dev           : Device handle
 * @param[in]   timeout_ms    : Wait timeout in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_poll_for_firmware_ready(
	VL53L1_DEV      Dev,
	uint32_t        timeout_ms);


/**
 * @brief  Polls bit 0 of VL53L1_GPIO__TIO_HV_STATUS register to determine
 *         the state of the GPIO (Interrupt) pin.
 *
 * Interrupt may be either active high or active low. The behaviour of bit 0
 * of the VL53L1_GPIO__TIO_HV_STATUS register is the same as the interrupt
 * signal generated on the GPIO pin.
 *
 * pdev->stat_cfg.gpio_hv_mux_ctrl bit 4 is used to select required check level
 *
 * Polling is implemented using VL53L1_WaitValueMaskEx()
 *
 * @param[in]   Dev           :  Device handle
 * @param[in]   timeout_ms    : Wait timeout in [ms]
 *
 * @return  VL53L1_ERROR_NONE     Success
 * @return  "Other error code"    See ::VL53L1_Error
 */

VL53L1_Error VL53L1_poll_for_range_completion(
	VL53L1_DEV   Dev,
	uint32_t     timeout_ms);



#ifdef __cplusplus
}
#endif

#endif /* _VL53L1_WAIT_H_ */
