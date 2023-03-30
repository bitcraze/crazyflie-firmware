
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53LX_WAIT_H_
#define _VL53LX_WAIT_H_

#include "vl53lx_platform.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53LX_Error VL53LX_wait_for_boot_completion(
	VL53LX_DEV      Dev);




VL53LX_Error VL53LX_wait_for_range_completion(
	VL53LX_DEV   Dev);




VL53LX_Error VL53LX_wait_for_test_completion(
	VL53LX_DEV   Dev);






VL53LX_Error VL53LX_is_boot_complete(
	VL53LX_DEV      Dev,
	uint8_t        *pready);



VL53LX_Error VL53LX_is_firmware_ready(
	VL53LX_DEV      Dev,
	uint8_t        *pready);




VL53LX_Error VL53LX_is_new_data_ready(
	VL53LX_DEV      Dev,
	uint8_t        *pready);






VL53LX_Error VL53LX_poll_for_boot_completion(
	VL53LX_DEV      Dev,
	uint32_t        timeout_ms);




VL53LX_Error VL53LX_poll_for_firmware_ready(
	VL53LX_DEV      Dev,
	uint32_t        timeout_ms);




VL53LX_Error VL53LX_poll_for_range_completion(
	VL53LX_DEV   Dev,
	uint32_t     timeout_ms);



#ifdef __cplusplus
}
#endif

#endif

