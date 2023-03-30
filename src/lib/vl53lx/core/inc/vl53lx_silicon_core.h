
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53LX_SILICON_CORE_H_
#define _VL53LX_SILICON_CORE_H_

#include "vl53lx_platform.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53LX_Error VL53LX_is_firmware_ready_silicon(
	VL53LX_DEV      Dev,
	uint8_t        *pready);


#ifdef __cplusplus
}
#endif

#endif

