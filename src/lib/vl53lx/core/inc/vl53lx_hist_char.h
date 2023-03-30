
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifndef _VL53LX_HIST_CHAR_H_
#define _VL53LX_HIST_CHAR_H_

#include "vl53lx_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif




VL53LX_Error VL53LX_set_calib_config(
	VL53LX_DEV      Dev,
	uint8_t         vcsel_delay__a0,
	uint8_t         calib_1,
	uint8_t         calib_2,
	uint8_t         calib_3,
	uint8_t         calib_2__a0,
	uint8_t         spad_readout);




VL53LX_Error VL53LX_set_hist_calib_pulse_delay(
	VL53LX_DEV      Dev,
	uint8_t         calib_delay);




VL53LX_Error VL53LX_disable_calib_pulse_delay(
	VL53LX_DEV      Dev);


#ifdef __cplusplus
}
#endif

#endif

