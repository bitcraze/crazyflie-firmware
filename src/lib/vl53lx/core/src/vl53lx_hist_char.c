
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#include <stdio.h>
#include <stdlib.h>



#include "vl53lx_core.h"
#include "vl53lx_register_settings.h"
#include "vl53lx_hist_char.h"

#define LOG_FUNCTION_START(fmt, ...)
#define LOG_FUNCTION_END(status, ...)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)


VL53LX_Error VL53LX_set_calib_config(
	VL53LX_DEV      Dev,
	uint8_t         vcsel_delay__a0,
	uint8_t         calib_1,
	uint8_t         calib_2,
	uint8_t         calib_3,
	uint8_t         calib_2__a0,
	uint8_t         spad_readout)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;
	uint8_t      comms_buffer[3];

	LOG_FUNCTION_START("");



	status = VL53LX_enable_powerforce(Dev);


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_disable_firmware(Dev);




	if (status == VL53LX_ERROR_NONE) {
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__VCSEL_DELAY__A0,
					vcsel_delay__a0);
	}



	if (status == VL53LX_ERROR_NONE) {


		comms_buffer[0] = calib_1;
		comms_buffer[1] = calib_2;
		comms_buffer[2] = calib_3;

		status = VL53LX_WriteMulti(
					Dev,
					VL53LX_RANGING_CORE__CALIB_1,
					comms_buffer,
					3);
	}



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__CALIB_2__A0,
					calib_2__a0);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(
					Dev,
					VL53LX_RANGING_CORE__SPAD_READOUT,
					spad_readout);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_enable_firmware(Dev);

	LOG_FUNCTION_END(status);

	return status;
}



VL53LX_Error VL53LX_set_hist_calib_pulse_delay(
	VL53LX_DEV      Dev,
	uint8_t         calib_delay)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status =
		VL53LX_set_calib_config(
			Dev,
			0x01,
			calib_delay,
			0x04,
			0x08,
			0x14,
			VL53LX_RANGING_CORE__SPAD_READOUT__CALIB_PULSES);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_disable_calib_pulse_delay(
	VL53LX_DEV      Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status =
		VL53LX_set_calib_config(
			Dev,
			0x00,
			0x00,
			0x00,
			0x00,
			0x00,
			VL53LX_RANGING_CORE__SPAD_READOUT__STANDARD);

	LOG_FUNCTION_END(status);

	return status;
}


