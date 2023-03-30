
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifndef _VL53LX_HIST_MAP_H_
#define _VL53LX_HIST_MAP_H_

#include "vl53lx_register_map.h"

#ifdef __cplusplus
extern "C"
{
#endif




#define VL53LX_HISTOGRAM_CONFIG__OPCODE_SEQUENCE_0 \
			VL53LX_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS

#define VL53LX_HISTOGRAM_CONFIG__OPCODE_SEQUENCE_1 \
		    VL53LX_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS

#define VL53LX_HISTOGRAM_CONFIG__OPCODE_SEQUENCE_2 \
		    VL53LX_SIGMA_ESTIMATOR__SIGMA_REF_MM

#define VL53LX_HISTOGRAM_CONFIG__AMB_THRESH_HIGH \
		    VL53LX_ALGO__RANGE_IGNORE_THRESHOLD_MCPS




#define VL53LX_RESULT__HISTOGRAM_BIN_0_2                               0x008E
#define VL53LX_RESULT__HISTOGRAM_BIN_0_1                               0x008F
#define VL53LX_RESULT__HISTOGRAM_BIN_0_0                               0x0090

#define VL53LX_RESULT__HISTOGRAM_BIN_23_2                              0x00D3
#define VL53LX_RESULT__HISTOGRAM_BIN_23_1                              0x00D4
#define VL53LX_RESULT__HISTOGRAM_BIN_23_0                              0x00D5

#define VL53LX_RESULT__HISTOGRAM_BIN_23_0_MSB                          0x00D9
#define VL53LX_RESULT__HISTOGRAM_BIN_23_0_LSB                          0x00DA



#define VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX       \
	VL53LX_RESULT__INTERRUPT_STATUS
#define VL53LX_HISTOGRAM_BIN_DATA_I2C_SIZE_BYTES  \
	(VL53LX_RESULT__HISTOGRAM_BIN_23_0_LSB - \
		VL53LX_RESULT__INTERRUPT_STATUS + 1)

#ifdef __cplusplus
}
#endif

#endif

