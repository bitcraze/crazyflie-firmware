
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */







#ifndef _VL53LX_DMAX_STRUCTS_H_
#define _VL53LX_DMAX_STRUCTS_H_

#include "vl53lx_types.h"

#ifdef __cplusplus
extern "C"
{
#endif


#define VL53LX_MAX_AMBIENT_DMAX_VALUES        5





typedef struct {



	uint16_t  ref__actual_effective_spads;

	uint16_t  ref__peak_signal_count_rate_mcps;

	uint16_t  ref__distance_mm;

	uint16_t   ref_reflectance_pc;




	uint16_t   coverglass_transmission;


} VL53LX_dmax_calibration_data_t;




typedef struct {



	uint8_t   signal_thresh_sigma;

	uint8_t   ambient_thresh_sigma;

	int32_t   min_ambient_thresh_events;

	int32_t   signal_total_events_limit;


	uint16_t  target_reflectance_for_dmax_calc[
			VL53LX_MAX_AMBIENT_DMAX_VALUES];

	uint16_t  max_effective_spads;




	uint16_t  dss_config__target_total_rate_mcps;

	uint8_t   dss_config__aperture_attenuation;


} VL53LX_hist_gen3_dmax_config_t;


#ifdef __cplusplus
}
#endif

#endif

