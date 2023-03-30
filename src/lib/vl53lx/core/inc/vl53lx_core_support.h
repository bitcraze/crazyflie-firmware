
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53LX_CORE_SUPPORT_H_
#define _VL53LX_CORE_SUPPORT_H_

#include "vl53lx_types.h"
#include "vl53lx_hist_structs.h"

#ifdef __cplusplus
extern "C" {
#endif




uint32_t VL53LX_calc_pll_period_us(
	uint16_t fast_osc_frequency);





uint32_t VL53LX_duration_maths(
	uint32_t  pll_period_us,
	uint32_t  vcsel_parm_pclks,
	uint32_t  window_vclks,
	uint32_t  periods_elapsed_mclks);



uint32_t VL53LX_events_per_spad_maths(
	int32_t   VL53LX_p_010,
	uint16_t  num_spads,
	uint32_t  duration);




uint32_t VL53LX_isqrt(
	uint32_t  num);




void VL53LX_hist_calc_zero_distance_phase(
	VL53LX_histogram_bin_data_t    *pdata);




void VL53LX_hist_estimate_ambient_from_thresholded_bins(
	int32_t                      ambient_threshold_sigma,
	VL53LX_histogram_bin_data_t *pdata);




void VL53LX_hist_remove_ambient_bins(
	VL53LX_histogram_bin_data_t    *pdata);




uint32_t VL53LX_calc_pll_period_mm(
	uint16_t fast_osc_frequency);




uint16_t VL53LX_rate_maths(
	int32_t   VL53LX_p_018,
	uint32_t  time_us);




uint16_t VL53LX_rate_per_spad_maths(
	uint32_t  frac_bits,
	uint32_t  peak_count_rate,
	uint16_t  num_spads,
	uint32_t  max_output_value);




int32_t VL53LX_range_maths(
	uint16_t  fast_osc_frequency,
	uint16_t  VL53LX_p_014,
	uint16_t  zero_distance_phase,
	uint8_t   fractional_bits,
	int32_t   gain_factor,
	int32_t   range_offset_mm);




uint8_t VL53LX_decode_vcsel_period(
	uint8_t vcsel_period_reg);



void VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
		VL53LX_xtalk_histogram_shape_t *pxtalk,
		VL53LX_histogram_bin_data_t    *phist);




void VL53LX_init_histogram_bin_data_struct(
	int32_t                      bin_value,
	uint16_t                     VL53LX_p_021,
	VL53LX_histogram_bin_data_t *pdata);




void VL53LX_decode_row_col(
	uint8_t   spad_number,
	uint8_t  *prow,
	uint8_t  *pcol);




void VL53LX_hist_find_min_max_bin_values(
	VL53LX_histogram_bin_data_t   *pdata);




void VL53LX_hist_estimate_ambient_from_ambient_bins(
	VL53LX_histogram_bin_data_t    *pdata);


#ifdef __cplusplus
}
#endif

#endif

