
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include <vl53lx_platform_log.h>
#include <vl53lx_platform_user_data.h>
#include <vl53lx_platform_user_defines.h>
#include "vl53lx_ll_def.h"
#include "vl53lx_ll_device.h"
#include "vl53lx_core_support.h"



#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53LX_TRACE_MODULE_CORE, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53LX_TRACE_MODULE_CORE, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53LX_TRACE_MODULE_CORE, \
		status, fmt, ##__VA_ARGS__)

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_CORE, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)


uint32_t VL53LX_calc_pll_period_us(
	uint16_t  fast_osc_frequency)
{


	uint32_t  pll_period_us        = 0;

	LOG_FUNCTION_START("");

	if (fast_osc_frequency > 0)
		pll_period_us = (0x01 << 30) / fast_osc_frequency;



	LOG_FUNCTION_END(0);

	return pll_period_us;
}


uint32_t  VL53LX_duration_maths(
	uint32_t  pll_period_us,
	uint32_t  vcsel_parm_pclks,
	uint32_t  window_vclks,
	uint32_t  elapsed_mclks)
{


	uint64_t  tmp_long_int = 0;
	uint32_t  duration_us  = 0;


	duration_us = window_vclks * pll_period_us;


	duration_us = duration_us >> 12;


	tmp_long_int = (uint64_t)duration_us;


	duration_us = elapsed_mclks * vcsel_parm_pclks;


	duration_us = duration_us >> 4;


	tmp_long_int = tmp_long_int * (uint64_t)duration_us;


	tmp_long_int = tmp_long_int >> 12;


	if (tmp_long_int > 0xFFFFFFFF)
		tmp_long_int = 0xFFFFFFFF;

	duration_us  = (uint32_t)tmp_long_int;

	return duration_us;
}


uint32_t VL53LX_events_per_spad_maths(
	int32_t   VL53LX_p_010,
	uint16_t  num_spads,
	uint32_t  duration)
{
	uint64_t total_hist_counts  = 0;
	uint64_t xtalk_per_spad     = 0;
	uint32_t rate_per_spad_kcps = 0;






	uint64_t dividend = ((uint64_t)VL53LX_p_010
			* 1000 * 256);

	if (num_spads != 0)
		total_hist_counts = do_division_u(
				dividend, (uint64_t)num_spads);



	if (duration > 0) {


		uint64_t dividend = (((uint64_t)(total_hist_counts << 11))
			+ ((uint64_t)duration / 2));

		xtalk_per_spad = do_division_u(dividend, (uint64_t)duration);
	} else {
		xtalk_per_spad =   (uint64_t)(total_hist_counts << 11);
	}

	rate_per_spad_kcps = (uint32_t)xtalk_per_spad;

	return rate_per_spad_kcps;
}


uint32_t VL53LX_isqrt(uint32_t num)
{



	uint32_t  res = 0;
	uint32_t  bit = 1 << 30;


	while (bit > num)
		bit >>= 2;

	while (bit != 0) {
		if (num >= res + bit)  {
			num -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}

	return res;
}


void  VL53LX_hist_calc_zero_distance_phase(
	VL53LX_histogram_bin_data_t   *pdata)
{


	uint32_t  period        = 0;
	uint32_t  VL53LX_p_014         = 0;

	LOG_FUNCTION_START("");

	period = 2048 *
		(uint32_t)VL53LX_decode_vcsel_period(pdata->VL53LX_p_005);

	VL53LX_p_014  = period;
	VL53LX_p_014 += (uint32_t)pdata->phasecal_result__reference_phase;
	VL53LX_p_014 += (2048 * (uint32_t)pdata->phasecal_result__vcsel_start);
	VL53LX_p_014 -= (2048 * (uint32_t)pdata->cal_config__vcsel_start);

	if (period != 0)
		VL53LX_p_014  = VL53LX_p_014 % period;
	else
		VL53LX_p_014 = 0;

	pdata->zero_distance_phase = (uint16_t)VL53LX_p_014;

	LOG_FUNCTION_END(0);
}


void  VL53LX_hist_estimate_ambient_from_thresholded_bins(
	int32_t                        ambient_threshold_sigma,
	VL53LX_histogram_bin_data_t   *pdata)
{


	uint8_t  bin                      = 0;
	int32_t  VL53LX_p_031 = 0;

	LOG_FUNCTION_START("");



	VL53LX_hist_find_min_max_bin_values(pdata);



	VL53LX_p_031  =
		(int32_t)VL53LX_isqrt((uint32_t)pdata->min_bin_value);
	VL53LX_p_031 *= ambient_threshold_sigma;
	VL53LX_p_031 += 0x07;
	VL53LX_p_031  = VL53LX_p_031 >> 4;
	VL53LX_p_031 += pdata->min_bin_value;



	pdata->number_of_ambient_samples = 0;
	pdata->ambient_events_sum        = 0;

	for (bin = 0; bin < pdata->VL53LX_p_021; bin++)
		if (pdata->bin_data[bin] < VL53LX_p_031) {
			pdata->ambient_events_sum += pdata->bin_data[bin];
			pdata->number_of_ambient_samples++;
		}



	if (pdata->number_of_ambient_samples > 0) {
		pdata->VL53LX_p_028 =
			pdata->ambient_events_sum;
		pdata->VL53LX_p_028 +=
			((int32_t)pdata->number_of_ambient_samples/2);
		pdata->VL53LX_p_028 /=
			(int32_t)pdata->number_of_ambient_samples;
	}

	LOG_FUNCTION_END(0);
}


void  VL53LX_hist_remove_ambient_bins(
	VL53LX_histogram_bin_data_t   *pdata)
{



	uint8_t bin = 0;
	uint8_t lc = 0;
	uint8_t i = 0;



	if ((pdata->bin_seq[0] & 0x07) == 0x07) {

		i = 0;
		for (lc = 0; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH; lc++) {
			if ((pdata->bin_seq[lc] & 0x07) != 0x07) {
				pdata->bin_seq[i] = pdata->bin_seq[lc];
				pdata->bin_rep[i] = pdata->bin_rep[lc];
				i++;
			}
		}



		for (lc = i; lc < VL53LX_MAX_BIN_SEQUENCE_LENGTH; lc++) {
			pdata->bin_seq[lc] = VL53LX_MAX_BIN_SEQUENCE_CODE + 1;
			pdata->bin_rep[lc] = 0;
		}
	}

	if (pdata->number_of_ambient_bins > 0) {


		for (bin = pdata->number_of_ambient_bins;
				bin < pdata->VL53LX_p_020; bin++) {
			pdata->bin_data[bin-pdata->number_of_ambient_bins] =
				pdata->bin_data[bin];
		}


		pdata->VL53LX_p_021 =
				pdata->VL53LX_p_021 -
				pdata->number_of_ambient_bins;
		pdata->number_of_ambient_bins = 0;
	}
}


uint32_t VL53LX_calc_pll_period_mm(
	uint16_t fast_osc_frequency)
{


	uint32_t pll_period_us = 0;
	uint32_t pll_period_mm = 0;

	LOG_FUNCTION_START("");



	pll_period_us  = VL53LX_calc_pll_period_us(fast_osc_frequency);




	pll_period_mm =
			VL53LX_SPEED_OF_LIGHT_IN_AIR_DIV_8 *
			(pll_period_us >> 2);


	pll_period_mm = (pll_period_mm + (0x01<<15)) >> 16;

	LOG_FUNCTION_END(0);

	return pll_period_mm;
}


uint16_t VL53LX_rate_maths(
	int32_t   VL53LX_p_018,
	uint32_t  time_us)
{


	uint32_t  tmp_int   = 0;
	uint32_t  frac_bits = 7;
	uint16_t  rate_mcps = 0;



	if (VL53LX_p_018 > VL53LX_SPAD_TOTAL_COUNT_MAX)
		tmp_int = VL53LX_SPAD_TOTAL_COUNT_MAX;
	else if (VL53LX_p_018 > 0)
		tmp_int = (uint32_t)VL53LX_p_018;




	if (VL53LX_p_018 > VL53LX_SPAD_TOTAL_COUNT_RES_THRES)
		frac_bits = 3;
	else
		frac_bits = 7;


	if (time_us > 0)
		tmp_int = ((tmp_int << frac_bits) + (time_us / 2)) / time_us;


	if (VL53LX_p_018 > VL53LX_SPAD_TOTAL_COUNT_RES_THRES)
		tmp_int = tmp_int << 4;



	if (tmp_int > 0xFFFF)
		tmp_int = 0xFFFF;

	rate_mcps =  (uint16_t)tmp_int;

	return rate_mcps;
}


uint16_t VL53LX_rate_per_spad_maths(
	uint32_t  frac_bits,
	uint32_t  peak_count_rate,
	uint16_t  num_spads,
	uint32_t  max_output_value)
{

	uint32_t  tmp_int   = 0;


	uint16_t  rate_per_spad = 0;





	if (num_spads > 0) {
		tmp_int = (peak_count_rate << 8) << frac_bits;
		tmp_int = (tmp_int +
			((uint32_t)num_spads / 2)) /
				(uint32_t)num_spads;
	} else {
		tmp_int = ((peak_count_rate) << frac_bits);
	}



	if (tmp_int > max_output_value)
		tmp_int = max_output_value;

	rate_per_spad = (uint16_t)tmp_int;

	return rate_per_spad;
}


int32_t VL53LX_range_maths(
	uint16_t  fast_osc_frequency,
	uint16_t  VL53LX_p_014,
	uint16_t  zero_distance_phase,
	uint8_t   fractional_bits,
	int32_t   gain_factor,
	int32_t   range_offset_mm)
{


	uint32_t    pll_period_us = 0;
	int64_t     tmp_long_int  = 0;
	int32_t     range_mm      = 0;
	int32_t     range_mm_10   = 0;



	pll_period_us  = VL53LX_calc_pll_period_us(fast_osc_frequency);



	tmp_long_int = (int64_t)VL53LX_p_014 - (int64_t)zero_distance_phase;



	tmp_long_int =  tmp_long_int * (int64_t)pll_period_us;



	tmp_long_int =  tmp_long_int / (0x01 << 9);



	tmp_long_int =  tmp_long_int * VL53LX_SPEED_OF_LIGHT_IN_AIR_DIV_8;



	tmp_long_int =  tmp_long_int / (0x01 << 22);


	range_mm  = (int32_t)tmp_long_int + range_offset_mm;


	range_mm *= gain_factor;
	range_mm += 0x0400;
	range_mm /= 0x0800;


	if (fractional_bits == 0) {
		range_mm_10 = range_mm * 10;
		range_mm_10 = range_mm_10 / (0x01 << 2);
		if ((range_mm_10 % 10) < 5)
			range_mm = (int16_t)(range_mm_10 / 10);
		else
			range_mm = (int16_t)(range_mm_10 / 10 + 1);
	} else if (fractional_bits == 1)
		range_mm = range_mm / (0x01 << 1);

	return range_mm;
}


uint8_t VL53LX_decode_vcsel_period(uint8_t vcsel_period_reg)
{


	uint8_t VL53LX_p_030 = 0;

	VL53LX_p_030 = (vcsel_period_reg + 1) << 1;

	return VL53LX_p_030;
}


void VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
		VL53LX_xtalk_histogram_shape_t *pxtalk,
		VL53LX_histogram_bin_data_t    *phist)
{


	phist->cal_config__vcsel_start =
			pxtalk->cal_config__vcsel_start;
	phist->VL53LX_p_015 =
			pxtalk->VL53LX_p_015;
	phist->VL53LX_p_019 =
			pxtalk->VL53LX_p_019;

	phist->phasecal_result__reference_phase   =
			pxtalk->phasecal_result__reference_phase;
	phist->phasecal_result__vcsel_start       =
			pxtalk->phasecal_result__vcsel_start;

	phist->vcsel_width =
			pxtalk->vcsel_width;
	phist->zero_distance_phase =
			pxtalk->zero_distance_phase;

	phist->zone_id      = pxtalk->zone_id;
	phist->VL53LX_p_020  = pxtalk->VL53LX_p_020;
	phist->time_stamp   = pxtalk->time_stamp;
}


void VL53LX_init_histogram_bin_data_struct(
	int32_t                      bin_value,
	uint16_t                     VL53LX_p_021,
	VL53LX_histogram_bin_data_t *pdata)
{



	uint16_t          i = 0;

	pdata->cfg_device_state          = VL53LX_DEVICESTATE_SW_STANDBY;
	pdata->rd_device_state           = VL53LX_DEVICESTATE_SW_STANDBY;

	pdata->zone_id                   = 0;
	pdata->time_stamp                = 0;

	pdata->VL53LX_p_019                 = 0;
	pdata->VL53LX_p_020               = VL53LX_HISTOGRAM_BUFFER_SIZE;
	pdata->VL53LX_p_021            = (uint8_t)VL53LX_p_021;
	pdata->number_of_ambient_bins    = 0;

	pdata->result__interrupt_status           = 0;
	pdata->result__range_status               = 0;
	pdata->result__report_status              = 0;
	pdata->result__stream_count               = 0;

	pdata->result__dss_actual_effective_spads = 0;
	pdata->phasecal_result__reference_phase   = 0;
	pdata->phasecal_result__vcsel_start       = 0;
	pdata->cal_config__vcsel_start            = 0;

	pdata->vcsel_width                        = 0;
	pdata->VL53LX_p_005                       = 0;
	pdata->VL53LX_p_015                = 0;
	pdata->total_periods_elapsed              = 0;

	pdata->min_bin_value                      = 0;
	pdata->max_bin_value                      = 0;

	pdata->zero_distance_phase                = 0;
	pdata->number_of_ambient_samples          = 0;
	pdata->ambient_events_sum                 = 0;
	pdata->VL53LX_p_028             = 0;

	for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++)
		pdata->bin_seq[i] = (uint8_t)i;

	for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++)
		pdata->bin_rep[i] = 1;


	for (i = 0; i < VL53LX_HISTOGRAM_BUFFER_SIZE; i++)
		if (i < VL53LX_p_021)
			pdata->bin_data[i] = bin_value;
		else
			pdata->bin_data[i] = 0;


}


void VL53LX_decode_row_col(
	uint8_t  spad_number,
	uint8_t  *prow,
	uint8_t  *pcol)
{



	if (spad_number > 127) {
		*prow = 8 + ((255-spad_number) & 0x07);
		*pcol = (spad_number-128) >> 3;
	} else {
		*prow = spad_number & 0x07;
		*pcol = (127-spad_number) >> 3;
	}
}


void  VL53LX_hist_find_min_max_bin_values(
	VL53LX_histogram_bin_data_t   *pdata)
{


	uint8_t  bin            = 0;

	LOG_FUNCTION_START("");

	for (bin = 0; bin < pdata->VL53LX_p_021; bin++) {

		if (bin == 0 || pdata->min_bin_value >= pdata->bin_data[bin])
			pdata->min_bin_value = pdata->bin_data[bin];

		if (bin == 0 || pdata->max_bin_value <= pdata->bin_data[bin])
			pdata->max_bin_value = pdata->bin_data[bin];

	}

	LOG_FUNCTION_END(0);

}


void  VL53LX_hist_estimate_ambient_from_ambient_bins(
	VL53LX_histogram_bin_data_t   *pdata)
{


	uint8_t  bin            = 0;

	LOG_FUNCTION_START("");

	if (pdata->number_of_ambient_bins > 0) {

		pdata->number_of_ambient_samples =
			pdata->number_of_ambient_bins;



		pdata->ambient_events_sum = 0;
		for (bin = 0; bin < pdata->number_of_ambient_bins; bin++)
			pdata->ambient_events_sum += pdata->bin_data[bin];

		pdata->VL53LX_p_028 = pdata->ambient_events_sum;
		pdata->VL53LX_p_028 +=
				((int32_t)pdata->number_of_ambient_bins / 2);
		pdata->VL53LX_p_028 /=
			(int32_t)pdata->number_of_ambient_bins;

	}

	LOG_FUNCTION_END(0);
}


