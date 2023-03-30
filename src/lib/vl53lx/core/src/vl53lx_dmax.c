
// SPDX-License-Identifier: BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX Protected and is dual licensed,
 either 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

 ******************************************************************************

 'STMicroelectronics Proprietary license'

 ******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 ******************************************************************************
 */




#include <vl53lx_platform_log.h>
#include <vl53lx_types.h>
#include "vl53lx_platform_user_defines.h"
#include "vl53lx_core_support.h"
#include "vl53lx_error_codes.h"

#include "vl53lx_dmax.h"


#define LOG_FUNCTION_START(fmt, ...)
#define LOG_FUNCTION_END(status, ...)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

#define trace_print(level, ...)


VL53LX_Error VL53LX_f_001(
	uint16_t                              target_reflectance,
	VL53LX_dmax_calibration_data_t	     *pcal,
	VL53LX_hist_gen3_dmax_config_t	     *pcfg,
	VL53LX_histogram_bin_data_t          *pbins,
	VL53LX_hist_gen3_dmax_private_data_t *pdata,
	int16_t                              *pambient_dmax_mm)
{



	VL53LX_Error status  = VL53LX_ERROR_NONE;

	uint32_t    pll_period_us       = 0;
	uint32_t    periods_elapsed     = 0;

	uint32_t    tmp32               = 0;
	uint64_t    tmp64               = 0;

	uint32_t    amb_thres_delta     = 0;

	LOG_FUNCTION_START("");



	pdata->VL53LX_p_004     = 0x0000;
	pdata->VL53LX_p_033 = 0x0000;
	pdata->VL53LX_p_034          = 0x0000;
	pdata->VL53LX_p_009    = 0x0000;
	pdata->VL53LX_p_028     = 0x0000;
	pdata->VL53LX_p_035 = 0x0000;
	pdata->VL53LX_p_036             = 0;
	pdata->VL53LX_p_022            = 0;

	*pambient_dmax_mm  = 0;


	if ((pbins->VL53LX_p_015        != 0) &&
		(pbins->total_periods_elapsed      != 0)) {



		pll_period_us   =
			VL53LX_calc_pll_period_us(pbins->VL53LX_p_015);



		periods_elapsed = pbins->total_periods_elapsed + 1;



		pdata->VL53LX_p_037  =
			VL53LX_duration_maths(
				pll_period_us,
				1<<4,
				VL53LX_RANGING_WINDOW_VCSEL_PERIODS,
				periods_elapsed);


		pdata->VL53LX_p_034 =
			VL53LX_rate_maths(
				pbins->VL53LX_p_028,
				pdata->VL53LX_p_037);



		pdata->VL53LX_p_033   =
			VL53LX_events_per_spad_maths(
				pbins->VL53LX_p_028,
				pbins->result__dss_actual_effective_spads,
				pdata->VL53LX_p_037);



		pdata->VL53LX_p_038 = pcfg->max_effective_spads;
		pdata->VL53LX_p_004  = pcfg->max_effective_spads;

		if (pdata->VL53LX_p_033 > 0) {
			tmp64   =
			(uint64_t)pcfg->dss_config__target_total_rate_mcps;
			tmp64  *= 1000;
			tmp64 <<= (11+1);
			tmp32 = pdata->VL53LX_p_033/2;
			tmp64 += (uint64_t)tmp32;
			tmp64 = do_division_u(tmp64,
				(uint64_t)pdata->VL53LX_p_033);

			if (tmp64 < (uint64_t)pcfg->max_effective_spads)
				pdata->VL53LX_p_004 = (uint16_t)tmp64;
		}
	}



	if ((pcal->ref__actual_effective_spads != 0) &&
		(pbins->VL53LX_p_015        != 0) &&
		(pcal->ref_reflectance_pc          != 0) &&
		(pbins->total_periods_elapsed      != 0)) {



		tmp64  = (uint64_t)pcal->ref__peak_signal_count_rate_mcps;
		tmp64 *= (1000 * 256);
		tmp32  = pcal->ref__actual_effective_spads/2;
		tmp64 += (uint64_t)tmp32;
		tmp64  = do_division_u(tmp64,
			(uint64_t)pcal->ref__actual_effective_spads);

		pdata->VL53LX_p_009   = (uint32_t)tmp64;
		pdata->VL53LX_p_009 <<= 4;



		tmp64   = (uint64_t)pdata->VL53LX_p_037;
		tmp64  *= (uint64_t)pdata->VL53LX_p_033;
		tmp64  *= (uint64_t)pdata->VL53LX_p_004;
		tmp64  += (1<<(11+7));
		tmp64 >>= (11+8);
		tmp64  +=  500;
		tmp64   = do_division_u(tmp64, 1000);


		if (tmp64 > 0x00FFFFFF)
			tmp64 = 0x00FFFFFF;

		pdata->VL53LX_p_028     = (uint32_t)tmp64;



		tmp64   = (uint64_t)pdata->VL53LX_p_037;
		tmp64  *= (uint64_t)pdata->VL53LX_p_009;
		tmp64  *= (uint64_t)pdata->VL53LX_p_004;
		tmp64  += (1<<(11+7));
		tmp64 >>= (11+8);



		tmp64  *= ((uint64_t)target_reflectance *
				   (uint64_t)pcal->coverglass_transmission);

		tmp64  += ((uint64_t)pcal->ref_reflectance_pc * 128);
		tmp64  = do_division_u(tmp64,
			((uint64_t)pcal->ref_reflectance_pc * 256));

		tmp64  +=  500;
		tmp64  = do_division_u(tmp64, 1000);


		if (tmp64 > 0x00FFFFFF)
			tmp64 = 0x00FFFFFF;

		pdata->VL53LX_p_035 = (uint32_t)tmp64;



		tmp32  = VL53LX_isqrt(pdata->VL53LX_p_028 << 8);
		tmp32 *= (uint32_t)pcfg->ambient_thresh_sigma;



		if (pdata->VL53LX_p_028 <
			(uint32_t)pcfg->min_ambient_thresh_events) {

			amb_thres_delta =
				pcfg->min_ambient_thresh_events -
				(uint32_t)pdata->VL53LX_p_028;


			amb_thres_delta <<= 8;

			if (tmp32 < amb_thres_delta)
				tmp32 = amb_thres_delta;
		}



		pdata->VL53LX_p_022 =
			(int16_t)VL53LX_f_002(
				tmp32,
				pdata->VL53LX_p_035,
				(uint32_t)pcal->ref__distance_mm,
				(uint32_t)pcfg->signal_thresh_sigma);



		tmp32  = (uint32_t)pdata->VL53LX_p_035;
		tmp32 *= (uint32_t)pbins->vcsel_width;
		tmp32 += (1 << 3);
		tmp32 /= (1 << 4);

		pdata->VL53LX_p_036 =
			(int16_t)VL53LX_f_002(
				256 * (uint32_t)pcfg->signal_total_events_limit,
				tmp32,
				(uint32_t)pcal->ref__distance_mm,
				(uint32_t)pcfg->signal_thresh_sigma);




		if (pdata->VL53LX_p_036 < pdata->VL53LX_p_022)
			*pambient_dmax_mm = pdata->VL53LX_p_036;
		else
			*pambient_dmax_mm = pdata->VL53LX_p_022;

	}

	LOG_FUNCTION_END(status);

	return status;

}


uint32_t VL53LX_f_002(
	uint32_t     events_threshold,
	uint32_t     ref_signal_events,
	uint32_t	 ref_distance_mm,
	uint32_t     signal_thresh_sigma)
{



	uint32_t    tmp32               = 0;
	uint32_t    range_mm            = 0;

	tmp32 = 4 * events_threshold;



	tmp32 += ((uint32_t)signal_thresh_sigma *
			  (uint32_t)signal_thresh_sigma);



	tmp32  = VL53LX_isqrt(tmp32);
	tmp32 += (uint32_t)signal_thresh_sigma;



	range_mm =
		(uint32_t)VL53LX_isqrt(ref_signal_events << 4);
	range_mm *= ref_distance_mm;
	if (tmp32 != 0) {
		range_mm += (tmp32);
		range_mm /= (2*tmp32);
	}

	return range_mm;

}

