
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
#include "vl53lx_core_support.h"
#include "vl53lx_error_codes.h"
#include "vl53lx_ll_def.h"

#include "vl53lx_hist_funcs.h"
#include "vl53lx_hist_core.h"
#include "vl53lx_hist_private_structs.h"
#include "vl53lx_dmax_private_structs.h"
#include "vl53lx_xtalk.h"
#include "vl53lx_hist_algos_gen3.h"
#include "vl53lx_hist_algos_gen4.h"
#include "vl53lx_dmax.h"



#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53LX_TRACE_MODULE_HISTOGRAM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53LX_TRACE_MODULE_HISTOGRAM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53LX_TRACE_MODULE_HISTOGRAM, \
	status, fmt, ##__VA_ARGS__)

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(VL53LX_TRACE_MODULE_HISTOGRAM, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)


VL53LX_Error VL53LX_hist_process_data(
	VL53LX_dmax_calibration_data_t     *pdmax_cal,
	VL53LX_hist_gen3_dmax_config_t     *pdmax_cfg,
	VL53LX_hist_post_process_config_t  *ppost_cfg,
	VL53LX_histogram_bin_data_t        *pbins_input,
	VL53LX_xtalk_histogram_data_t      *pxtalk_shape,
	uint8_t                            *pArea1,
	uint8_t                            *pArea2,
	VL53LX_range_results_t             *presults,
	uint8_t                            *HistMergeNumber)
{



	VL53LX_Error  status  = VL53LX_ERROR_NONE;

	VL53LX_hist_gen3_algo_private_data_t  *palgo_gen3 =
			(VL53LX_hist_gen3_algo_private_data_t *) pArea1;
	VL53LX_hist_gen4_algo_filtered_data_t *pfiltered4 =
			(VL53LX_hist_gen4_algo_filtered_data_t *) pArea2;

	VL53LX_hist_gen3_dmax_private_data_t   dmax_algo_gen3;
	VL53LX_hist_gen3_dmax_private_data_t  *pdmax_algo_gen3 =
						&dmax_algo_gen3;

	VL53LX_histogram_bin_data_t             bins_averaged;
	VL53LX_histogram_bin_data_t           *pbins_averaged = &bins_averaged;

	VL53LX_range_data_t                   *pdata;

	uint32_t xtalk_rate_kcps               = 0;
	uint32_t max_xtalk_rate_per_spad_kcps  = 0;
	uint8_t  xtalk_enable                  = 0;
	uint8_t  r                             = 0;
	uint8_t  t                             = 0;
	uint32_t XtalkDetectMaxSigma           = 0;


	int16_t  delta_mm                      = 0;


	LOG_FUNCTION_START("");



	VL53LX_f_031(
			pbins_input,
			pbins_averaged);



	VL53LX_init_histogram_bin_data_struct(
			0,
			pxtalk_shape->xtalk_shape.VL53LX_p_021,
			&(pxtalk_shape->xtalk_hist_removed));



	VL53LX_copy_xtalk_bin_data_to_histogram_data_struct(
			&(pxtalk_shape->xtalk_shape),
			&(pxtalk_shape->xtalk_hist_removed));



	if ((status == VL53LX_ERROR_NONE) &&
		(ppost_cfg->algo__crosstalk_compensation_enable > 0))
		status =
		VL53LX_f_032(
		ppost_cfg->algo__crosstalk_compensation_plane_offset_kcps,
		ppost_cfg->algo__crosstalk_compensation_x_plane_gradient_kcps,
		ppost_cfg->algo__crosstalk_compensation_y_plane_gradient_kcps,
		0,
		0,
		pbins_input->result__dss_actual_effective_spads,
		pbins_input->roi_config__user_roi_centre_spad,
		pbins_input->roi_config__user_roi_requested_global_xy_size,
		&(xtalk_rate_kcps));



	if ((status == VL53LX_ERROR_NONE) &&
		(ppost_cfg->algo__crosstalk_compensation_enable > 0))
		status =
			VL53LX_f_033(
			  pbins_averaged,
			  &(pxtalk_shape->xtalk_shape),
			  xtalk_rate_kcps,
			  &(pxtalk_shape->xtalk_hist_removed));





	presults->xmonitor.total_periods_elapsed =
		pbins_averaged->total_periods_elapsed;
	presults->xmonitor.VL53LX_p_004 =
		pbins_averaged->result__dss_actual_effective_spads;

	presults->xmonitor.peak_signal_count_rate_mcps = 0;
	presults->xmonitor.VL53LX_p_009     = 0;

	presults->xmonitor.range_id     = 0;
	presults->xmonitor.range_status = VL53LX_DEVICEERROR_NOUPDATE;



	xtalk_enable = 0;
	if (ppost_cfg->algo__crosstalk_compensation_enable > 0)
		xtalk_enable = 1;



	for (r = 0 ; r <= xtalk_enable ; r++) {


		ppost_cfg->algo__crosstalk_compensation_enable = r;



		status =
		VL53LX_f_025(
			pdmax_cal,
			pdmax_cfg,
			ppost_cfg,
			pbins_averaged,
			&(pxtalk_shape->xtalk_hist_removed),
			palgo_gen3,
			pfiltered4,
			pdmax_algo_gen3,
			presults,
			*HistMergeNumber);


		if (!(status == VL53LX_ERROR_NONE && r == 0))
			continue;



		if (presults->active_results == 0) {
			pdata = &(presults->VL53LX_p_003[0]);
			pdata->ambient_count_rate_mcps =
				pdmax_algo_gen3->VL53LX_p_034;
			pdata->VL53LX_p_004 =
				pdmax_algo_gen3->VL53LX_p_004;
		}



		max_xtalk_rate_per_spad_kcps = (uint32_t)(
		ppost_cfg->algo__crosstalk_detect_max_valid_rate_kcps);
		max_xtalk_rate_per_spad_kcps *= (uint32_t)(*HistMergeNumber);
		max_xtalk_rate_per_spad_kcps <<= 4;

		for (t = 0 ; t < presults->active_results ; t++) {

			pdata = &(presults->VL53LX_p_003[t]);



			if (pdata->max_range_mm > pdata->min_range_mm)
				delta_mm =
					pdata->max_range_mm -
					pdata->min_range_mm;
			else
				delta_mm =
					pdata->min_range_mm -
					pdata->max_range_mm;

			XtalkDetectMaxSigma =
				ppost_cfg->algo__crosstalk_detect_max_sigma_mm;
			XtalkDetectMaxSigma *= (uint32_t)(*HistMergeNumber);
			XtalkDetectMaxSigma <<= 5;
			if (pdata->median_range_mm  >
			ppost_cfg->algo__crosstalk_detect_min_valid_range_mm &&
			pdata->median_range_mm  <
			ppost_cfg->algo__crosstalk_detect_max_valid_range_mm &&
			pdata->VL53LX_p_009 <
			max_xtalk_rate_per_spad_kcps &&
			pdata->VL53LX_p_002 < XtalkDetectMaxSigma &&
			delta_mm <
			ppost_cfg->algo__crosstalk_detect_min_max_tolerance) {



				memcpy(
					&(presults->xmonitor),
					pdata,
					sizeof(VL53LX_range_data_t));

			}
		}

	}



	ppost_cfg->algo__crosstalk_compensation_enable = xtalk_enable;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_hist_ambient_dmax(
	uint16_t                            target_reflectance,
	VL53LX_dmax_calibration_data_t     *pdmax_cal,
	VL53LX_hist_gen3_dmax_config_t     *pdmax_cfg,
	VL53LX_histogram_bin_data_t        *pbins,
	int16_t                            *pambient_dmax_mm)
{



	VL53LX_Error  status  = VL53LX_ERROR_NONE;

	VL53LX_hist_gen3_dmax_private_data_t   dmax_algo;
	VL53LX_hist_gen3_dmax_private_data_t  *pdmax_algo = &dmax_algo;

	LOG_FUNCTION_START("");

	status =
		VL53LX_f_001(
			target_reflectance,
			pdmax_cal,
			pdmax_cfg,
			pbins,
			pdmax_algo,
			pambient_dmax_mm);

	LOG_FUNCTION_END(status);

	return status;
}

