
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#include "vl53lx_platform.h"
#include "vl53lx_platform_ipp.h"
#include "vl53lx_ll_def.h"
#include "vl53lx_ll_device.h"
#include "vl53lx_register_map.h"
#include "vl53lx_register_funcs.h"
#include "vl53lx_register_settings.h"
#include "vl53lx_hist_map.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_core.h"
#include "vl53lx_wait.h"
#include "vl53lx_api_preset_modes.h"
#include "vl53lx_silicon_core.h"
#include "vl53lx_api_core.h"
#include "vl53lx_api_calibration.h"

#ifdef VL53LX_LOG_ENABLE
  #include "vl53lx_api_debug.h"
#endif


#define LOG_FUNCTION_START(fmt, ...) 
#define LOG_FUNCTION_END(status, ...) 
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

#define trace_print(level, ...)


VL53LX_Error VL53LX_run_ref_spad_char(
	VL53LX_DEV        Dev,
	VL53LX_Error     *pcal_status)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	uint8_t comms_buffer[6];

	VL53LX_refspadchar_config_t *prefspadchar  = &(pdev->refspadchar);

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_enable_powerforce(Dev);



	if (status == VL53LX_ERROR_NONE)
		status =
		VL53LX_set_ref_spad_char_config(
			Dev,
			prefspadchar->VL53LX_p_005,
			prefspadchar->timeout_us,
			prefspadchar->target_count_rate_mcps,
			prefspadchar->max_count_rate_limit_mcps,
			prefspadchar->min_count_rate_limit_mcps,
			pdev->stat_nvm.osc_measured__fast_osc__frequency);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_run_device_test(
					Dev,
					prefspadchar->device_test_mode);



	if (status == VL53LX_ERROR_NONE)
		status =
		VL53LX_ReadMulti(
			Dev,
			VL53LX_REF_SPAD_CHAR_RESULT__NUM_ACTUAL_REF_SPADS,
			comms_buffer,
			2);

	if (status == VL53LX_ERROR_NONE) {
		pdev->dbg_results.ref_spad_char_result__num_actual_ref_spads =
				comms_buffer[0];
		pdev->dbg_results.ref_spad_char_result__ref_location =
				comms_buffer[1];
	}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WriteMulti(
				Dev,
				VL53LX_REF_SPAD_MAN__NUM_REQUESTED_REF_SPADS,
				comms_buffer,
				2);

	if (status == VL53LX_ERROR_NONE) {
		pdev->customer.ref_spad_man__num_requested_ref_spads =
				comms_buffer[0];
		pdev->customer.ref_spad_man__ref_location =
				comms_buffer[1];
	}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_ReadMulti(
				Dev,
				VL53LX_RESULT__SPARE_0_SD1,
				comms_buffer,
				6);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WriteMulti(
				Dev,
				VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0,
				comms_buffer,
				6);

	if (status == VL53LX_ERROR_NONE) {
		pdev->customer.global_config__spad_enables_ref_0 =
				comms_buffer[0];
		pdev->customer.global_config__spad_enables_ref_1 =
				comms_buffer[1];
		pdev->customer.global_config__spad_enables_ref_2 =
				comms_buffer[2];
		pdev->customer.global_config__spad_enables_ref_3 =
				comms_buffer[3];
		pdev->customer.global_config__spad_enables_ref_4 =
				comms_buffer[4];
		pdev->customer.global_config__spad_enables_ref_5 =
				comms_buffer[5];
	}

#ifdef VL53LX_LOG_ENABLE

	if (status == VL53LX_ERROR_NONE)
		VL53LX_print_customer_nvm_managed(
			&(pdev->customer),
			"run_ref_spad_char():pdev->lldata.customer.",
			VL53LX_TRACE_MODULE_REF_SPAD_CHAR);
#endif

	if (status == VL53LX_ERROR_NONE) {

		switch (pdev->sys_results.result__range_status) {

		case VL53LX_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS:
			status = VL53LX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS;
			break;

		case VL53LX_DEVICEERROR_REFSPADCHARMORETHANTARGET:
			status = VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH;
			break;

		case VL53LX_DEVICEERROR_REFSPADCHARLESSTHANTARGET:
			status = VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW;
			break;
		}
	}



	*pcal_status = status;



	IGNORE_STATUS(
		IGNORE_REF_SPAD_CHAR_NOT_ENOUGH_SPADS,
		VL53LX_WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS,
		status);

	IGNORE_STATUS(
		IGNORE_REF_SPAD_CHAR_RATE_TOO_HIGH,
		VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH,
		status);

	IGNORE_STATUS(
		IGNORE_REF_SPAD_CHAR_RATE_TOO_LOW,
		VL53LX_WARNING_REF_SPAD_CHAR_RATE_TOO_LOW,
		status);


	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_and_avg_xtalk_samples(
		VL53LX_DEV	                  Dev,
		uint8_t                       num_of_samples,
		uint8_t                       measurement_mode,
		int16_t                       xtalk_filter_thresh_max_mm,
		int16_t                       xtalk_filter_thresh_min_mm,
		uint16_t                      xtalk_max_valid_rate_kcps,
		uint8_t                       xtalk_result_id,
		uint8_t                       xtalk_histo_id,
		VL53LX_xtalk_range_results_t *pXR,
		VL53LX_histogram_bin_data_t  *psum_histo,
		VL53LX_histogram_bin_data_t  *pavg_histo)
{



	VL53LX_Error status        = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
		VL53LXDevStructGetLLDriverHandle(Dev);

#ifdef VL53LX_LOG_ENABLE
	VL53LX_LLDriverResults_t *pres =
		VL53LXDevStructGetLLResultsHandle(Dev);
#endif

	VL53LX_range_results_t      *prs =
			(VL53LX_range_results_t *) pdev->wArea1;

	VL53LX_range_data_t         *prange_data;
	VL53LX_xtalk_range_data_t   *pxtalk_range_data;

	uint8_t i                = 0;
	uint8_t j                = 0;
	uint8_t zone_id          = 0;
	uint8_t final_zone       = pdev->zone_cfg.active_zones+1;
	uint8_t valid_result;

	uint8_t smudge_corr_en   = 0;




	smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;

	status = VL53LX_dynamic_xtalk_correction_disable(Dev);


	VL53LX_load_patch(Dev);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_init_and_start_range(
				Dev,
				measurement_mode,
				VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);


	for (i = 0; i <= (final_zone*num_of_samples); i++) {



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_wait_for_range_completion(Dev);



		if (status == VL53LX_ERROR_NONE)
			status =
				VL53LX_get_device_results(
					Dev,
					VL53LX_DEVICERESULTSLEVEL_FULL,
					prs);



		if (status == VL53LX_ERROR_NONE &&
			pdev->ll_state.rd_device_state !=
			VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {

			zone_id = pdev->ll_state.rd_zone_id + xtalk_result_id;
			prange_data       = &(prs->VL53LX_p_003[0]);


			if (prs->active_results > 1) {
				for (j = 1;
				j < prs->active_results; j++) {
					if (prs->VL53LX_p_003[j].median_range_mm
						<
						prange_data->median_range_mm)
						prange_data =
						&(prs->VL53LX_p_003[j]);

				}
			}

			pxtalk_range_data = &(pXR->VL53LX_p_003[zone_id]);



			if ((prs->active_results > 0) &&
				(prange_data->median_range_mm <
						xtalk_filter_thresh_max_mm) &&
				(prange_data->median_range_mm >
						xtalk_filter_thresh_min_mm) &&
				(prange_data->VL53LX_p_009 <
				(uint32_t)(xtalk_max_valid_rate_kcps * 16)))
				valid_result = 1;
			else
				valid_result = 0;

			if (valid_result == 1) {

				pxtalk_range_data->no_of_samples++;

				pxtalk_range_data->rate_per_spad_kcps_sum +=
					prange_data->VL53LX_p_009;

				pxtalk_range_data->signal_total_events_sum +=
					prange_data->VL53LX_p_010;

				pxtalk_range_data->sigma_mm_sum +=
					(uint32_t)prange_data->VL53LX_p_002;



				pxtalk_range_data->median_phase_sum +=
					(uint32_t)prange_data->VL53LX_p_011;




			}

			if ((valid_result == 1) && (zone_id >= 4)) {
				status = VL53LX_sum_histogram_data(
						&(pdev->hist_data),
						psum_histo);



				if (prange_data->VL53LX_p_012 <
					pXR->central_histogram__window_start)
					pXR->central_histogram__window_start =
					prange_data->VL53LX_p_012;


				if (prange_data->VL53LX_p_013 >
					pXR->central_histogram__window_end)
					pXR->central_histogram__window_end =
						prange_data->VL53LX_p_013;

			}

		}



#ifdef VL53LX_LOG_ENABLE
		if (status == VL53LX_ERROR_NONE) {
			VL53LX_print_range_results(
					&(pres->range_results),
					"pres->range_results.",
					VL53LX_TRACE_MODULE_CORE);
		}
#endif



		if (status == VL53LX_ERROR_NONE)
			status =
				VL53LX_clear_interrupt_and_enable_next_range(
					Dev,
					measurement_mode);


	}




	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_stop_range(Dev);

	VL53LX_unload_patch(Dev);



	for (i = 0; i < (pdev->zone_cfg.active_zones+1); i++) {

		pxtalk_range_data = &(pXR->VL53LX_p_003[i+xtalk_result_id]);

		if (pxtalk_range_data->no_of_samples > 0) {
			pxtalk_range_data->rate_per_spad_kcps_avg =
			pxtalk_range_data->rate_per_spad_kcps_sum /
			(uint32_t)pxtalk_range_data->no_of_samples;

			pxtalk_range_data->signal_total_events_avg =
			pxtalk_range_data->signal_total_events_sum /
			(int32_t)pxtalk_range_data->no_of_samples;

			pxtalk_range_data->sigma_mm_avg =
			pxtalk_range_data->sigma_mm_sum /
			(uint32_t)pxtalk_range_data->no_of_samples;



			pxtalk_range_data->median_phase_avg =
				pxtalk_range_data->median_phase_sum /
				(uint32_t)pxtalk_range_data->no_of_samples;



		} else {
			pxtalk_range_data->rate_per_spad_kcps_avg =
				pxtalk_range_data->rate_per_spad_kcps_sum;
			pxtalk_range_data->signal_total_events_avg =
				pxtalk_range_data->signal_total_events_sum;
			pxtalk_range_data->sigma_mm_avg =
				pxtalk_range_data->sigma_mm_sum;



			pxtalk_range_data->median_phase_avg =
					pxtalk_range_data->median_phase_sum;


		}
	}



	memcpy(pavg_histo, &(pdev->hist_data),
			sizeof(VL53LX_histogram_bin_data_t));



	if (status == VL53LX_ERROR_NONE) {

		pxtalk_range_data = &(pXR->VL53LX_p_003[xtalk_histo_id]);

		status = VL53LX_avg_histogram_data(
			pxtalk_range_data->no_of_samples,
			psum_histo,
			pavg_histo);
	}




	if (status == VL53LX_ERROR_NONE) {
		if (smudge_corr_en == 1)
			status = VL53LX_dynamic_xtalk_correction_enable(Dev);
	}


	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_run_phasecal_average(
	VL53LX_DEV	            Dev,
	uint8_t                 measurement_mode,
	uint8_t                 phasecal_result__vcsel_start,
	uint16_t                phasecal_num_of_samples,
	VL53LX_range_results_t *prange_results,
	uint16_t               *pphasecal_result__reference_phase,
	uint16_t               *pzero_distance_phase)
{


	VL53LX_Error status        = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
		VL53LXDevStructGetLLDriverHandle(Dev);

	uint16_t  i                                = 0;
	uint16_t  m                                = 0;
	uint32_t  samples                          = 0;

	uint32_t  period                           = 0;
	uint32_t  VL53LX_p_014                            = 0;
	uint32_t  phasecal_result__reference_phase = 0;
	uint32_t  zero_distance_phase              = 0;


	VL53LX_load_patch(Dev);

	for (m = 0; m < phasecal_num_of_samples; m++) {



		if (status == VL53LX_ERROR_NONE)
			status =
			VL53LX_init_and_start_range(
				Dev,
				measurement_mode,
				VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);

		for (i = 0; i <= 1; i++) {



			if (status == VL53LX_ERROR_NONE)
				status =
					VL53LX_wait_for_range_completion(Dev);



			if (status == VL53LX_ERROR_NONE)
				status =
					VL53LX_get_device_results(
						Dev,
						VL53LX_DEVICERESULTSLEVEL_FULL,
						prange_results);


			if (status == VL53LX_ERROR_NONE)
				status =
				VL53LX_clear_interrupt_and_enable_next_range(
					Dev,
					measurement_mode);
		}



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_stop_range(Dev);



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_WaitUs(Dev, 1000);



		if (status == VL53LX_ERROR_NONE) {

			samples++;


			period = 2048 *
				(uint32_t)VL53LX_decode_vcsel_period(
					pdev->hist_data.VL53LX_p_005);

			VL53LX_p_014  = period;
			VL53LX_p_014 += (uint32_t)(
			pdev->hist_data.phasecal_result__reference_phase);
			VL53LX_p_014 +=
				(2048 *
				(uint32_t)phasecal_result__vcsel_start);
			VL53LX_p_014 -= (2048 *
			(uint32_t)pdev->hist_data.cal_config__vcsel_start);

			if (period != 0) {
				VL53LX_p_014  = VL53LX_p_014 % period;
			}
			else {
				status =
				VL53LX_ERROR_DIVISION_BY_ZERO;
				VL53LX_p_014 = 0;
			}

			phasecal_result__reference_phase += (uint32_t)(
			pdev->hist_data.phasecal_result__reference_phase);

			zero_distance_phase += (uint32_t)VL53LX_p_014;
		}
	}
	VL53LX_unload_patch(Dev);



	if (status == VL53LX_ERROR_NONE && samples > 0) {

		phasecal_result__reference_phase += (samples >> 1);
		phasecal_result__reference_phase /= samples;

		zero_distance_phase += (samples >> 1);
		zero_distance_phase /= samples;

		*pphasecal_result__reference_phase =
			(uint16_t)phasecal_result__reference_phase;
		*pzero_distance_phase =
			(uint16_t)zero_distance_phase;
	}

	return status;
}


VL53LX_Error VL53LX_run_device_test(
	VL53LX_DEV             Dev,
	VL53LX_DeviceTestMode  device_test_mode)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	uint8_t      comms_buffer[2];
	uint8_t      gpio_hv_mux__ctrl = 0;

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_RdByte(
				Dev,
				VL53LX_GPIO_HV_MUX__CTRL,
				&gpio_hv_mux__ctrl);

	if (status == VL53LX_ERROR_NONE)
		pdev->stat_cfg.gpio_hv_mux__ctrl = gpio_hv_mux__ctrl;


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_start_test(
					Dev,
					device_test_mode);


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_wait_for_test_completion(Dev);


	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_ReadMulti(
				Dev,
				VL53LX_RESULT__RANGE_STATUS,
				comms_buffer,
				2);

	if (status == VL53LX_ERROR_NONE) {
		pdev->sys_results.result__range_status  = comms_buffer[0];
		pdev->sys_results.result__report_status = comms_buffer[1];
	}



	pdev->sys_results.result__range_status &=
		VL53LX_RANGE_STATUS__RANGE_STATUS_MASK;

	if (status == VL53LX_ERROR_NONE) {
		trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"    Device Test Complete:\n\t%-32s = %3u\n\t%-32s = %3u\n",
		"result__range_status",
		pdev->sys_results.result__range_status,
		"result__report_status",
		pdev->sys_results.result__report_status);


		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_clear_interrupt(Dev);
	}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_start_test(
				Dev,
				0x00);

	LOG_FUNCTION_END(status);

	return status;
}


void VL53LX_hist_xtalk_extract_data_init(
	VL53LX_hist_xtalk_extract_data_t *pxtalk_data)
{


	int32_t lb = 0;

	pxtalk_data->sample_count             = 0U;
	pxtalk_data->pll_period_mm            = 0U;
	pxtalk_data->peak_duration_us_sum     = 0U;
	pxtalk_data->effective_spad_count_sum = 0U;
	pxtalk_data->zero_distance_phase_sum  = 0U;
	pxtalk_data->zero_distance_phase_avg  = 0U;
	pxtalk_data->event_scaler_sum         = 0U;
	pxtalk_data->event_scaler_avg         = 4096U;
	pxtalk_data->signal_events_sum        = 0;
	pxtalk_data->xtalk_rate_kcps_per_spad = 0U;
	pxtalk_data->VL53LX_p_012             = 0U;
	pxtalk_data->VL53LX_p_013               = 0U;
	pxtalk_data->target_start             = 0U;

	for (lb = 0; lb < VL53LX_XTALK_HISTO_BINS; lb++)
		pxtalk_data->bin_data_sums[lb] = 0;

}


VL53LX_Error VL53LX_hist_xtalk_extract_update(
	int16_t                             target_distance_mm,
	uint16_t                            target_width_oversize,
	VL53LX_histogram_bin_data_t        *phist_bins,
	VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status =
		VL53LX_hist_xtalk_extract_calc_window(
			target_distance_mm,
			target_width_oversize,
			phist_bins,
			pxtalk_data);

	if (status == VL53LX_ERROR_NONE) {
		status =
			VL53LX_hist_xtalk_extract_calc_event_sums(
				phist_bins,
				pxtalk_data);
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_hist_xtalk_extract_fini(
	VL53LX_histogram_bin_data_t        *phist_bins,
	VL53LX_hist_xtalk_extract_data_t   *pxtalk_data,
	VL53LX_xtalk_calibration_results_t *pxtalk_cal,
	VL53LX_xtalk_histogram_shape_t     *pxtalk_shape)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_xtalk_calibration_results_t *pX = pxtalk_cal;

	LOG_FUNCTION_START("");

	if (pxtalk_data->sample_count > 0) {


		pxtalk_data->event_scaler_avg  = pxtalk_data->event_scaler_sum;
		pxtalk_data->event_scaler_avg +=
				(pxtalk_data->sample_count >> 1);
		pxtalk_data->event_scaler_avg /=  pxtalk_data->sample_count;



		status =
			VL53LX_hist_xtalk_extract_calc_rate_per_spad(
				pxtalk_data);



		if (status == VL53LX_ERROR_NONE) {


			pxtalk_data->zero_distance_phase_avg =
				pxtalk_data->zero_distance_phase_sum;
			pxtalk_data->zero_distance_phase_avg +=
					(pxtalk_data->sample_count >> 1);
			pxtalk_data->zero_distance_phase_avg /=
					pxtalk_data->sample_count;


			status =
				VL53LX_hist_xtalk_extract_calc_shape(
					pxtalk_data,
					pxtalk_shape);




			pxtalk_shape->phasecal_result__vcsel_start =
				phist_bins->phasecal_result__vcsel_start;
			pxtalk_shape->cal_config__vcsel_start =
				phist_bins->cal_config__vcsel_start;
			pxtalk_shape->vcsel_width =
				phist_bins->vcsel_width;
			pxtalk_shape->VL53LX_p_015 =
				phist_bins->VL53LX_p_015;
		}


		if (status == VL53LX_ERROR_NONE) {


			pX->algo__crosstalk_compensation_plane_offset_kcps =
				pxtalk_data->xtalk_rate_kcps_per_spad;
			pX->algo__crosstalk_compensation_x_plane_gradient_kcps
				= 0U;
			pX->algo__crosstalk_compensation_y_plane_gradient_kcps
				= 0U;

		}
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error   VL53LX_run_hist_xtalk_extraction(
	VL53LX_DEV	                        Dev,
	int16_t                             cal_distance_mm,
	VL53LX_Error                       *pcal_status)
{


	#define OVERSIZE 4
	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_xtalkextract_config_t *pX = &(pdev->xtalk_extract_cfg);
	VL53LX_xtalk_config_t *pC = &(pdev->xtalk_cfg);
	VL53LX_xtalk_calibration_results_t *pXC = &(pdev->xtalk_cal);



	uint8_t smudge_corr_en   = 0;
	uint8_t i                = 0;
	int8_t k = 0;
	uint8_t nbloops;
	int32_t initMergeSize = 0;
	int32_t MergeEnabled = 0;
	uint32_t deltaXtalk;
	uint32_t stepXtalk;
	uint32_t XtalkMin;
	uint32_t XtalkMax;
	uint8_t measurement_mode = VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
	int8_t MaxId;
	uint8_t histo_merge_nb;
	uint8_t wait_for_accumulation;
	VL53LX_range_results_t     *prange_results =
		(VL53LX_range_results_t *) pdev->wArea1;
	uint8_t Very1stRange = 0;

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_set_preset_mode(
				Dev,
				VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE,
				pX->dss_config__target_total_rate_mcps,
				pX->phasecal_config_timeout_us,
				pX->mm_config_timeout_us,
				pX->range_config_timeout_us,
				100);



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_disable_xtalk_compensation(Dev);



	smudge_corr_en = pdev->smudge_correct_config.smudge_corr_enabled;

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_dynamic_xtalk_correction_disable(Dev);


	VL53LX_load_patch(Dev);

	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
			&initMergeSize);
	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_HIST_MERGE,
			&MergeEnabled);
	memset(&pdev->xtalk_cal, 0,	sizeof(pdev->xtalk_cal));

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_and_start_range(
			Dev, measurement_mode,
			VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS);

	MaxId = pdev->tuning_parms.tp_hist_merge_max_size - 1;
	nbloops = (MergeEnabled == 0 ? 1 : 2);
	for (k = 0; k < nbloops; k++) {

		VL53LX_hist_xtalk_extract_data_init(
				&(pdev->xtalk_extract));
		VL53LX_set_tuning_parm(Dev,
				VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
				k * MaxId + 1);

		for (i = 0; i <= pX->num_of_samples; i++) {
			if (status == VL53LX_ERROR_NONE)
				status = VL53LX_wait_for_range_completion(Dev);
			if (status == VL53LX_ERROR_NONE)
				status = VL53LX_get_device_results(Dev,
					VL53LX_DEVICERESULTSLEVEL_FULL,
					prange_results);
			Very1stRange =
				(pdev->ll_state.rd_device_state ==
				VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC);

			VL53LX_compute_histo_merge_nb(Dev, &histo_merge_nb);
			wait_for_accumulation = ((k != 0) &&
				(MergeEnabled) &&
				(status == VL53LX_ERROR_NONE) &&
				(histo_merge_nb <
				pdev->tuning_parms.tp_hist_merge_max_size));
			if (wait_for_accumulation)
				i = 0;
			else {
				if ((status == VL53LX_ERROR_NONE) &&
					(!Very1stRange)) {
					status =
					VL53LX_hist_xtalk_extract_update(
						cal_distance_mm,
						OVERSIZE,
						&(pdev->hist_data),
						&(pdev->xtalk_extract));
				}
			}

			if (status == VL53LX_ERROR_NONE)
				status =
				VL53LX_clear_interrupt_and_enable_next_range(
					Dev, measurement_mode);
		}

		if (status == VL53LX_ERROR_NONE)
			status =
			VL53LX_hist_xtalk_extract_fini(
				&(pdev->hist_data),
				&(pdev->xtalk_extract),
				&(pdev->xtalk_cal),
				&(pdev->xtalk_shapes.xtalk_shape));
		if (status == VL53LX_ERROR_NONE) {
			pXC->algo__xtalk_cpo_HistoMerge_kcps[k * MaxId] =
			pXC->algo__crosstalk_compensation_plane_offset_kcps;
		}
	}


	VL53LX_stop_range(Dev);

	VL53LX_set_tuning_parm(Dev, VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
			initMergeSize);
	VL53LX_unload_patch(Dev);

	if (status != VL53LX_ERROR_NONE)
		status = VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL;
	else if ((MergeEnabled == 1) && (MaxId > 0)) {
		XtalkMin = pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[0];
		XtalkMax =
		pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[MaxId];
		pdev->xtalk_cal.
		algo__crosstalk_compensation_plane_offset_kcps = XtalkMin;
		if (XtalkMax > XtalkMin) {
			deltaXtalk =  XtalkMax - XtalkMin;
			stepXtalk = deltaXtalk / MaxId;
			for (k = 1; k < MaxId; k++)
			pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[k] =
				XtalkMin + stepXtalk * k;
		} else
			status =
				VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL;
	}

	if (status == VL53LX_ERROR_NONE) {
		pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pXC->algo__crosstalk_compensation_x_plane_gradient_kcps;
		pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pXC->algo__crosstalk_compensation_y_plane_gradient_kcps;
		pC->algo__crosstalk_compensation_plane_offset_kcps =
		pXC->algo__crosstalk_compensation_plane_offset_kcps;
	}


	// pdev->xtalk_results.cal_status = status;
	// *pcal_status = pdev->xtalk_results.cal_status;


	status = VL53LX_enable_xtalk_compensation(Dev);
	if (smudge_corr_en == 1)
		status = VL53LX_dynamic_xtalk_correction_enable(Dev);

#ifdef VL53LX_LOG_ENABLE



	VL53LX_print_customer_nvm_managed(
		&(pdev->customer),
		"run_xtalk_extraction():pdev->lldata.customer.",
		VL53LX_TRACE_MODULE_XTALK_DATA);

	VL53LX_print_xtalk_config(
		&(pdev->xtalk_cfg),
		"run_xtalk_extraction():pdev->lldata.xtalk_cfg.",
		VL53LX_TRACE_MODULE_XTALK_DATA);

	VL53LX_print_xtalk_histogram_data(
		&(pdev->xtalk_shapes),
		"pdev->lldata.xtalk_shapes.",
		VL53LX_TRACE_MODULE_XTALK_DATA);

#endif

	LOG_FUNCTION_END(status);

	return status;
}

