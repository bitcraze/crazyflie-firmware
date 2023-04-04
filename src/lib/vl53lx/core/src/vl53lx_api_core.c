
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
#include "vl53lx_register_settings.h"
#include "vl53lx_register_funcs.h"
#include "vl53lx_hist_map.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_nvm_map.h"
#include "vl53lx_nvm_structs.h"
#include "vl53lx_nvm.h"
#include "vl53lx_core.h"
#include "vl53lx_wait.h"
#include "vl53lx_api_preset_modes.h"
#include "vl53lx_silicon_core.h"
#include "vl53lx_api_core.h"
#include "vl53lx_tuning_parm_defaults.h"

#ifdef VL53LX_LOG_ENABLE
#include "vl53lx_api_debug.h"
#endif

#define LOG_FUNCTION_START(fmt, ...) 
#define LOG_FUNCTION_END(status, ...) 
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

#define trace_print(level, ...)

#define VL53LX_MAX_I2C_XFER_SIZE 256

static VL53LX_Error select_offset_per_vcsel(VL53LX_LLDriverData_t *pdev,
		int16_t *poffset) {
	VL53LX_Error status = VL53LX_ERROR_NONE;
	int16_t tA, tB;
	uint8_t isc;

	switch (pdev->preset_mode) {
	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:
		tA = pdev->per_vcsel_cal_data.short_a_offset_mm;
		tB = pdev->per_vcsel_cal_data.short_b_offset_mm;
		break;
	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
		tA = pdev->per_vcsel_cal_data.medium_a_offset_mm;
		tB = pdev->per_vcsel_cal_data.medium_b_offset_mm;
		break;
	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:
		tA = pdev->per_vcsel_cal_data.long_a_offset_mm;
		tB = pdev->per_vcsel_cal_data.long_b_offset_mm;
		break;
	default:
		tA = pdev->per_vcsel_cal_data.long_a_offset_mm;
		tB = pdev->per_vcsel_cal_data.long_b_offset_mm;
		status = VL53LX_ERROR_INVALID_PARAMS;
		*poffset = 0;
		break;
	}

	isc = pdev->ll_state.cfg_internal_stream_count;
	if (status == VL53LX_ERROR_NONE)
		*poffset = (isc & 0x01) ? tA : tB;

	return status;
}

static void vl53lx_diff_histo_stddev(VL53LX_LLDriverData_t *pdev,
	VL53LX_histogram_bin_data_t *pdata, uint8_t timing, uint8_t HighIndex,
	uint8_t prev_pos, int32_t *pdiff_histo_stddev) {
	uint16_t   bin                      = 0;
	int32_t    total_rate_pre = 0;
	int32_t    total_rate_cur = 0;
	int32_t    PrevBin, CurrBin;

	total_rate_pre = 0;
	total_rate_cur = 0;


	for (bin = timing * 4; bin < HighIndex; bin++) {
		total_rate_pre +=
		pdev->multi_bins_rec[prev_pos][timing][bin];
		total_rate_cur += pdata->bin_data[bin];
	}

	if ((total_rate_pre != 0) && (total_rate_cur != 0))
		for (bin = timing * 4; bin < HighIndex; bin++) {
			PrevBin = pdev->multi_bins_rec[prev_pos][timing][bin];
			PrevBin = (PrevBin * 1000) / total_rate_pre;
			CurrBin = pdata->bin_data[bin] * 1000 / total_rate_cur;
			*pdiff_histo_stddev += (PrevBin - CurrBin) *
					(PrevBin - CurrBin);
	}
}

static void vl53lx_histo_merge(VL53LX_DEV Dev,
		VL53LX_histogram_bin_data_t *pdata) {
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	uint16_t   bin                      = 0;
	uint8_t    i                        = 0;
	int32_t    TuningBinRecSize		    = 0;
	uint8_t    recom_been_reset			= 0;
	uint8_t    timing					= 0;
	int32_t    rmt  = 0;
	int32_t    diff_histo_stddev		= 0;
	uint8_t    HighIndex, prev_pos;
	uint8_t    BuffSize = VL53LX_HISTOGRAM_BUFFER_SIZE;
	uint8_t    pos;

	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE,
			&TuningBinRecSize);

	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD,
		&rmt);


	if (pdev->pos_before_next_recom == 0) {

		timing = 1 - pdata->result__stream_count % 2;

		diff_histo_stddev = 0;
		HighIndex = BuffSize - timing * 4;
		if (pdev->bin_rec_pos > 0)
			prev_pos = pdev->bin_rec_pos - 1;
		else
			prev_pos = (TuningBinRecSize - 1);

		if (pdev->multi_bins_rec[prev_pos][timing][4] > 0)
			vl53lx_diff_histo_stddev(pdev, pdata,
				timing, HighIndex, prev_pos,
				&diff_histo_stddev);

		if (diff_histo_stddev >= rmt) {
			memset(pdev->multi_bins_rec, 0,
				sizeof(pdev->multi_bins_rec));
			pdev->bin_rec_pos = 0;

			recom_been_reset = 1;

			if (timing == 0)
				pdev->pos_before_next_recom =
					VL53LX_FRAME_WAIT_EVENT;
			else
				pdev->pos_before_next_recom =
					VL53LX_FRAME_WAIT_EVENT + 1;
		} else {

			pos = pdev->bin_rec_pos;
			for (i = 0; i < BuffSize; i++)
				pdev->multi_bins_rec[pos][timing][i] =
					pdata->bin_data[i];
		}

		if (pdev->bin_rec_pos == (TuningBinRecSize - 1) && timing == 1)
			pdev->bin_rec_pos = 0;
		else if (timing == 1)
			pdev->bin_rec_pos++;

		if (!((recom_been_reset == 1) && (timing == 0)) &&
			 (pdev->pos_before_next_recom == 0)) {

			for (bin = 0; bin < BuffSize; bin++)
				pdata->bin_data[bin] = 0;

			for (bin = 0; bin < BuffSize; bin++)
				for (i = 0; i < TuningBinRecSize; i++)
					pdata->bin_data[bin] +=
					(pdev->multi_bins_rec[i][timing][bin]);
		}
	} else {

		pdev->pos_before_next_recom--;
		if (pdev->pos_before_next_recom == 255)
			pdev->pos_before_next_recom = 0;
	}
}

VL53LX_Error VL53LX_load_patch(VL53LX_DEV Dev)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	int32_t patch_tuning = 0;
	uint8_t comms_buffer[256];
	uint32_t patch_power;

	LOG_FUNCTION_START("");

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(Dev,
		VL53LX_FIRMWARE__ENABLE, 0x00);

	if (status == VL53LX_ERROR_NONE)
		VL53LX_enable_powerforce(Dev);

	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER,
			&patch_tuning);

	switch (patch_tuning) {
	case 0:
		patch_power = 0x00;
		break;
	case 1:
		patch_power = 0x10;
		break;
	case 2:
		patch_power = 0x20;
		break;
	case 3:
		patch_power = 0x40;
		break;
	default:
		patch_power = 0x00;
	}

	if (status == VL53LX_ERROR_NONE) {

		comms_buffer[0] = 0x29;
		comms_buffer[1] = 0xC9;
		comms_buffer[2] = 0x0E;
		comms_buffer[3] = 0x40;
		comms_buffer[4] = 0x28;
		comms_buffer[5] = patch_power;

		status = VL53LX_WriteMulti(Dev,
		VL53LX_PATCH__OFFSET_0, comms_buffer, 6);
	}

	if (status == VL53LX_ERROR_NONE) {
		comms_buffer[0] = 0x03;
		comms_buffer[1] = 0x6D;
		comms_buffer[2] = 0x03;
		comms_buffer[3] = 0x6F;
		comms_buffer[4] = 0x07;
		comms_buffer[5] = 0x29;
		status = VL53LX_WriteMulti(Dev,
		VL53LX_PATCH__ADDRESS_0, comms_buffer, 6);
	}

	if (status == VL53LX_ERROR_NONE) {
		comms_buffer[0] = 0x00;
		comms_buffer[1] = 0x07;
		status = VL53LX_WriteMulti(Dev,
		VL53LX_PATCH__JMP_ENABLES, comms_buffer, 2);
	}

	if (status == VL53LX_ERROR_NONE) {
		comms_buffer[0] = 0x00;
		comms_buffer[1] = 0x07;
		status = VL53LX_WriteMulti(Dev,
		VL53LX_PATCH__DATA_ENABLES, comms_buffer, 2);
	}

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(Dev,
		VL53LX_PATCH__CTRL, 0x01);

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(Dev,
		VL53LX_FIRMWARE__ENABLE, 0x01);

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_unload_patch(VL53LX_DEV Dev)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(Dev,
		VL53LX_FIRMWARE__ENABLE, 0x00);

	if (status == VL53LX_ERROR_NONE)
		VL53LX_disable_powerforce(Dev);

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(Dev,
		VL53LX_PATCH__CTRL, 0x00);

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_WrByte(Dev,
		VL53LX_FIRMWARE__ENABLE, 0x01);

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_get_version(
	VL53LX_DEV           Dev,
	VL53LX_ll_version_t *pdata)
{


	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	VL53LX_init_version(Dev);

	memcpy(pdata, &(pdev->version), sizeof(VL53LX_ll_version_t));

	return VL53LX_ERROR_NONE;
}


VL53LX_Error VL53LX_data_init(
	VL53LX_DEV        Dev,
	uint8_t           read_p2p_data)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t    *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);



	VL53LX_zone_objects_t    *pobjects;

	uint8_t  i = 0;

	LOG_FUNCTION_START("");

	VL53LX_init_ll_driver_state(
			Dev,
			VL53LX_DEVICESTATE_UNKNOWN);

	pres->range_results.max_results    = VL53LX_MAX_RANGE_RESULTS;
	pres->range_results.active_results = 0;
	pres->zone_results.max_zones       = VL53LX_MAX_USER_ZONES;
	pres->zone_results.active_zones    = 0;

	for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
		pobjects = &(pres->zone_results.VL53LX_p_003[i]);
		pobjects->xmonitor.VL53LX_p_016 = 0;
		pobjects->xmonitor.VL53LX_p_017  = 0;
		pobjects->xmonitor.VL53LX_p_011          = 0;
		pobjects->xmonitor.range_status =
				VL53LX_DEVICEERROR_NOUPDATE;
	}



	pres->zone_hists.max_zones         = VL53LX_MAX_USER_ZONES;
	pres->zone_hists.active_zones      = 0;



	pres->zone_cal.max_zones           = VL53LX_MAX_USER_ZONES;
	pres->zone_cal.active_zones        = 0;
	for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
		pres->zone_cal.VL53LX_p_003[i].no_of_samples   = 0;
		pres->zone_cal.VL53LX_p_003[i].effective_spads = 0;
		pres->zone_cal.VL53LX_p_003[i].peak_rate_mcps  = 0;
		pres->zone_cal.VL53LX_p_003[i].median_range_mm = 0;
		pres->zone_cal.VL53LX_p_003[i].range_mm_offset = 0;
	}

	pdev->wait_method             = VL53LX_WAIT_METHOD_BLOCKING;
	pdev->preset_mode   = VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE;
	pdev->zone_preset             = 0;
	pdev->measurement_mode        = VL53LX_DEVICEMEASUREMENTMODE_STOP;

	pdev->offset_calibration_mode =
		VL53LX_OFFSETCALIBRATIONMODE__MM1_MM2__STANDARD;
	pdev->offset_correction_mode  =
		VL53LX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS;
	pdev->dmax_mode  =
		VL53LX_DEVICEDMAXMODE__FMT_CAL_DATA;

	pdev->phasecal_config_timeout_us  =  1000;
	pdev->mm_config_timeout_us        =  2000;
	pdev->range_config_timeout_us     = 13000;
	pdev->inter_measurement_period_ms =   100;
	pdev->dss_config__target_total_rate_mcps = 0x0A00;
	pdev->debug_mode                  =  0x00;

	pdev->offset_results.max_results    = VL53LX_MAX_OFFSET_RANGE_RESULTS;
	pdev->offset_results.active_results = 0;



	pdev->gain_cal.standard_ranging_gain_factor =
			VL53LX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT;
	pdev->gain_cal.histogram_ranging_gain_factor =
			VL53LX_TUNINGPARM_HIST_GAIN_FACTOR_DEFAULT;


	VL53LX_init_version(Dev);


	memset(pdev->multi_bins_rec, 0, sizeof(pdev->multi_bins_rec));
	pdev->bin_rec_pos = 0;
	pdev->pos_before_next_recom = 0;



	if (read_p2p_data > 0 && status == VL53LX_ERROR_NONE)
		status = VL53LX_read_p2p_data(Dev);


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_refspadchar_config_struct(
			&(pdev->VL53LX_LLDriverCommonData->refspadchar));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_ssc_config_struct(
			&(pdev->VL53LX_LLDriverCommonData->ssc_cfg));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_xtalk_config_struct(
			&(pdev->customer),
			&(pdev->VL53LX_LLDriverCommonData->xtalk_cfg));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_xtalk_extract_config_struct(
			&(pdev->VL53LX_LLDriverCommonData->xtalk_extract_cfg));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_offset_cal_config_struct(
		    &(pdev->VL53LX_LLDriverCommonData->offsetcal_cfg));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_zone_cal_config_struct(
			&(pdev->zonecal_cfg));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_hist_post_process_config_struct(
			pdev->VL53LX_LLDriverCommonData->xtalk_cfg.global_crosstalk_compensation_enable,
			&(pdev->histpostprocess));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_hist_gen3_dmax_config_struct(
			&(pdev->VL53LX_LLDriverCommonData->dmax_cfg));


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_tuning_parm_storage_struct(
			&(pdev->VL53LX_LLDriverCommonData->tuning_parms));



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_set_preset_mode(
			Dev,
			pdev->preset_mode,
			pdev->dss_config__target_total_rate_mcps,
			pdev->phasecal_config_timeout_us,
			pdev->mm_config_timeout_us,
			pdev->range_config_timeout_us,
			pdev->inter_measurement_period_ms);


	VL53LX_init_histogram_bin_data_struct(
			0,
			VL53LX_HISTOGRAM_BUFFER_SIZE,
			&(pdev->VL53LX_LLDriverCommonData->hist_data));

	VL53LX_init_histogram_bin_data_struct(
			0,
			VL53LX_HISTOGRAM_BUFFER_SIZE,
			&(pdev->VL53LX_LLDriverCommonData->hist_xtalk));


	VL53LX_init_xtalk_bin_data_struct(
			0,
			VL53LX_XTALK_HISTO_BINS,
			&(pdev->xtalk_shapes.xtalk_shape));



	VL53LX_xtalk_cal_data_init(
			Dev
			);



	VL53LX_dynamic_xtalk_correction_data_init(
			Dev
			);



	VL53LX_low_power_auto_data_init(
			Dev
			);

#ifdef VL53LX_LOG_ENABLE



	VL53LX_print_static_nvm_managed(
		&(pdev->stat_nvm),
		"data_init():pdev->lldata.stat_nvm.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_customer_nvm_managed(
		&(pdev->customer),
		"data_init():pdev->lldata.customer.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_nvm_copy_data(
		&(pdev->nvm_copy_data),
		"data_init():pdev->lldata.nvm_copy_data.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_dmax_calibration_data(
		&(pdev->fmt_dmax_cal),
		"data_init():pdev->lldata.fmt_dmax_cal.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_dmax_calibration_data(
		&(pdev->cust_dmax_cal),
		"data_init():pdev->lldata.cust_dmax_cal.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_additional_offset_cal_data(
		&(pdev->add_off_cal_data),
		"data_init():pdev->lldata.add_off_cal_data.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_user_zone(
		&(pdev->mm_roi),
		"data_init():pdev->lldata.mm_roi.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_optical_centre(
		&(pdev->optical_centre),
		"data_init():pdev->lldata.optical_centre.",
		VL53LX_TRACE_MODULE_DATA_INIT);

	VL53LX_print_cal_peak_rate_map(
		&(pdev->cal_peak_rate_map),
		"data_init():pdev->lldata.cal_peak_rate_map.",
		VL53LX_TRACE_MODULE_DATA_INIT);

#endif

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_read_p2p_data(
	VL53LX_DEV        Dev)
{



	VL53LX_Error status       = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);
	VL53LX_additional_offset_cal_data_t *pCD = &(pdev->add_off_cal_data);

	VL53LX_decoded_nvm_fmt_range_data_t fmt_rrd;

	LOG_FUNCTION_START("");

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_get_static_nvm_managed(
						Dev,
						&(pdev->stat_nvm));

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_get_customer_nvm_managed(
						Dev,
						&(pdev->customer));

	if (status == VL53LX_ERROR_NONE) {

		status = VL53LX_get_nvm_copy_data(
						Dev,
						&(pdev->nvm_copy_data));


		if (status == VL53LX_ERROR_NONE)
			VL53LX_copy_rtn_good_spads_to_buffer(
					&(pdev->nvm_copy_data),
					&(pdev->rtn_good_spads[0]));
	}



	if (status == VL53LX_ERROR_NONE) {
		pHP->algo__crosstalk_compensation_plane_offset_kcps =
		pN->algo__crosstalk_compensation_plane_offset_kcps;
		pHP->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pN->algo__crosstalk_compensation_x_plane_gradient_kcps;
		pHP->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pN->algo__crosstalk_compensation_y_plane_gradient_kcps;
	}


	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_read_nvm_optical_centre(
				Dev,
				&(pdev->optical_centre));



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_read_nvm_cal_peak_rate_map(
				Dev,
				&(pdev->cal_peak_rate_map));



	if (status == VL53LX_ERROR_NONE) {

		status =
			VL53LX_read_nvm_additional_offset_cal_data(
				Dev,
				&(pdev->add_off_cal_data));



		if (pCD->result__mm_inner_peak_signal_count_rtn_mcps == 0 &&
			pCD->result__mm_outer_peak_signal_count_rtn_mcps == 0) {

			pCD->result__mm_inner_peak_signal_count_rtn_mcps
					= 0x0080;
			pCD->result__mm_outer_peak_signal_count_rtn_mcps
					= 0x0180;



			VL53LX_calc_mm_effective_spads(
			pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
			pdev->nvm_copy_data.roi_config__mode_roi_xy_size,
			0xC7,
			0xFF,
			&(pdev->rtn_good_spads[0]),
			VL53LX_RTN_SPAD_APERTURE_TRANSMISSION,
			&(pCD->result__mm_inner_actual_effective_spads),
			&(pCD->result__mm_outer_actual_effective_spads));
		}
	}


	if (status == VL53LX_ERROR_NONE) {

		status =
			VL53LX_read_nvm_fmt_range_results_data(
				Dev,
				VL53LX_NVM__FMT__RANGE_RESULTS__140MM_DARK,
				&fmt_rrd);

		if (status == VL53LX_ERROR_NONE) {
			pdev->fmt_dmax_cal.ref__actual_effective_spads =
			fmt_rrd.result__actual_effective_rtn_spads;
			pdev->fmt_dmax_cal.ref__peak_signal_count_rate_mcps =
			fmt_rrd.result__peak_signal_count_rate_rtn_mcps;
			pdev->fmt_dmax_cal.ref__distance_mm =
			fmt_rrd.measured_distance_mm;


			if (pdev->cal_peak_rate_map.cal_reflectance_pc != 0) {
				pdev->fmt_dmax_cal.ref_reflectance_pc =
				pdev->cal_peak_rate_map.cal_reflectance_pc;
			} else {
				pdev->fmt_dmax_cal.ref_reflectance_pc = 0x0014;
			}


			pdev->fmt_dmax_cal.coverglass_transmission = 0x0100;
		}
	}


	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_RdWord(
				Dev,
				VL53LX_RESULT__OSC_CALIBRATE_VAL,
				&(pdev->VL53LX_LLDriverCommonData->dbg_results.result__osc_calibrate_val));



	if (pdev->stat_nvm.osc_measured__fast_osc__frequency < 0x1000) {
		trace_print(
			VL53LX_TRACE_LEVEL_WARNING,
			"\nInvalid %s value (0x%04X) - forcing to 0x%04X\n\n",
			"pdev->stat_nvm.osc_measured__fast_osc__frequency",
			pdev->stat_nvm.osc_measured__fast_osc__frequency,
			0xBCCC);
		pdev->stat_nvm.osc_measured__fast_osc__frequency = 0xBCCC;
	}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_get_mode_mitigation_roi(
				Dev,
				&(pdev->mm_roi));



	if (pdev->optical_centre.x_centre == 0 &&
		pdev->optical_centre.y_centre == 0) {
		pdev->optical_centre.x_centre =
				pdev->mm_roi.x_centre << 4;
		pdev->optical_centre.y_centre =
				pdev->mm_roi.y_centre << 4;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_part_to_part_data(
	VL53LX_DEV                            Dev,
	VL53LX_calibration_data_t            *pcal_data)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_xtalk_config_t *pC = &(pdev->VL53LX_LLDriverCommonData->xtalk_cfg);
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

	uint32_t tempu32;

	LOG_FUNCTION_START("");

	if (pcal_data->struct_version !=
		VL53LX_LL_CALIBRATION_DATA_STRUCT_VERSION) {
		status = VL53LX_ERROR_INVALID_PARAMS;
	}

	if (status == VL53LX_ERROR_NONE) {


		memcpy(
			&(pdev->customer),
			&(pcal_data->customer),
			sizeof(VL53LX_customer_nvm_managed_t));


		memcpy(
			&(pdev->add_off_cal_data),
			&(pcal_data->add_off_cal_data),
			sizeof(VL53LX_additional_offset_cal_data_t));


		memcpy(
			&(pdev->fmt_dmax_cal),
			&(pcal_data->fmt_dmax_cal),
			sizeof(VL53LX_dmax_calibration_data_t));


		memcpy(
			&(pdev->cust_dmax_cal),
			&(pcal_data->cust_dmax_cal),
			sizeof(VL53LX_dmax_calibration_data_t));


		memcpy(
			&(pdev->xtalk_shapes),
			&(pcal_data->xtalkhisto),
			sizeof(VL53LX_xtalk_histogram_data_t));


		memcpy(
			&(pdev->gain_cal),
			&(pcal_data->gain_cal),
			sizeof(VL53LX_gain_calibration_data_t));


		memcpy(
			&(pdev->cal_peak_rate_map),
			&(pcal_data->cal_peak_rate_map),
			sizeof(VL53LX_cal_peak_rate_map_t));


		memcpy(
			&(pdev->per_vcsel_cal_data),
			&(pcal_data->per_vcsel_cal_data),
			sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));



		pC->algo__crosstalk_compensation_plane_offset_kcps =
			pN->algo__crosstalk_compensation_plane_offset_kcps;
		pC->algo__crosstalk_compensation_x_plane_gradient_kcps =
			pN->algo__crosstalk_compensation_x_plane_gradient_kcps;
		pC->algo__crosstalk_compensation_y_plane_gradient_kcps =
			pN->algo__crosstalk_compensation_y_plane_gradient_kcps;

		pHP->algo__crosstalk_compensation_plane_offset_kcps =
			VL53LX_calc_crosstalk_plane_offset_with_margin(
			pC->algo__crosstalk_compensation_plane_offset_kcps,
			pC->histogram_mode_crosstalk_margin_kcps);

		pHP->algo__crosstalk_compensation_x_plane_gradient_kcps =
			pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
		pHP->algo__crosstalk_compensation_y_plane_gradient_kcps =
			pC->algo__crosstalk_compensation_y_plane_gradient_kcps;



		if (pC->global_crosstalk_compensation_enable == 0x00) {
			pN->algo__crosstalk_compensation_plane_offset_kcps =
				0x00;
			pN->algo__crosstalk_compensation_x_plane_gradient_kcps =
				0x00;
			pN->algo__crosstalk_compensation_y_plane_gradient_kcps =
				0x00;
		} else {
			tempu32 =
			VL53LX_calc_crosstalk_plane_offset_with_margin(
			pC->algo__crosstalk_compensation_plane_offset_kcps,
			pC->lite_mode_crosstalk_margin_kcps);


			if (tempu32 > 0xFFFF)
				tempu32 = 0xFFFF;

			pN->algo__crosstalk_compensation_plane_offset_kcps =
				(uint16_t)tempu32;
		}
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_part_to_part_data(
	VL53LX_DEV                      Dev,
	VL53LX_calibration_data_t      *pcal_data)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_xtalk_config_t *pC = &(pdev->VL53LX_LLDriverCommonData->xtalk_cfg);
	VL53LX_customer_nvm_managed_t *pCN = &(pcal_data->customer);

	LOG_FUNCTION_START("");

	pcal_data->struct_version =
			VL53LX_LL_CALIBRATION_DATA_STRUCT_VERSION;


	memcpy(
		&(pcal_data->customer),
		&(pdev->customer),
		sizeof(VL53LX_customer_nvm_managed_t));




	if (pC->algo__crosstalk_compensation_plane_offset_kcps > 0xFFFF) {
		pCN->algo__crosstalk_compensation_plane_offset_kcps =
			0xFFFF;
	} else {
		pCN->algo__crosstalk_compensation_plane_offset_kcps =
		(uint16_t)pC->algo__crosstalk_compensation_plane_offset_kcps;
	}
	pCN->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
	pCN->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_y_plane_gradient_kcps;


	memcpy(
		&(pcal_data->fmt_dmax_cal),
		&(pdev->fmt_dmax_cal),
		sizeof(VL53LX_dmax_calibration_data_t));


	memcpy(
		&(pcal_data->cust_dmax_cal),
		&(pdev->cust_dmax_cal),
		sizeof(VL53LX_dmax_calibration_data_t));


	memcpy(
		&(pcal_data->add_off_cal_data),
		&(pdev->add_off_cal_data),
		sizeof(VL53LX_additional_offset_cal_data_t));


	memcpy(
		&(pcal_data->optical_centre),
		&(pdev->optical_centre),
		sizeof(VL53LX_optical_centre_t));


	memcpy(
		&(pcal_data->xtalkhisto),
		&(pdev->xtalk_shapes),
		sizeof(VL53LX_xtalk_histogram_data_t));


	memcpy(
		&(pcal_data->gain_cal),
		&(pdev->gain_cal),
		sizeof(VL53LX_gain_calibration_data_t));


	memcpy(
		&(pcal_data->cal_peak_rate_map),
		&(pdev->cal_peak_rate_map),
		sizeof(VL53LX_cal_peak_rate_map_t));


	memcpy(
		&(pcal_data->per_vcsel_cal_data),
		&(pdev->per_vcsel_cal_data),
		sizeof(VL53LX_per_vcsel_period_offset_cal_data_t));

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_inter_measurement_period_ms(
	VL53LX_DEV              Dev,
	uint32_t                inter_measurement_period_ms)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	if (pdev->VL53LX_LLDriverCommonData->dbg_results.result__osc_calibrate_val == 0)
		status = VL53LX_ERROR_DIVISION_BY_ZERO;

	if (status == VL53LX_ERROR_NONE) {
		pdev->inter_measurement_period_ms = inter_measurement_period_ms;
		pdev->tim_cfg.system__intermeasurement_period =
			inter_measurement_period_ms *
			(uint32_t)pdev->VL53LX_LLDriverCommonData->dbg_results.result__osc_calibrate_val;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_inter_measurement_period_ms(
	VL53LX_DEV              Dev,
	uint32_t               *pinter_measurement_period_ms)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	if (pdev->VL53LX_LLDriverCommonData->dbg_results.result__osc_calibrate_val == 0)
		status = VL53LX_ERROR_DIVISION_BY_ZERO;

	if (status == VL53LX_ERROR_NONE)
		*pinter_measurement_period_ms =
			pdev->tim_cfg.system__intermeasurement_period /
			(uint32_t)pdev->VL53LX_LLDriverCommonData->dbg_results.result__osc_calibrate_val;


	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_timeouts_us(
	VL53LX_DEV          Dev,
	uint32_t            phasecal_config_timeout_us,
	uint32_t            mm_config_timeout_us,
	uint32_t            range_config_timeout_us)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	if (pdev->stat_nvm.osc_measured__fast_osc__frequency == 0)
		status = VL53LX_ERROR_DIVISION_BY_ZERO;

	if (status == VL53LX_ERROR_NONE) {

		pdev->phasecal_config_timeout_us = phasecal_config_timeout_us;
		pdev->mm_config_timeout_us       = mm_config_timeout_us;
		pdev->range_config_timeout_us    = range_config_timeout_us;

		status =
		VL53LX_calc_timeout_register_values(
			phasecal_config_timeout_us,
			mm_config_timeout_us,
			range_config_timeout_us,
			pdev->stat_nvm.osc_measured__fast_osc__frequency,
			&(pdev->gen_cfg),
			&(pdev->tim_cfg));
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_timeouts_us(
	VL53LX_DEV           Dev,
	uint32_t            *pphasecal_config_timeout_us,
	uint32_t            *pmm_config_timeout_us,
	uint32_t			*prange_config_timeout_us)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);

	uint32_t  macro_period_us = 0;
	uint16_t  timeout_encoded = 0;

	LOG_FUNCTION_START("");

	if (pdev->stat_nvm.osc_measured__fast_osc__frequency == 0)
		status = VL53LX_ERROR_DIVISION_BY_ZERO;

	if (status == VL53LX_ERROR_NONE) {


		macro_period_us =
			VL53LX_calc_macro_period_us(
			pdev->stat_nvm.osc_measured__fast_osc__frequency,
			pdev->tim_cfg.range_config__vcsel_period_a);



		*pphasecal_config_timeout_us =
			VL53LX_calc_timeout_us(
			(uint32_t)pdev->gen_cfg.phasecal_config__timeout_macrop,
			macro_period_us);



		timeout_encoded =
			(uint16_t)pdev->tim_cfg.mm_config__timeout_macrop_a_hi;
		timeout_encoded = (timeout_encoded << 8) +
			(uint16_t)pdev->tim_cfg.mm_config__timeout_macrop_a_lo;

		*pmm_config_timeout_us =
			VL53LX_calc_decoded_timeout_us(
				timeout_encoded,
				macro_period_us);



		timeout_encoded =
		(uint16_t)pdev->tim_cfg.range_config__timeout_macrop_a_hi;
		timeout_encoded = (timeout_encoded << 8) +
		(uint16_t)pdev->tim_cfg.range_config__timeout_macrop_a_lo;

		*prange_config_timeout_us =
			VL53LX_calc_decoded_timeout_us(
				timeout_encoded,
				macro_period_us);

		pdev->phasecal_config_timeout_us = *pphasecal_config_timeout_us;
		pdev->mm_config_timeout_us       = *pmm_config_timeout_us;
		pdev->range_config_timeout_us    = *prange_config_timeout_us;

	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_user_zone(
	VL53LX_DEV              Dev,
	VL53LX_user_zone_t     *puser_zone)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	VL53LX_encode_row_col(
		puser_zone->y_centre,
		puser_zone->x_centre,
		&(pdev->dyn_cfg.roi_config__user_roi_centre_spad));


	VL53LX_encode_zone_size(
		puser_zone->width,
		puser_zone->height,
		&(pdev->dyn_cfg.roi_config__user_roi_requested_global_xy_size));



	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_user_zone(
	VL53LX_DEV              Dev,
	VL53LX_user_zone_t     *puser_zone)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	VL53LX_decode_row_col(
			pdev->dyn_cfg.roi_config__user_roi_centre_spad,
			&(puser_zone->y_centre),
			&(puser_zone->x_centre));


	VL53LX_decode_zone_size(
		pdev->dyn_cfg.roi_config__user_roi_requested_global_xy_size,
		&(puser_zone->width),
		&(puser_zone->height));

	LOG_FUNCTION_END(status);

	return status;
}



VL53LX_Error VL53LX_get_mode_mitigation_roi(
	VL53LX_DEV              Dev,
	VL53LX_user_zone_t     *pmm_roi)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	uint8_t  x       = 0;
	uint8_t  y       = 0;
	uint8_t  xy_size = 0;

	LOG_FUNCTION_START("");


	VL53LX_decode_row_col(
			pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
			&y,
			&x);

	pmm_roi->x_centre = x;
	pmm_roi->y_centre = y;


	xy_size = pdev->nvm_copy_data.roi_config__mode_roi_xy_size;

	pmm_roi->height = xy_size >> 4;
	pmm_roi->width  = xy_size & 0x0F;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_init_zone_config_histogram_bins(
	VL53LX_zone_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	uint8_t i;

	LOG_FUNCTION_START("");

	for (i = 0; i < pdata->max_zones; i++)
		pdata->bin_config[i] = VL53LX_ZONECONFIG_BINCONFIG__LOWAMB;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_set_zone_config(
	VL53LX_DEV                 Dev,
	VL53LX_zone_config_t      *pzone_cfg)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	memcpy(&(pdev->zone_cfg.user_zones), &(pzone_cfg->user_zones),
			sizeof(pdev->zone_cfg.user_zones));


	pdev->zone_cfg.max_zones    = pzone_cfg->max_zones;
	pdev->zone_cfg.active_zones = pzone_cfg->active_zones;

	status = VL53LX_init_zone_config_histogram_bins(&pdev->zone_cfg);



	if (pzone_cfg->active_zones == 0)
		pdev->gen_cfg.global_config__stream_divider = 0;
	else if (pzone_cfg->active_zones < VL53LX_MAX_USER_ZONES)
		pdev->gen_cfg.global_config__stream_divider =
				pzone_cfg->active_zones + 1;
	else
		pdev->gen_cfg.global_config__stream_divider =
				VL53LX_MAX_USER_ZONES + 1;

	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_get_zone_config(
	VL53LX_DEV                 Dev,
	VL53LX_zone_config_t      *pzone_cfg)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	memcpy(pzone_cfg, &(pdev->zone_cfg), sizeof(VL53LX_zone_config_t));

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_preset_mode_timing_cfg(
	VL53LX_DEV                   Dev,
	VL53LX_DevicePresetModes     device_preset_mode,
	uint16_t                    *pdss_config__target_total_rate_mcps,
	uint32_t                    *pphasecal_config_timeout_us,
	uint32_t                    *pmm_config_timeout_us,
	uint32_t                    *prange_config_timeout_us)
{
	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	switch (device_preset_mode) {

	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:
		*pdss_config__target_total_rate_mcps =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mcps;
		*pphasecal_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_long_us;
		*pmm_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_histo_us;
		*prange_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_histo_us;

	break;

	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
		*pdss_config__target_total_rate_mcps =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mcps;
		*pphasecal_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_med_us;
		*pmm_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_histo_us;
		*prange_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_histo_us;
	break;

	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:
		*pdss_config__target_total_rate_mcps =
				pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mcps;
		*pphasecal_config_timeout_us =
			pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_short_us;
		*pmm_config_timeout_us =
				pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_histo_us;
		*prange_config_timeout_us =
				pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_histo_us;
	break;

	default:
		status = VL53LX_ERROR_INVALID_PARAMS;
		break;

	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_preset_mode(
	VL53LX_DEV                   Dev,
	VL53LX_DevicePresetModes     device_preset_mode,
	uint16_t                     dss_config__target_total_rate_mcps,
	uint32_t                     phasecal_config_timeout_us,
	uint32_t                     mm_config_timeout_us,
	uint32_t                     range_config_timeout_us,
	uint32_t                     inter_measurement_period_ms)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	VL53LX_hist_post_process_config_t *phistpostprocess =
			&(pdev->histpostprocess);

	VL53LX_static_config_t        *pstatic       = &(pdev->stat_cfg);
	VL53LX_histogram_config_t     *phistogram    = &(pdev->hist_cfg);
	VL53LX_general_config_t       *pgeneral      = &(pdev->gen_cfg);
	VL53LX_timing_config_t        *ptiming       = &(pdev->tim_cfg);
	VL53LX_dynamic_config_t       *pdynamic      = &(pdev->dyn_cfg);
	VL53LX_system_control_t       *psystem       = &(pdev->sys_ctrl);
	VL53LX_zone_config_t          *pzone_cfg     = &(pdev->zone_cfg);
	VL53LX_tuning_parm_storage_t  *ptuning_parms = &(pdev->VL53LX_LLDriverCommonData->tuning_parms);

	LOG_FUNCTION_START("");


	pdev->preset_mode                 = device_preset_mode;
	pdev->mm_config_timeout_us        = mm_config_timeout_us;
	pdev->range_config_timeout_us     = range_config_timeout_us;
	pdev->inter_measurement_period_ms = inter_measurement_period_ms;



	VL53LX_init_ll_driver_state(
			Dev,
			VL53LX_DEVICESTATE_SW_STANDBY);



	switch (device_preset_mode) {
	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_LONG_RANGE:

		status = VL53LX_preset_mode_histogram_long_range(
					phistpostprocess,
					pstatic,
					phistogram,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms,
					pzone_cfg);
		break;

	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_MEDIUM_RANGE:
		status = VL53LX_preset_mode_histogram_medium_range(
					phistpostprocess,
					pstatic,
					phistogram,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms,
					pzone_cfg);
		break;

	case VL53LX_DEVICEPRESETMODE_HISTOGRAM_SHORT_RANGE:
		status = VL53LX_preset_mode_histogram_short_range(
					phistpostprocess,
					pstatic,
					phistogram,
					pgeneral,
					ptiming,
					pdynamic,
					psystem,
					ptuning_parms,
					pzone_cfg);
		break;

	default:
		status = VL53LX_ERROR_INVALID_PARAMS;
		break;

	}



	if (status == VL53LX_ERROR_NONE) {

		pstatic->dss_config__target_total_rate_mcps =
				dss_config__target_total_rate_mcps;
		pdev->dss_config__target_total_rate_mcps    =
				dss_config__target_total_rate_mcps;

	}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_set_timeouts_us(
				Dev,
				phasecal_config_timeout_us,
				mm_config_timeout_us,
				range_config_timeout_us);

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_set_inter_measurement_period_ms(
				Dev,
				inter_measurement_period_ms);



	V53L1_init_zone_results_structure(
			pdev->zone_cfg.active_zones+1,
			&(pres->zone_results));

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error  VL53LX_enable_xtalk_compensation(
	VL53LX_DEV                 Dev)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint32_t tempu32;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_xtalk_config_t *pC = &(pdev->VL53LX_LLDriverCommonData->xtalk_cfg);
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

	LOG_FUNCTION_START("");


	tempu32 = VL53LX_calc_crosstalk_plane_offset_with_margin(
		pC->algo__crosstalk_compensation_plane_offset_kcps,
		pC->lite_mode_crosstalk_margin_kcps);
	if (tempu32 > 0xFFFF)
		tempu32 = 0xFFFF;

	pN->algo__crosstalk_compensation_plane_offset_kcps =
		(uint16_t)tempu32;

	pN->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_x_plane_gradient_kcps;

	pN->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_y_plane_gradient_kcps;


	pHP->algo__crosstalk_compensation_plane_offset_kcps =
		VL53LX_calc_crosstalk_plane_offset_with_margin(
			pC->algo__crosstalk_compensation_plane_offset_kcps,
			pC->histogram_mode_crosstalk_margin_kcps);

	pHP->algo__crosstalk_compensation_x_plane_gradient_kcps
		= pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
	pHP->algo__crosstalk_compensation_y_plane_gradient_kcps
		= pC->algo__crosstalk_compensation_y_plane_gradient_kcps;



	pC->global_crosstalk_compensation_enable = 0x01;

	pHP->algo__crosstalk_compensation_enable =
		pC->global_crosstalk_compensation_enable;




	if (status == VL53LX_ERROR_NONE) {
		pC->crosstalk_range_ignore_threshold_rate_mcps =
		VL53LX_calc_range_ignore_threshold(
			pC->algo__crosstalk_compensation_plane_offset_kcps,
			pC->algo__crosstalk_compensation_x_plane_gradient_kcps,
			pC->algo__crosstalk_compensation_y_plane_gradient_kcps,
			pC->crosstalk_range_ignore_threshold_mult);
}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_set_customer_nvm_managed(
				Dev,
				&(pdev->customer));

	LOG_FUNCTION_END(status);

	return status;

}

void VL53LX_get_xtalk_compensation_enable(
	VL53LX_DEV    Dev,
	uint8_t       *pcrosstalk_compensation_enable)
{


	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");



	*pcrosstalk_compensation_enable =
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.global_crosstalk_compensation_enable;

}


VL53LX_Error  VL53LX_disable_xtalk_compensation(
	VL53LX_DEV                 Dev)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

	LOG_FUNCTION_START("");


	pN->algo__crosstalk_compensation_plane_offset_kcps =
		0x00;

	pN->algo__crosstalk_compensation_x_plane_gradient_kcps =
		0x00;

	pN->algo__crosstalk_compensation_y_plane_gradient_kcps =
		0x00;



	pdev->VL53LX_LLDriverCommonData->xtalk_cfg.global_crosstalk_compensation_enable = 0x00;

	pHP->algo__crosstalk_compensation_enable =
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.global_crosstalk_compensation_enable;



	if (status == VL53LX_ERROR_NONE) {
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
			0x0000;
	}



	if (status == VL53LX_ERROR_NONE) {
		status =
			VL53LX_set_customer_nvm_managed(
				Dev,
				&(pdev->customer));
	}
	LOG_FUNCTION_END(status);

	return status;

}


VL53LX_Error VL53LX_init_and_start_range(
	VL53LX_DEV                     Dev,
	uint8_t                        measurement_mode,
	VL53LX_DeviceConfigLevel       device_config_level)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t  *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	uint8_t buffer[VL53LX_MAX_I2C_XFER_SIZE];

	VL53LX_static_nvm_managed_t   *pstatic_nvm   = &(pdev->stat_nvm);
	VL53LX_customer_nvm_managed_t *pcustomer_nvm = &(pdev->customer);
	VL53LX_static_config_t        *pstatic       = &(pdev->stat_cfg);
	VL53LX_general_config_t       *pgeneral      = &(pdev->gen_cfg);
	VL53LX_timing_config_t        *ptiming       = &(pdev->tim_cfg);
	VL53LX_dynamic_config_t       *pdynamic      = &(pdev->dyn_cfg);
	VL53LX_system_control_t       *psystem       = &(pdev->sys_ctrl);

	VL53LX_ll_driver_state_t  *pstate   = &(pdev->ll_state);
	VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);

	uint8_t  *pbuffer                   = &buffer[0];
	uint16_t i                          = 0;
	uint16_t i2c_index                  = 0;
	uint16_t i2c_buffer_offset_bytes    = 0;
	uint16_t i2c_buffer_size_bytes      = 0;

	LOG_FUNCTION_START("");


	pdev->measurement_mode = measurement_mode;



	psystem->system__mode_start =
		(psystem->system__mode_start &
		VL53LX_DEVICEMEASUREMENTMODE_STOP_MASK) |
		measurement_mode;



	status =
		VL53LX_set_user_zone(
		Dev,
		&(pdev->zone_cfg.user_zones[pdev->ll_state.cfg_zone_id]));


	if (pdev->zone_cfg.active_zones > 0) {
		status =
		VL53LX_set_zone_dss_config(
		Dev,
		&(pres->zone_dyn_cfgs.VL53LX_p_003[pdev->ll_state.cfg_zone_id])
		);
	}




	if (((pdev->sys_ctrl.system__mode_start &
		VL53LX_DEVICESCHEDULERMODE_HISTOGRAM) == 0x00) &&
		(pdev->VL53LX_LLDriverCommonData->xtalk_cfg.global_crosstalk_compensation_enable
				== 0x01)) {
		pdev->stat_cfg.algo__range_ignore_threshold_mcps =
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps;
	}





	if (pdev->low_power_auto_data.low_power_auto_range_count == 0xFF)
		pdev->low_power_auto_data.low_power_auto_range_count = 0x0;


	if ((pdev->low_power_auto_data.is_low_power_auto_mode == 1) &&
		(pdev->low_power_auto_data.low_power_auto_range_count == 0)) {

		pdev->low_power_auto_data.saved_interrupt_config =
			pdev->gen_cfg.system__interrupt_config_gpio;

		pdev->gen_cfg.system__interrupt_config_gpio = 1 << 5;

		if ((pdev->dyn_cfg.system__sequence_config & (
			VL53LX_SEQUENCE_MM1_EN | VL53LX_SEQUENCE_MM2_EN)) ==
				0x0) {
			pN->algo__part_to_part_range_offset_mm =
			(pN->mm_config__outer_offset_mm << 2);
		} else {
			pN->algo__part_to_part_range_offset_mm = 0x0;
		}


		if (device_config_level <
				VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS) {
			device_config_level =
				VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS;
		}
	}

	if ((pdev->low_power_auto_data.is_low_power_auto_mode == 1) &&
		(pdev->low_power_auto_data.low_power_auto_range_count == 1)) {

		pdev->gen_cfg.system__interrupt_config_gpio =
			pdev->low_power_auto_data.saved_interrupt_config;


		device_config_level = VL53LX_DEVICECONFIGLEVEL_FULL;
	}





	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_save_cfg_data(Dev);



	switch (device_config_level) {
	case VL53LX_DEVICECONFIGLEVEL_FULL:
		i2c_index = VL53LX_STATIC_NVM_MANAGED_I2C_INDEX;
		break;
	case VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS:
		i2c_index = VL53LX_CUSTOMER_NVM_MANAGED_I2C_INDEX;
		break;
	case VL53LX_DEVICECONFIGLEVEL_STATIC_ONWARDS:
		i2c_index = VL53LX_STATIC_CONFIG_I2C_INDEX;
		break;
	case VL53LX_DEVICECONFIGLEVEL_GENERAL_ONWARDS:
		i2c_index = VL53LX_GENERAL_CONFIG_I2C_INDEX;
		break;
	case VL53LX_DEVICECONFIGLEVEL_TIMING_ONWARDS:
		i2c_index = VL53LX_TIMING_CONFIG_I2C_INDEX;
		break;
	case VL53LX_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS:
		i2c_index = VL53LX_DYNAMIC_CONFIG_I2C_INDEX;
		break;
	default:
		i2c_index = VL53LX_SYSTEM_CONTROL_I2C_INDEX;
		break;
	}



	i2c_buffer_size_bytes =
			(VL53LX_SYSTEM_CONTROL_I2C_INDEX +
			VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES) -
			i2c_index;



	pbuffer = &buffer[0];
	for (i = 0; i < i2c_buffer_size_bytes; i++)
		*pbuffer++ = 0;



	if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_FULL &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
			VL53LX_STATIC_NVM_MANAGED_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_encode_static_nvm_managed(
				pstatic_nvm,
				VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}

	if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_CUSTOMER_ONWARDS &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
			VL53LX_CUSTOMER_NVM_MANAGED_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_encode_customer_nvm_managed(
				pcustomer_nvm,
				VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}

	if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_STATIC_ONWARDS &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
			VL53LX_STATIC_CONFIG_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_encode_static_config(
				pstatic,
				VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}

	if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_GENERAL_ONWARDS &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
				VL53LX_GENERAL_CONFIG_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_encode_general_config(
				pgeneral,
				VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}

	if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_TIMING_ONWARDS &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
				VL53LX_TIMING_CONFIG_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_encode_timing_config(
				ptiming,
				VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}

	if (device_config_level >= VL53LX_DEVICECONFIGLEVEL_DYNAMIC_ONWARDS &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
			VL53LX_DYNAMIC_CONFIG_I2C_INDEX - i2c_index;


		if ((psystem->system__mode_start &
			VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK) ==
			VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK) {
			pdynamic->system__grouped_parameter_hold_0 =
					pstate->cfg_gph_id | 0x01;
			pdynamic->system__grouped_parameter_hold_1 =
					pstate->cfg_gph_id | 0x01;
			pdynamic->system__grouped_parameter_hold   =
					pstate->cfg_gph_id;
		}
		status =
			VL53LX_i2c_encode_dynamic_config(
				pdynamic,
				VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}

	if (status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
				VL53LX_SYSTEM_CONTROL_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_encode_system_control(
				psystem,
				VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes]);
	}



	if (status == VL53LX_ERROR_NONE) {
		status =
			VL53LX_WriteMulti(
				Dev,
				i2c_index,
				buffer,
				(uint32_t)i2c_buffer_size_bytes);
	}


	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_update_ll_driver_rd_state(Dev);

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_update_ll_driver_cfg_state(Dev);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_stop_range(
	VL53LX_DEV     Dev)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);



	pdev->sys_ctrl.system__mode_start =
			(pdev->sys_ctrl.system__mode_start &
				VL53LX_DEVICEMEASUREMENTMODE_STOP_MASK) |
			 VL53LX_DEVICEMEASUREMENTMODE_ABORT;

	status = VL53LX_set_system_control(
				Dev,
				&pdev->sys_ctrl);


	pdev->sys_ctrl.system__mode_start =
			(pdev->sys_ctrl.system__mode_start &
				VL53LX_DEVICEMEASUREMENTMODE_STOP_MASK);


	VL53LX_init_ll_driver_state(
			Dev,
			VL53LX_DEVICESTATE_SW_STANDBY);


	V53L1_init_zone_results_structure(
			pdev->zone_cfg.active_zones+1,
			&(pres->zone_results));


	V53L1_init_zone_dss_configs(Dev);


	if (pdev->low_power_auto_data.is_low_power_auto_mode == 1)
		VL53LX_low_power_auto_data_stop_range(Dev);

	return status;
}


VL53LX_Error VL53LX_get_measurement_results(
	VL53LX_DEV                     Dev,
	VL53LX_DeviceResultsLevel      device_results_level)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	uint8_t buffer[VL53LX_MAX_I2C_XFER_SIZE];

	VL53LX_system_results_t   *psystem_results = &(pdev->sys_results);
	VL53LX_core_results_t     *pcore_results   = &(pdev->VL53LX_LLDriverCommonData->core_results);
	VL53LX_debug_results_t    *pdebug_results  = &(pdev->VL53LX_LLDriverCommonData->dbg_results);

	uint16_t i2c_index               = VL53LX_SYSTEM_RESULTS_I2C_INDEX;
	uint16_t i2c_buffer_offset_bytes = 0;
	uint16_t i2c_buffer_size_bytes   = 0;

	LOG_FUNCTION_START("");



	switch (device_results_level) {
	case VL53LX_DEVICERESULTSLEVEL_FULL:
		i2c_buffer_size_bytes =
				(VL53LX_DEBUG_RESULTS_I2C_INDEX +
				VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES) -
				i2c_index;
		break;
	case VL53LX_DEVICERESULTSLEVEL_UPTO_CORE:
		i2c_buffer_size_bytes =
				(VL53LX_CORE_RESULTS_I2C_INDEX +
				VL53LX_CORE_RESULTS_I2C_SIZE_BYTES) -
				i2c_index;
		break;
	default:
		i2c_buffer_size_bytes =
				VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES;
		break;
	}



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_ReadMulti(
				Dev,
				i2c_index,
				buffer,
				(uint32_t)i2c_buffer_size_bytes);



	if (device_results_level >= VL53LX_DEVICERESULTSLEVEL_FULL &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
				VL53LX_DEBUG_RESULTS_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_decode_debug_results(
				VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes],
				pdebug_results);
	}

	if (device_results_level >= VL53LX_DEVICERESULTSLEVEL_UPTO_CORE &&
		status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes =
				VL53LX_CORE_RESULTS_I2C_INDEX - i2c_index;

		status =
			VL53LX_i2c_decode_core_results(
				VL53LX_CORE_RESULTS_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes],
				pcore_results);
	}

	if (status == VL53LX_ERROR_NONE) {

		i2c_buffer_offset_bytes = 0;
		status =
			VL53LX_i2c_decode_system_results(
				VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES,
				&buffer[i2c_buffer_offset_bytes],
				psystem_results);
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_device_results(
	VL53LX_DEV                    Dev,
	VL53LX_DeviceResultsLevel     device_results_level,
	VL53LX_range_results_t       *prange_results)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	VL53LX_range_results_t   *presults =
			&(pres->range_results);
	VL53LX_zone_objects_t    *pobjects =
			&(pres->zone_results.VL53LX_p_003[0]);
	VL53LX_ll_driver_state_t *pstate   =
			&(pdev->ll_state);
	VL53LX_zone_config_t     *pzone_cfg =
			&(pdev->zone_cfg);
	VL53LX_zone_hist_info_t  *phist_info =
			&(pres->zone_hists.VL53LX_p_003[0]);

	VL53LX_dmax_calibration_data_t   dmax_cal;
	VL53LX_dmax_calibration_data_t *pdmax_cal = &dmax_cal;
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_xtalk_config_t *pC = &(pdev->VL53LX_LLDriverCommonData->xtalk_cfg);
	VL53LX_low_power_auto_data_t *pL = &(pdev->low_power_auto_data);
	VL53LX_histogram_bin_data_t *pHD = &(pdev->VL53LX_LLDriverCommonData->hist_data);
	VL53LX_customer_nvm_managed_t *pN = &(pdev->customer);
	VL53LX_zone_histograms_t *pZH = &(pres->zone_hists);
	VL53LX_xtalk_calibration_results_t *pXCR = &(pdev->xtalk_cal);
	uint8_t tmp8;
	uint8_t zid;
	uint8_t i;
	uint8_t histo_merge_nb, idx;
	VL53LX_range_data_t *pdata;

	LOG_FUNCTION_START("");


	if ((pdev->sys_ctrl.system__mode_start &
		 VL53LX_DEVICESCHEDULERMODE_HISTOGRAM)
		 == VL53LX_DEVICESCHEDULERMODE_HISTOGRAM) {



		status = VL53LX_get_histogram_bin_data(
						Dev,
						&(pdev->VL53LX_LLDriverCommonData->hist_data));




		if (status == VL53LX_ERROR_NONE &&
			pHD->number_of_ambient_bins == 0) {
			zid = pdev->ll_state.rd_zone_id;
			status = VL53LX_hist_copy_and_scale_ambient_info(
			&(pZH->VL53LX_p_003[zid]),
			&(pdev->VL53LX_LLDriverCommonData->hist_data));
		}


		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;

		VL53LX_compute_histo_merge_nb(Dev, &histo_merge_nb);
		if (histo_merge_nb == 0)
			histo_merge_nb = 1;
		idx = histo_merge_nb - 1;
		if (pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge == 1)
			pC->algo__crosstalk_compensation_plane_offset_kcps =
				pXCR->algo__xtalk_cpo_HistoMerge_kcps[idx];

		pHP->gain_factor =
			pdev->gain_cal.histogram_ranging_gain_factor;

		pHP->algo__crosstalk_compensation_plane_offset_kcps =
		VL53LX_calc_crosstalk_plane_offset_with_margin(
		pC->algo__crosstalk_compensation_plane_offset_kcps,
		pC->histogram_mode_crosstalk_margin_kcps);

		pHP->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_x_plane_gradient_kcps;
		pHP->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pC->algo__crosstalk_compensation_y_plane_gradient_kcps;

		pdev->VL53LX_LLDriverCommonData->dmax_cfg.ambient_thresh_sigma =
			pHP->ambient_thresh_sigma1;
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.min_ambient_thresh_events =
			pHP->min_ambient_thresh_events;
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.signal_total_events_limit =
			pHP->signal_total_events_limit;
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.dss_config__target_total_rate_mcps =
			pdev->stat_cfg.dss_config__target_total_rate_mcps;
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.dss_config__aperture_attenuation =
			pdev->gen_cfg.dss_config__aperture_attenuation;

		pHP->algo__crosstalk_detect_max_valid_range_mm =
			pC->algo__crosstalk_detect_max_valid_range_mm;
		pHP->algo__crosstalk_detect_min_valid_range_mm =
			pC->algo__crosstalk_detect_min_valid_range_mm;
		pHP->algo__crosstalk_detect_max_valid_rate_kcps =
			pC->algo__crosstalk_detect_max_valid_rate_kcps;
		pHP->algo__crosstalk_detect_max_sigma_mm =
			pC->algo__crosstalk_detect_max_sigma_mm;



		VL53LX_copy_rtn_good_spads_to_buffer(
				&(pdev->nvm_copy_data),
				&(pdev->rtn_good_spads[0]));



		switch (pdev->offset_correction_mode) {

		case VL53LX_OFFSETCORRECTIONMODE__MM1_MM2_OFFSETS:
			tmp8 = pdev->gen_cfg.dss_config__aperture_attenuation;

			VL53LX_hist_combine_mm1_mm2_offsets(
			pN->mm_config__inner_offset_mm,
			pN->mm_config__outer_offset_mm,
			pdev->nvm_copy_data.roi_config__mode_roi_centre_spad,
			pdev->nvm_copy_data.roi_config__mode_roi_xy_size,
			pHD->roi_config__user_roi_centre_spad,
			pHD->roi_config__user_roi_requested_global_xy_size,
			&(pdev->add_off_cal_data),
			&(pdev->rtn_good_spads[0]),
			(uint16_t)tmp8,
			&(pHP->range_offset_mm));
		break;
		case VL53LX_OFFSETCORRECTIONMODE__PER_VCSEL_OFFSETS:
			select_offset_per_vcsel(
			pdev,
			&(pHP->range_offset_mm));
			pHP->range_offset_mm *= 4;
		break;
		default:
			pHP->range_offset_mm = 0;
		break;

		}



		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;


		VL53LX_calc_max_effective_spads(
		pHD->roi_config__user_roi_centre_spad,
		pHD->roi_config__user_roi_requested_global_xy_size,
		&(pdev->rtn_good_spads[0]),
		(uint16_t)pdev->gen_cfg.dss_config__aperture_attenuation,
		&(pdev->VL53LX_LLDriverCommonData->dmax_cfg.max_effective_spads));

		status =
			VL53LX_get_dmax_calibration_data(
				Dev,
				pdev->dmax_mode,
				pdmax_cal);


		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;

		status = VL53LX_ipp_hist_process_data(
				Dev,
				pdmax_cal,
				&(pdev->VL53LX_LLDriverCommonData->dmax_cfg),
				&(pdev->histpostprocess),
				&(pdev->VL53LX_LLDriverCommonData->hist_data),
				&(pdev->xtalk_shapes),
				pdev->VL53LX_LLDriverCommonData->wArea1,
				pdev->VL53LX_LLDriverCommonData->wArea2,
				&histo_merge_nb,
				presults);

		if ((pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge == 1) &&
			(histo_merge_nb > 1))
		for (i = 0; i < VL53LX_MAX_RANGE_RESULTS; i++) {
			pdata = &(presults->VL53LX_p_003[i]);
			pdata->VL53LX_p_016 /= histo_merge_nb;
			pdata->VL53LX_p_017 /= histo_merge_nb;
			pdata->VL53LX_p_010 /= histo_merge_nb;
			pdata->peak_signal_count_rate_mcps /= histo_merge_nb;
			pdata->avg_signal_count_rate_mcps /= histo_merge_nb;
			pdata->ambient_count_rate_mcps /= histo_merge_nb;
			pdata->VL53LX_p_009 /= histo_merge_nb;
		}


		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;

		status = VL53LX_hist_wrap_dmax(
				&(pdev->histpostprocess),
				&(pdev->VL53LX_LLDriverCommonData->hist_data),
				&(presults->wrap_dmax_mm));


		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;

		zid = pdev->ll_state.rd_zone_id;
		status = VL53LX_hist_phase_consistency_check(
			Dev,
			&(pZH->VL53LX_p_003[zid]),
			&(pres->zone_results.VL53LX_p_003[zid]),
			presults);


		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;

		zid = pdev->ll_state.rd_zone_id;
		status = VL53LX_hist_xmonitor_consistency_check(
			Dev,
			&(pZH->VL53LX_p_003[zid]),
			&(pres->zone_results.VL53LX_p_003[zid]),
			&(presults->xmonitor));


		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;


		zid = pdev->ll_state.rd_zone_id;
		pZH->max_zones    = VL53LX_MAX_USER_ZONES;
		pZH->active_zones =
				pdev->zone_cfg.active_zones+1;
		pHD->zone_id       = zid;

		if (zid <
				pres->zone_results.max_zones) {

			phist_info =
			&(pZH->VL53LX_p_003[zid]);

			phist_info->rd_device_state =
				pHD->rd_device_state;

			phist_info->number_of_ambient_bins =
				pHD->number_of_ambient_bins;

			phist_info->result__dss_actual_effective_spads =
			pHD->result__dss_actual_effective_spads;

			phist_info->VL53LX_p_005 =
				pHD->VL53LX_p_005;

			phist_info->total_periods_elapsed =
				pHD->total_periods_elapsed;

			phist_info->ambient_events_sum =
				pHD->ambient_events_sum;
		}



		if (status != VL53LX_ERROR_NONE)
			goto UPDATE_DYNAMIC_CONFIG;

		VL53LX_hist_copy_results_to_sys_and_core(
				&(pdev->VL53LX_LLDriverCommonData->hist_data),
				presults,
				&(pdev->sys_results),
				&(pdev->VL53LX_LLDriverCommonData->core_results));


UPDATE_DYNAMIC_CONFIG:
		if (pzone_cfg->active_zones > 0) {
			if (pstate->rd_device_state !=
				VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {
				if (status == VL53LX_ERROR_NONE) {
					status = VL53LX_dynamic_zone_update(
						Dev, presults);
				}
			}


			for (i = 0; i < VL53LX_MAX_USER_ZONES; i++) {
				pzone_cfg->bin_config[i] =
				((pdev->ll_state.cfg_internal_stream_count)
						& 0x01) ?
					VL53LX_ZONECONFIG_BINCONFIG__HIGHAMB :
					VL53LX_ZONECONFIG_BINCONFIG__LOWAMB;
			}

			if (status == VL53LX_ERROR_NONE)
				status = VL53LX_multizone_hist_bins_update(Dev);

		}



		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_dynamic_xtalk_correction_corrector(Dev);

#ifdef VL53LX_LOG_ENABLE
		if (status == VL53LX_ERROR_NONE)
			VL53LX_print_histogram_bin_data(
				&(pdev->VL53LX_LLDriverCommonData->hist_data),
				"get_device_results():pdev->lldata.hist_data.",
				VL53LX_TRACE_MODULE_HISTOGRAM_DATA);
#endif

		if (pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge == 1)
			pC->algo__crosstalk_compensation_plane_offset_kcps =
				pXCR->algo__xtalk_cpo_HistoMerge_kcps[0];
	} else {

		if (status == VL53LX_ERROR_NONE)
			status = VL53LX_get_measurement_results(
							Dev,
							device_results_level);

		if (status == VL53LX_ERROR_NONE)
			VL53LX_copy_sys_and_core_results_to_range_results(
			(int32_t)pdev->gain_cal.standard_ranging_gain_factor,
			&(pdev->sys_results),
			&(pdev->VL53LX_LLDriverCommonData->core_results),
			presults);



		if (pL->is_low_power_auto_mode == 1) {

			if ((status == VL53LX_ERROR_NONE) &&
				(pL->low_power_auto_range_count == 0)) {

				status =
				VL53LX_low_power_auto_setup_manual_calibration(
						Dev);
				pL->low_power_auto_range_count = 1;
			} else if ((status == VL53LX_ERROR_NONE) &&
				(pL->low_power_auto_range_count == 1)) {
				pL->low_power_auto_range_count = 2;
			}


			if ((pL->low_power_auto_range_count != 0xFF) &&
				(status == VL53LX_ERROR_NONE)) {
				status = VL53LX_low_power_auto_update_DSS(
						Dev);
			}
		}

	}


	presults->cfg_device_state = pdev->ll_state.cfg_device_state;
	presults->rd_device_state  = pdev->ll_state.rd_device_state;
	presults->zone_id          = pdev->ll_state.rd_zone_id;

	if (status == VL53LX_ERROR_NONE) {


		pres->zone_results.max_zones    = VL53LX_MAX_USER_ZONES;
		pres->zone_results.active_zones = pdev->zone_cfg.active_zones+1;
		zid = pdev->ll_state.rd_zone_id;

		if (zid < pres->zone_results.max_zones) {

			pobjects =
			&(pres->zone_results.VL53LX_p_003[zid]);

			pobjects->cfg_device_state  =
					presults->cfg_device_state;
			pobjects->rd_device_state   = presults->rd_device_state;
			pobjects->zone_id           = presults->zone_id;
			pobjects->stream_count      = presults->stream_count;



			pobjects->xmonitor.VL53LX_p_016 =
				presults->xmonitor.VL53LX_p_016;
			pobjects->xmonitor.VL53LX_p_017 =
				presults->xmonitor.VL53LX_p_017;
			pobjects->xmonitor.VL53LX_p_011 =
				presults->xmonitor.VL53LX_p_011;
			pobjects->xmonitor.range_status =
				presults->xmonitor.range_status;

			pobjects->max_objects      = presults->max_results;
			pobjects->active_objects   = presults->active_results;

			for (i = 0; i < presults->active_results; i++) {
				pobjects->VL53LX_p_003[i].VL53LX_p_016 =
					presults->VL53LX_p_003[i].VL53LX_p_016;
				pobjects->VL53LX_p_003[i].VL53LX_p_017 =
					presults->VL53LX_p_003[i].VL53LX_p_017;
				pobjects->VL53LX_p_003[i].VL53LX_p_011 =
					presults->VL53LX_p_003[i].VL53LX_p_011;
				pobjects->VL53LX_p_003[i].range_status =
					presults->VL53LX_p_003[i].range_status;
			}


		}
	}



	memcpy(
		prange_results,
		presults,
		sizeof(VL53LX_range_results_t));



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_check_ll_driver_rd_state(Dev);

#ifdef VL53LX_LOG_ENABLE
	if (status == VL53LX_ERROR_NONE)
		VL53LX_print_range_results(
			presults,
			"get_device_results():pdev->llresults.range_results.",
			VL53LX_TRACE_MODULE_RANGE_RESULTS_DATA);
#endif

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_clear_interrupt_and_enable_next_range(
	VL53LX_DEV        Dev,
	uint8_t           measurement_mode)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");










	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_init_and_start_range(
				Dev,
				measurement_mode,
				VL53LX_DEVICECONFIGLEVEL_GENERAL_ONWARDS);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_histogram_bin_data(
		VL53LX_DEV                   Dev,
		VL53LX_histogram_bin_data_t *pdata)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	VL53LX_zone_private_dyn_cfg_t *pzone_dyn_cfg;

	VL53LX_static_nvm_managed_t   *pstat_nvm = &(pdev->stat_nvm);
	VL53LX_static_config_t        *pstat_cfg = &(pdev->stat_cfg);
	VL53LX_general_config_t       *pgen_cfg  = &(pdev->gen_cfg);
	VL53LX_timing_config_t        *ptim_cfg  = &(pdev->tim_cfg);
	VL53LX_range_results_t        *presults  = &(pres->range_results);

	uint8_t    buffer[VL53LX_MAX_I2C_XFER_SIZE];
	uint8_t   *pbuffer = &buffer[0];
	uint8_t    bin_23_0 = 0x00;
	uint16_t   bin                      = 0;
	uint16_t   i2c_buffer_offset_bytes  = 0;
	uint16_t   encoded_timeout          = 0;

	uint32_t   pll_period_us            = 0;
	uint32_t   periods_elapsed_tmp      = 0;

	uint8_t    i                        = 0;

	int32_t    hist_merge				= 0;

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_ReadMulti(
			Dev,
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX,
			pbuffer,
			VL53LX_HISTOGRAM_BIN_DATA_I2C_SIZE_BYTES);



	pdata->result__interrupt_status               = *(pbuffer +   0);
	pdata->result__range_status                   = *(pbuffer +   1);
	pdata->result__report_status                  = *(pbuffer +   2);
	pdata->result__stream_count                   = *(pbuffer +   3);
	pdata->result__dss_actual_effective_spads =
		VL53LX_i2c_decode_uint16_t(2, pbuffer +   4);



	i2c_buffer_offset_bytes =
			VL53LX_PHASECAL_RESULT__REFERENCE_PHASE -
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

	pbuffer = &buffer[i2c_buffer_offset_bytes];

	pdata->phasecal_result__reference_phase =
			VL53LX_i2c_decode_uint16_t(2, pbuffer);

	i2c_buffer_offset_bytes =
			VL53LX_PHASECAL_RESULT__VCSEL_START -
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

	pdata->phasecal_result__vcsel_start = buffer[i2c_buffer_offset_bytes];



	pdev->VL53LX_LLDriverCommonData->dbg_results.phasecal_result__reference_phase =
			pdata->phasecal_result__reference_phase;
	pdev->VL53LX_LLDriverCommonData->dbg_results.phasecal_result__vcsel_start =
			pdata->phasecal_result__vcsel_start;



	i2c_buffer_offset_bytes =
			VL53LX_RESULT__HISTOGRAM_BIN_23_0_MSB -
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

	bin_23_0 = buffer[i2c_buffer_offset_bytes] << 2;

	i2c_buffer_offset_bytes =
			VL53LX_RESULT__HISTOGRAM_BIN_23_0_LSB -
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

	bin_23_0 += buffer[i2c_buffer_offset_bytes];

	i2c_buffer_offset_bytes =
			VL53LX_RESULT__HISTOGRAM_BIN_23_0 -
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

	buffer[i2c_buffer_offset_bytes] = bin_23_0;



	i2c_buffer_offset_bytes =
			VL53LX_RESULT__HISTOGRAM_BIN_0_2 -
			VL53LX_HISTOGRAM_BIN_DATA_I2C_INDEX;

	pbuffer = &buffer[i2c_buffer_offset_bytes];
	for (bin = 0; bin < VL53LX_HISTOGRAM_BUFFER_SIZE; bin++) {
		pdata->bin_data[bin] =
			(int32_t)VL53LX_i2c_decode_uint32_t(3, pbuffer);
		pbuffer += 3;
	}




	VL53LX_get_tuning_parm(Dev, VL53LX_TUNINGPARM_HIST_MERGE, &hist_merge);

	if (pdata->result__stream_count == 0) {

		memset(pdev->multi_bins_rec, 0, sizeof(pdev->multi_bins_rec));
		pdev->bin_rec_pos = 0;
		pdev->pos_before_next_recom = 0;
	}

	if (hist_merge == 1)
		vl53lx_histo_merge(Dev, pdata);


	pdata->zone_id                 = pdev->ll_state.rd_zone_id;
	pdata->VL53LX_p_019               = 0;
	pdata->VL53LX_p_020             = VL53LX_HISTOGRAM_BUFFER_SIZE;
	pdata->VL53LX_p_021          = VL53LX_HISTOGRAM_BUFFER_SIZE;

	pdata->cal_config__vcsel_start = pgen_cfg->cal_config__vcsel_start;



	pdata->vcsel_width =
		((uint16_t)pgen_cfg->global_config__vcsel_width) << 4;
	pdata->vcsel_width +=
		(uint16_t)pstat_cfg->ana_config__vcsel_pulse_width_offset;


	pdata->VL53LX_p_015 =
		pstat_nvm->osc_measured__fast_osc__frequency;



	VL53LX_hist_get_bin_sequence_config(Dev, pdata);



	if (pdev->ll_state.rd_timing_status == 0) {

		encoded_timeout =
			(ptim_cfg->range_config__timeout_macrop_a_hi << 8)
			+ ptim_cfg->range_config__timeout_macrop_a_lo;
		pdata->VL53LX_p_005 =  ptim_cfg->range_config__vcsel_period_a;
	} else {

		encoded_timeout =
			(ptim_cfg->range_config__timeout_macrop_b_hi << 8)
			+ ptim_cfg->range_config__timeout_macrop_b_lo;
		pdata->VL53LX_p_005 = ptim_cfg->range_config__vcsel_period_b;
	}



	pdata->number_of_ambient_bins  = 0;

	for (i = 0; i < 6; i++) {
		if ((pdata->bin_seq[i] & 0x07) == 0x07)
			pdata->number_of_ambient_bins  =
					pdata->number_of_ambient_bins + 0x04;
	}

	pdata->total_periods_elapsed =
		VL53LX_decode_timeout(encoded_timeout);




	pll_period_us =
		VL53LX_calc_pll_period_us(pdata->VL53LX_p_015);



	periods_elapsed_tmp = pdata->total_periods_elapsed + 1;



	pdata->peak_duration_us =
		VL53LX_duration_maths(
			pll_period_us,
			(uint32_t)pdata->vcsel_width,
			VL53LX_RANGING_WINDOW_VCSEL_PERIODS,
			periods_elapsed_tmp);

	pdata->woi_duration_us     = 0;



	VL53LX_hist_calc_zero_distance_phase(pdata);



	VL53LX_hist_estimate_ambient_from_ambient_bins(pdata);



	pdata->cfg_device_state = pdev->ll_state.cfg_device_state;
	pdata->rd_device_state  = pdev->ll_state.rd_device_state;



	pzone_dyn_cfg = &(pres->zone_dyn_cfgs.VL53LX_p_003[pdata->zone_id]);

	pdata->roi_config__user_roi_centre_spad =
		pzone_dyn_cfg->roi_config__user_roi_centre_spad;
	pdata->roi_config__user_roi_requested_global_xy_size =
		pzone_dyn_cfg->roi_config__user_roi_requested_global_xy_size;



	presults->device_status = VL53LX_DEVICEERROR_NOUPDATE;



	switch (pdata->result__range_status &
			VL53LX_RANGE_STATUS__RANGE_STATUS_MASK) {

	case VL53LX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
	case VL53LX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
	case VL53LX_DEVICEERROR_NOVHVVALUEFOUND:
	case VL53LX_DEVICEERROR_USERROICLIP:
	case VL53LX_DEVICEERROR_MULTCLIPFAIL:

		presults->device_status = (pdata->result__range_status &
				VL53LX_RANGE_STATUS__RANGE_STATUS_MASK);

		status = VL53LX_ERROR_RANGE_ERROR;

	break;

	}

	LOG_FUNCTION_END(status);

	return status;
}


void VL53LX_copy_sys_and_core_results_to_range_results(
	int32_t                           gain_factor,
	VL53LX_system_results_t          *psys,
	VL53LX_core_results_t            *pcore,
	VL53LX_range_results_t           *presults)
{
	uint8_t  i = 0;

	VL53LX_range_data_t *pdata;
	int32_t range_mm = 0;
	uint32_t tmpu32 = 0;
	uint16_t rpscr_crosstalk_corrected_mcps_sd0;
	uint16_t rmmo_effective_spads_sd0;
	uint16_t rmmi_effective_spads_sd0;

	LOG_FUNCTION_START("");



	presults->zone_id         = 0;
	presults->stream_count    = psys->result__stream_count;
	presults->wrap_dmax_mm    = 0;
	presults->max_results     = VL53LX_MAX_RANGE_RESULTS;
	presults->active_results  = 1;
	rpscr_crosstalk_corrected_mcps_sd0 =
	psys->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
	rmmo_effective_spads_sd0 =
			psys->result__mm_outer_actual_effective_spads_sd0;
	rmmi_effective_spads_sd0 =
			psys->result__mm_inner_actual_effective_spads_sd0;


	for (i = 0; i < VL53LX_MAX_AMBIENT_DMAX_VALUES; i++)
		presults->VL53LX_p_022[i] = 0;

	pdata = &(presults->VL53LX_p_003[0]);

	for (i = 0; i < 2; i++) {

		pdata->range_id     = i;
		pdata->time_stamp   = 0;

		if ((psys->result__stream_count == 0) &&
			((psys->result__range_status &
				VL53LX_RANGE_STATUS__RANGE_STATUS_MASK) ==
			VL53LX_DEVICEERROR_RANGECOMPLETE)) {
			pdata->range_status =
				VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK;
		} else {
			pdata->range_status =
				psys->result__range_status &
				VL53LX_RANGE_STATUS__RANGE_STATUS_MASK;
		}

		pdata->VL53LX_p_012 = 0;
		pdata->VL53LX_p_019    = 0;
		pdata->VL53LX_p_023   = 0;
		pdata->VL53LX_p_024     = 0;
		pdata->VL53LX_p_013   = 0;
		pdata->VL53LX_p_025    = 0;

		switch (i) {

		case 0:
			if (psys->result__report_status ==
				VL53LX_DEVICEREPORTSTATUS_MM1)
				pdata->VL53LX_p_004 =
						rmmi_effective_spads_sd0;
			else if (psys->result__report_status ==
					VL53LX_DEVICEREPORTSTATUS_MM2)
				pdata->VL53LX_p_004 =
						rmmo_effective_spads_sd0;
			else
				pdata->VL53LX_p_004 =
				psys->result__dss_actual_effective_spads_sd0;

			pdata->peak_signal_count_rate_mcps =
					rpscr_crosstalk_corrected_mcps_sd0;
			pdata->avg_signal_count_rate_mcps =
				psys->result__avg_signal_count_rate_mcps_sd0;
			pdata->ambient_count_rate_mcps =
				psys->result__ambient_count_rate_mcps_sd0;




			tmpu32 = ((uint32_t)psys->result__sigma_sd0 << 5);
			if (tmpu32 > 0xFFFF)
				tmpu32 = 0xFFFF;

			pdata->VL53LX_p_002 = (uint16_t)tmpu32;



			pdata->VL53LX_p_011 =
				psys->result__phase_sd0;

			range_mm = (int32_t)(
			psys->result__final_crosstalk_corrected_range_mm_sd0);


			range_mm *= gain_factor;
			range_mm += 0x0400;
			range_mm /= 0x0800;

			pdata->median_range_mm = (int16_t)range_mm;

			pdata->VL53LX_p_017 =
				pcore->result_core__ranging_total_events_sd0;
			pdata->VL53LX_p_010 =
				pcore->result_core__signal_total_events_sd0;
			pdata->total_periods_elapsed =
				pcore->result_core__total_periods_elapsed_sd0;
			pdata->VL53LX_p_016 =
				pcore->result_core__ambient_window_events_sd0;

			break;
		case 1:

			pdata->VL53LX_p_004 =
				psys->result__dss_actual_effective_spads_sd1;
			pdata->peak_signal_count_rate_mcps =
				psys->result__peak_signal_count_rate_mcps_sd1;
			pdata->avg_signal_count_rate_mcps =
				0xFFFF;
			pdata->ambient_count_rate_mcps =
				psys->result__ambient_count_rate_mcps_sd1;




			tmpu32 = ((uint32_t)psys->result__sigma_sd1 << 5);
			if (tmpu32 > 0xFFFF)
				tmpu32 = 0xFFFF;

			pdata->VL53LX_p_002 = (uint16_t)tmpu32;



			pdata->VL53LX_p_011 =
				psys->result__phase_sd1;

			range_mm = (int32_t)(
			psys->result__final_crosstalk_corrected_range_mm_sd1);


			range_mm *= gain_factor;
			range_mm += 0x0400;
			range_mm /= 0x0800;

			pdata->median_range_mm = (int16_t)range_mm;

			pdata->VL53LX_p_017 =
				pcore->result_core__ranging_total_events_sd1;
			pdata->VL53LX_p_010 =
				pcore->result_core__signal_total_events_sd1;
			pdata->total_periods_elapsed  =
				pcore->result_core__total_periods_elapsed_sd1;
			pdata->VL53LX_p_016 =
				pcore->result_core__ambient_window_events_sd1;

			break;
		}


		pdata->VL53LX_p_026    = pdata->VL53LX_p_011;
		pdata->VL53LX_p_027    = pdata->VL53LX_p_011;
		pdata->min_range_mm = pdata->median_range_mm;
		pdata->max_range_mm = pdata->median_range_mm;

		pdata++;
	}



	presults->device_status = VL53LX_DEVICEERROR_NOUPDATE;



	switch (psys->result__range_status &
			VL53LX_RANGE_STATUS__RANGE_STATUS_MASK) {

	case VL53LX_DEVICEERROR_VCSELCONTINUITYTESTFAILURE:
	case VL53LX_DEVICEERROR_VCSELWATCHDOGTESTFAILURE:
	case VL53LX_DEVICEERROR_NOVHVVALUEFOUND:
	case VL53LX_DEVICEERROR_USERROICLIP:
	case VL53LX_DEVICEERROR_MULTCLIPFAIL:

		presults->device_status = (psys->result__range_status &
				VL53LX_RANGE_STATUS__RANGE_STATUS_MASK);

		presults->VL53LX_p_003[0].range_status =
			VL53LX_DEVICEERROR_NOUPDATE;
	break;

	}

	LOG_FUNCTION_END(0);
}


VL53LX_Error VL53LX_set_zone_dss_config(
	VL53LX_DEV                      Dev,
	VL53LX_zone_private_dyn_cfg_t  *pzone_dyn_cfg)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);

	LOG_FUNCTION_START("");

	if (pstate->cfg_device_state ==
		VL53LX_DEVICESTATE_RANGING_DSS_MANUAL) {
		pdev->gen_cfg.dss_config__roi_mode_control =
		VL53LX_DSS_CONTROL__MODE_EFFSPADS;
		pdev->gen_cfg.dss_config__manual_effective_spads_select =
			pzone_dyn_cfg->dss_requested_effective_spad_count;
	} else {
		pdev->gen_cfg.dss_config__roi_mode_control =
			VL53LX_DSS_CONTROL__MODE_TARGET_RATE;
	}

	LOG_FUNCTION_END(status);
	return status;
}


VL53LX_Error VL53LX_set_dmax_mode(
	VL53LX_DEV               Dev,
	VL53LX_DeviceDmaxMode    dmax_mode)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->dmax_mode = dmax_mode;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_dmax_mode(
	VL53LX_DEV               Dev,
	VL53LX_DeviceDmaxMode   *pdmax_mode)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	*pdmax_mode = pdev->dmax_mode;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_dmax_calibration_data(
	VL53LX_DEV                      Dev,
	VL53LX_DeviceDmaxMode           dmax_mode,
	VL53LX_dmax_calibration_data_t *pdmax_cal)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t    *pdev =
		VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	switch (dmax_mode) {

	case VL53LX_DEVICEDMAXMODE__CUST_CAL_DATA:
		memcpy(
			pdmax_cal,
			&(pdev->cust_dmax_cal),
			sizeof(VL53LX_dmax_calibration_data_t));
	break;

	case VL53LX_DEVICEDMAXMODE__FMT_CAL_DATA:
		memcpy(
			pdmax_cal,
			&(pdev->fmt_dmax_cal),
			sizeof(VL53LX_dmax_calibration_data_t));
	break;

	default:
		status = VL53LX_ERROR_INVALID_PARAMS;
	break;

	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_offset_correction_mode(
	VL53LX_DEV                     Dev,
	VL53LX_OffsetCorrectionMode    offset_cor_mode)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->offset_correction_mode = offset_cor_mode;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_offset_correction_mode(
	VL53LX_DEV                     Dev,
	VL53LX_OffsetCorrectionMode   *poffset_cor_mode)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	*poffset_cor_mode = pdev->offset_correction_mode;

	LOG_FUNCTION_END(status);

	return status;
}





VL53LX_Error VL53LX_get_tuning_parm(
	VL53LX_DEV                     Dev,
	VL53LX_TuningParms             tuning_parm_key,
	int32_t                       *ptuning_parm_value)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_xtalkextract_config_t *pXC = &(pdev->VL53LX_LLDriverCommonData->xtalk_extract_cfg);

	LOG_FUNCTION_START("");

	switch (tuning_parm_key) {

	case VL53LX_TUNINGPARM_VERSION:
		*ptuning_parm_value =
			(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_tuning_parm_version;
	break;
	case VL53LX_TUNINGPARM_KEY_TABLE_VERSION:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_tuning_parm_key_table_version;
	break;
	case VL53LX_TUNINGPARM_LLD_VERSION:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_tuning_parm_lld_version;
	break;
	case VL53LX_TUNINGPARM_HIST_ALGO_SELECT:
		*ptuning_parm_value =
				(int32_t)pHP->hist_algo_select;
	break;
	case VL53LX_TUNINGPARM_HIST_TARGET_ORDER:
		*ptuning_parm_value =
				(int32_t)pHP->hist_target_order;
	break;
	case VL53LX_TUNINGPARM_HIST_FILTER_WOI_0:
		*ptuning_parm_value =
				(int32_t)pHP->filter_woi0;
	break;
	case VL53LX_TUNINGPARM_HIST_FILTER_WOI_1:
		*ptuning_parm_value =
				(int32_t)pHP->filter_woi1;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_EST_METHOD:
		*ptuning_parm_value =
				(int32_t)pHP->hist_amb_est_method;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0:
		*ptuning_parm_value =
				(int32_t)pHP->ambient_thresh_sigma0;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1:
		*ptuning_parm_value =
				(int32_t)pHP->ambient_thresh_sigma1;
	break;
	case VL53LX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS:
		*ptuning_parm_value =
				(int32_t)pHP->min_ambient_thresh_events;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_EVENTS_SCALER:
		*ptuning_parm_value =
				(int32_t)pHP->ambient_thresh_events_scaler;
	break;
	case VL53LX_TUNINGPARM_HIST_NOISE_THRESHOLD:
		*ptuning_parm_value =
				(int32_t)pHP->noise_threshold;
	break;
	case VL53LX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT:
		*ptuning_parm_value =
				(int32_t)pHP->signal_total_events_limit;
	break;
	case VL53LX_TUNINGPARM_HIST_SIGMA_EST_REF_MM:
		*ptuning_parm_value =
				(int32_t)pHP->sigma_estimator__sigma_ref_mm;
	break;
	case VL53LX_TUNINGPARM_HIST_SIGMA_THRESH_MM:
		*ptuning_parm_value =
				(int32_t)pHP->sigma_thresh;
	break;
	case VL53LX_TUNINGPARM_HIST_GAIN_FACTOR:
		*ptuning_parm_value =
		(int32_t)pdev->gain_cal.histogram_ranging_gain_factor;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE:
		*ptuning_parm_value =
	(int32_t)pHP->algo__consistency_check__phase_tolerance;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM:
		*ptuning_parm_value =
	(int32_t)pHP->algo__consistency_check__min_max_tolerance;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA:
		*ptuning_parm_value =
		(int32_t)pHP->algo__consistency_check__event_sigma;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT:
		*ptuning_parm_value =
		(int32_t)pHP->algo__consistency_check__event_min_spad_count;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_hist_long;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_hist_med;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_hist_short;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_hist_long;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_hist_med;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_hist_short;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM:
		*ptuning_parm_value = (int32_t)(
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm);
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM:
		*ptuning_parm_value = (int32_t)(
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm);
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE:
		*ptuning_parm_value =
		(int32_t)pHP->algo__crosstalk_detect_min_max_tolerance;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS:
		*ptuning_parm_value = (int32_t)(
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps);
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA:
		*ptuning_parm_value =
		(int32_t)pHP->algo__crosstalk_detect_event_sigma;
	break;
	case VL53LX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->xtalk_cfg.histogram_mode_crosstalk_margin_kcps;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_consistency_lite_phase_tolerance;
	break;
	case VL53LX_TUNINGPARM_PHASECAL_TARGET:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_target;
	break;
	case VL53LX_TUNINGPARM_LITE_CAL_REPEAT_RATE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_cal_repeat_rate;
	break;
	case VL53LX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR:
		*ptuning_parm_value =
		(int32_t)pdev->gain_cal.standard_ranging_gain_factor;
	break;
	case VL53LX_TUNINGPARM_LITE_MIN_CLIP_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_min_clip;
	break;
	case VL53LX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_long_sigma_thresh_mm;
	break;
	case VL53LX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_med_sigma_thresh_mm;
	break;
	case VL53LX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_short_sigma_thresh_mm;
	break;
	case VL53LX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS:
		*ptuning_parm_value = (int32_t)(
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps);
	break;
	case VL53LX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps;
	break;
	case VL53LX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS:
		*ptuning_parm_value = (int32_t)(
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps);
	break;
	case VL53LX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_sigma_est_pulse_width_ns;
	break;
	case VL53LX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_sigma_est_amb_width_ns;
	break;
	case VL53LX_TUNINGPARM_LITE_SIGMA_REF_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_sigma_ref_mm;
	break;
	case VL53LX_TUNINGPARM_LITE_RIT_MULT:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->xtalk_cfg.crosstalk_range_ignore_threshold_mult;
	break;
	case VL53LX_TUNINGPARM_LITE_SEED_CONFIG:
		*ptuning_parm_value =
				(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_seed_cfg;
	break;
	case VL53LX_TUNINGPARM_LITE_QUANTIFIER:
		*ptuning_parm_value =
				(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_quantifier;
	break;
	case VL53LX_TUNINGPARM_LITE_FIRST_ORDER_SELECT:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_first_order_select;
	break;
	case VL53LX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->xtalk_cfg.lite_mode_crosstalk_margin_kcps;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_lite_long;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_lite_med;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_lite_short;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_lite_long;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_lite_med;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_lite_short;
	break;
	case VL53LX_TUNINGPARM_TIMED_SEED_CONFIG:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_timed_seed_cfg;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->dmax_cfg.signal_thresh_sigma;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[0];
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[1];
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[2];
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[3];
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[4];
	break;
	case VL53LX_TUNINGPARM_VHV_LOOPBOUND:
		*ptuning_parm_value =
		(int32_t)pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->refspadchar.device_test_mode;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->refspadchar.VL53LX_p_005;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->refspadchar.timeout_us;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->refspadchar.target_count_rate_mcps;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->refspadchar.min_count_rate_limit_mcps;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->refspadchar.max_count_rate_limit_mcps;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES:
		*ptuning_parm_value =
		(int32_t)pXC->num_of_samples;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM:
		*ptuning_parm_value =
		(int32_t)pXC->algo__crosstalk_extract_min_valid_range_mm;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM:
		*ptuning_parm_value =
		(int32_t)pXC->algo__crosstalk_extract_max_valid_range_mm;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pXC->dss_config__target_total_rate_mcps;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pXC->phasecal_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS:
		*ptuning_parm_value =
		(int32_t)pXC->algo__crosstalk_extract_max_valid_rate_kcps;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM:
		*ptuning_parm_value =
		(int32_t)pXC->algo__crosstalk_extract_max_sigma_mm;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pXC->mm_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pXC->range_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.dss_config__target_total_rate_mcps;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.phasecal_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.mm_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.range_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.pre_num_of_samples;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.mm1_num_of_samples;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.mm2_num_of_samples;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->zonecal_cfg.dss_config__target_total_rate_mcps;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US:
		*ptuning_parm_value =
	(int32_t)pdev->zonecal_cfg.phasecal_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->zonecal_cfg.mm_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES:
		*ptuning_parm_value =
		(int32_t)pdev->zonecal_cfg.phasecal_num_of_samples;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->zonecal_cfg.range_config_timeout_us;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES:
		*ptuning_parm_value =
		(int32_t)pdev->zonecal_cfg.zone_num_of_samples;
	break;
	case VL53LX_TUNINGPARM_SPADMAP_VCSEL_PERIOD:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->ssc_cfg.VL53LX_p_005;
	break;
	case VL53LX_TUNINGPARM_SPADMAP_VCSEL_START:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->ssc_cfg.vcsel_start;
	break;
	case VL53LX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->ssc_cfg.rate_limit_mcps;
	break;
	case VL53LX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_lite_mcps;
	break;
	case VL53LX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mcps;
	break;
	case VL53LX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mz_mcps;
	break;
	case VL53LX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_timed_mcps;
	break;
	case VL53LX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_lite_us;
	break;
	case VL53LX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_long_us;
	break;
	case VL53LX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_med_us;
	break;
	case VL53LX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_short_us;
	break;
	case VL53LX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_mz_long_us;
	break;
	case VL53LX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_mz_med_us;
	break;
	case VL53LX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_mz_short_us;
	break;
	case VL53LX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_timed_us;
	break;
	case VL53LX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_lite_us;
	break;
	case VL53LX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_histo_us;
	break;
	case VL53LX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_mz_us;
	break;
	case VL53LX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_timed_us;
	break;
	case VL53LX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_lite_us;
	break;
	case VL53LX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_histo_us;
	break;
	case VL53LX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_mz_us;
	break;
	case VL53LX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_timed_us;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_margin;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NOISE_MARGIN:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.noise_margin;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_xtalk_offset_limit;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_xtalk_offset_limit_hi;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.sample_limit;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.single_xtalk_delta;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.averaged_xtalk_delta;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_CLIP_LIMIT:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_clip_limit;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SCALER_CALC_METHOD:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.scaler_calc_method;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.x_gradient_scaler;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.y_gradient_scaler;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_USER_SCALER_SET:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_scaler_set;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_single_apply;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD:
		*ptuning_parm_value = (int32_t)(
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_ambient_threshold);
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_ambient_threshold;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_sample_limit;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_xtalk_offset;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_min_range_mm;
	break;
	case VL53LX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND:
		*ptuning_parm_value =
		(int32_t)pdev->low_power_auto_data.vhv_loop_bound;
	break;
	case VL53LX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_lpa_us;
	break;
	case VL53LX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_lpa_us;
	break;
	case VL53LX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS:
		*ptuning_parm_value =
		(int32_t)pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_very_short_mcps;
	break;
	case VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER:
		*ptuning_parm_value =
		(int32_t) pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_patch_power;
	break;
	case VL53LX_TUNINGPARM_HIST_MERGE:
		*ptuning_parm_value =
		(int32_t) pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge;
	break;
	case VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD:
		*ptuning_parm_value =
		(int32_t) pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_reset_merge_threshold;
	break;
	case VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE:
		*ptuning_parm_value =
		(int32_t) pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge_max_size;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.max_smudge_factor;
	break;

	case VL53LX_TUNINGPARM_UWR_ENABLE:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_enable;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_1_min;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_1_max;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_2_min;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_2_max;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_3_min;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_3_max;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_4_min;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_4_max;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_5_min;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_5_max;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_1_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_1_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_2_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_2_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_3_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_3_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_4_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_4_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_5_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_5_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_1_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_1_min;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_1_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_1_max;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_2_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_2_min;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_2_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_2_max;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_3_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_3_min;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_3_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_3_max;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_4_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_4_min;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_4_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_4_max;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_5_MIN:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_5_min;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_5_MAX:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_5_max;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_1_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_1_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_2_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_2_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_3_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_3_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_4_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_4_rangeb;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEA:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_5_rangea;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEB:
		*ptuning_parm_value =
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_5_rangeb;
	break;

	default:
		*ptuning_parm_value = 0x7FFFFFFF;
		status = VL53LX_ERROR_INVALID_PARAMS;
	break;

	}

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_set_tuning_parm(
	VL53LX_DEV            Dev,
	VL53LX_TuningParms    tuning_parm_key,
	int32_t               tuning_parm_value)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_hist_post_process_config_t *pHP = &(pdev->histpostprocess);
	VL53LX_xtalkextract_config_t *pXC = &(pdev->VL53LX_LLDriverCommonData->xtalk_extract_cfg);

	LOG_FUNCTION_START("");

	switch (tuning_parm_key) {

	case VL53LX_TUNINGPARM_VERSION:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_tuning_parm_version =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_KEY_TABLE_VERSION:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_tuning_parm_key_table_version =
					(uint16_t)tuning_parm_value;



		if ((uint16_t)tuning_parm_value
			!= VL53LX_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT)
			status = VL53LX_ERROR_TUNING_PARM_KEY_MISMATCH;

	break;
	case VL53LX_TUNINGPARM_LLD_VERSION:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_tuning_parm_lld_version =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_ALGO_SELECT:
		pHP->hist_algo_select =
				(VL53LX_HistAlgoSelect)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_TARGET_ORDER:
		pHP->hist_target_order =
				(VL53LX_HistTargetOrder)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_FILTER_WOI_0:
		pHP->filter_woi0 =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_FILTER_WOI_1:
		pHP->filter_woi1 =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_EST_METHOD:
		pHP->hist_amb_est_method =
				(VL53LX_HistAmbEstMethod)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0:
		pHP->ambient_thresh_sigma0 =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1:
		pHP->ambient_thresh_sigma1 =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS:
		pHP->min_ambient_thresh_events =
		(int32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_AMB_EVENTS_SCALER:
		pHP->ambient_thresh_events_scaler =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_NOISE_THRESHOLD:
		pHP->noise_threshold =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT:
		pHP->signal_total_events_limit =
		(int32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_SIGMA_EST_REF_MM:
		pHP->sigma_estimator__sigma_ref_mm =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_SIGMA_THRESH_MM:
		pHP->sigma_thresh =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_GAIN_FACTOR:
		pdev->gain_cal.histogram_ranging_gain_factor =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE:
		pHP->algo__consistency_check__phase_tolerance =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM:
		pHP->algo__consistency_check__min_max_tolerance =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA:
		pHP->algo__consistency_check__event_sigma =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT:
		pHP->algo__consistency_check__event_min_spad_count =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_hist_long =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_hist_med =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_hist_short =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_hist_long =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_hist_med =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_hist_short =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_min_valid_range_mm =
				(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_max_valid_range_mm =
				(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_max_sigma_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE:
		pHP->algo__crosstalk_detect_min_max_tolerance =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_detect_max_valid_rate_kcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA:
		pHP->algo__crosstalk_detect_event_sigma =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.histogram_mode_crosstalk_margin_kcps =
				(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_consistency_lite_phase_tolerance =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_PHASECAL_TARGET:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_target =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_CAL_REPEAT_RATE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_cal_repeat_rate =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_RANGING_GAIN_FACTOR:
		pdev->gain_cal.standard_ranging_gain_factor =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_MIN_CLIP_MM:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_min_clip =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_long_sigma_thresh_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_med_sigma_thresh_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_short_sigma_thresh_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_long_min_count_rate_rtn_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_med_min_count_rate_rtn_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_short_min_count_rate_rtn_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_sigma_est_pulse_width_ns =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_sigma_est_amb_width_ns =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_SIGMA_REF_MM:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_sigma_ref_mm =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_RIT_MULT:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.crosstalk_range_ignore_threshold_mult =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_SEED_CONFIG:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_seed_cfg =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_QUANTIFIER:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_quantifier =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_FIRST_ORDER_SELECT:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_lite_first_order_select =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS:
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.lite_mode_crosstalk_margin_kcps =
				(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_lite_long =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_lite_med =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_rtn_lite_short =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_lite_long =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_lite_med =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_init_phase_ref_lite_short =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_TIMED_SEED_CONFIG:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_timed_seed_cfg =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA:
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.signal_thresh_sigma =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0:
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[0] =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1:
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[1] =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2:
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[2] =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3:
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[3] =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4:
		pdev->VL53LX_LLDriverCommonData->dmax_cfg.target_reflectance_for_dmax_calc[4] =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_VHV_LOOPBOUND:
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE:
		pdev->VL53LX_LLDriverCommonData->refspadchar.device_test_mode =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD:
		pdev->VL53LX_LLDriverCommonData->refspadchar.VL53LX_p_005 =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->refspadchar.timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->refspadchar.target_count_rate_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS:
		pdev->VL53LX_LLDriverCommonData->refspadchar.min_count_rate_limit_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS:
		pdev->VL53LX_LLDriverCommonData->refspadchar.max_count_rate_limit_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES:
		pXC->num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM:
		pXC->algo__crosstalk_extract_min_valid_range_mm =
				(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM:
		pXC->algo__crosstalk_extract_max_valid_range_mm =
				(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS:
		pXC->dss_config__target_total_rate_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US:
		pXC->phasecal_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS:
		 pXC->algo__crosstalk_extract_max_valid_rate_kcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM:
		pXC->algo__crosstalk_extract_max_sigma_mm =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US:
		pXC->mm_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US:
		pXC->range_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.dss_config__target_total_rate_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.phasecal_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.mm_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.range_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.pre_num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.mm1_num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES:
		pdev->VL53LX_LLDriverCommonData->offsetcal_cfg.mm2_num_of_samples =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS:
		pdev->zonecal_cfg.dss_config__target_total_rate_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US:
		pdev->zonecal_cfg.phasecal_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US:
		pdev->zonecal_cfg.mm_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES:
		pdev->zonecal_cfg.phasecal_num_of_samples =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US:
		pdev->zonecal_cfg.range_config_timeout_us =
				(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES:
		pdev->zonecal_cfg.zone_num_of_samples =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_SPADMAP_VCSEL_PERIOD:
		pdev->VL53LX_LLDriverCommonData->ssc_cfg.VL53LX_p_005 =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_SPADMAP_VCSEL_START:
		pdev->VL53LX_LLDriverCommonData->ssc_cfg.vcsel_start =
				(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS:
		pdev->VL53LX_LLDriverCommonData->ssc_cfg.rate_limit_mcps =
				(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_lite_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_histo_mz_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_timed_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_lite_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_long_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_med_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_hist_short_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_mz_long_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_mz_med_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_mz_short_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_timeout_timed_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_lite_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_histo_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_mz_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_timed_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_lite_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_histo_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_mz_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_timed_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_margin =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NOISE_MARGIN:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.noise_margin =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_xtalk_offset_limit =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_xtalk_offset_limit_hi =
			(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.sample_limit =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.single_xtalk_delta =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.averaged_xtalk_delta =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_CLIP_LIMIT:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_clip_limit =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_SCALER_CALC_METHOD:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.scaler_calc_method =
			(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.x_gradient_scaler =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.y_gradient_scaler =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_USER_SCALER_SET:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_scaler_set =
			(uint8_t)tuning_parm_value;
	break;

	case VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_single_apply =
			(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_ambient_threshold =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_ambient_threshold =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_sample_limit =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_xtalk_offset =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_min_range_mm =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND:
		pdev->low_power_auto_data.vhv_loop_bound =
			(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_mm_timeout_lpa_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_range_timeout_lpa_us =
			(uint32_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_dss_target_very_short_mcps =
			(uint16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_phasecal_patch_power =
			(uint16_t) tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_MERGE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge =
			(uint16_t) tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_reset_merge_threshold =
			(uint16_t) tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge_max_size =
			(uint16_t) tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR:
		pdev->VL53LX_LLDriverCommonData->smudge_correct_config.max_smudge_factor =
			(uint32_t)tuning_parm_value;
	break;

	case VL53LX_TUNINGPARM_UWR_ENABLE:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_enable =
			(uint8_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_1_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_1_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_2_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_2_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_3_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_3_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_4_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_4_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_5_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_z_5_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_1_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_1_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_2_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_2_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_3_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_3_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_4_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_4_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_5_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_med_corr_z_5_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_1_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_1_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_1_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_1_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_2_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_2_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_2_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_2_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_3_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_3_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_3_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_3_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_4_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_4_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_4_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_4_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_5_MIN:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_5_min =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_ZONE_5_MAX:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_z_5_max =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_1_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_1_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_2_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_2_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_3_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_3_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_4_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_4_rangeb =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEA:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_5_rangea =
			(int16_t)tuning_parm_value;
	break;
	case VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEB:
		pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_uwr_lng_corr_z_5_rangeb =
			(int16_t)tuning_parm_value;
	break;

	default:
		status = VL53LX_ERROR_INVALID_PARAMS;
	break;

	}

	LOG_FUNCTION_END(status);

	return status;
}





VL53LX_Error VL53LX_dynamic_xtalk_correction_enable(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_enabled = 1;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_disable(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_enabled = 0;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_apply_disable(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_apply_enabled = 0;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_single_apply_enable(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_single_apply = 1;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_single_apply_disable(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_single_apply = 0;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_dynamic_xtalk_correction_apply_enable(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_apply_enabled = 1;

	LOG_FUNCTION_END(status);

	return status;
}






VL53LX_Error VL53LX_get_current_xtalk_settings(
	VL53LX_DEV                          Dev,
	VL53LX_xtalk_calibration_results_t *pxtalk
	)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	uint8_t i;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pxtalk->algo__crosstalk_compensation_plane_offset_kcps =
		pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps;
	pxtalk->algo__crosstalk_compensation_x_plane_gradient_kcps =
	pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps;
	pxtalk->algo__crosstalk_compensation_y_plane_gradient_kcps =
	pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps;
	for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
		pxtalk->algo__xtalk_cpo_HistoMerge_kcps[i] =
		pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[i];

	LOG_FUNCTION_END(status);

	return status;

}





VL53LX_Error VL53LX_set_current_xtalk_settings(
	VL53LX_DEV                          Dev,
	VL53LX_xtalk_calibration_results_t *pxtalk
	)
{

	uint8_t i;
	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_compensation_plane_offset_kcps =
		pxtalk->algo__crosstalk_compensation_plane_offset_kcps;
	pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_compensation_x_plane_gradient_kcps =
		pxtalk->algo__crosstalk_compensation_x_plane_gradient_kcps;
	pdev->VL53LX_LLDriverCommonData->xtalk_cfg.algo__crosstalk_compensation_y_plane_gradient_kcps =
		pxtalk->algo__crosstalk_compensation_y_plane_gradient_kcps;
	for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
		pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[i] =
		pxtalk->algo__xtalk_cpo_HistoMerge_kcps[i];

	LOG_FUNCTION_END(status);

	return status;

}



