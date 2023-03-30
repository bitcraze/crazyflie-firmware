
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include <vl53lx_platform_log.h>
#include "vl53lx_ll_def.h"
#include "vl53lx_register_structs.h"
#include "vl53lx_register_settings.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_core.h"
#include "vl53lx_api_preset_modes.h"
#include "vl53lx_tuning_parm_defaults.h"


#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53LX_TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53LX_TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53LX_TRACE_MODULE_API,\
	status, fmt, ##__VA_ARGS__)


VL53LX_Error VL53LX_init_refspadchar_config_struct(
	VL53LX_refspadchar_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	pdata->device_test_mode =
		VL53LX_TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT;
	pdata->VL53LX_p_005              =
		VL53LX_TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT;
	pdata->timeout_us                =
		VL53LX_TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT;
	pdata->target_count_rate_mcps    =
		VL53LX_TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT;
	pdata->min_count_rate_limit_mcps =
		VL53LX_TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT;
	pdata->max_count_rate_limit_mcps =
		VL53LX_TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_ssc_config_struct(
	VL53LX_ssc_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");




	pdata->array_select = VL53LX_DEVICESSCARRAY_RTN;


	pdata->VL53LX_p_005 =
			VL53LX_TUNINGPARM_SPADMAP_VCSEL_PERIOD_DEFAULT;


	pdata->vcsel_start  =
			VL53LX_TUNINGPARM_SPADMAP_VCSEL_START_DEFAULT;


	pdata->vcsel_width = 0x02;


	pdata->timeout_us   = 36000;


	pdata->rate_limit_mcps =
			VL53LX_TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_xtalk_config_struct(
	VL53LX_customer_nvm_managed_t *pnvm,
	VL53LX_xtalk_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");





	pdata->algo__crosstalk_compensation_plane_offset_kcps      =
		pnvm->algo__crosstalk_compensation_plane_offset_kcps;
	pdata->algo__crosstalk_compensation_x_plane_gradient_kcps  =
		pnvm->algo__crosstalk_compensation_x_plane_gradient_kcps;
	pdata->algo__crosstalk_compensation_y_plane_gradient_kcps  =
		pnvm->algo__crosstalk_compensation_y_plane_gradient_kcps;



	pdata->nvm_default__crosstalk_compensation_plane_offset_kcps      =
		(uint32_t)pnvm->algo__crosstalk_compensation_plane_offset_kcps;
	pdata->nvm_default__crosstalk_compensation_x_plane_gradient_kcps  =
		pnvm->algo__crosstalk_compensation_x_plane_gradient_kcps;
	pdata->nvm_default__crosstalk_compensation_y_plane_gradient_kcps  =
		pnvm->algo__crosstalk_compensation_y_plane_gradient_kcps;

	pdata->histogram_mode_crosstalk_margin_kcps                =
			VL53LX_TUNINGPARM_HIST_XTALK_MARGIN_KCPS_DEFAULT;
	pdata->lite_mode_crosstalk_margin_kcps                     =
			VL53LX_TUNINGPARM_LITE_XTALK_MARGIN_KCPS_DEFAULT;



	pdata->crosstalk_range_ignore_threshold_mult =
			VL53LX_TUNINGPARM_LITE_RIT_MULT_DEFAULT;

	if ((pdata->algo__crosstalk_compensation_plane_offset_kcps == 0x00)
		&& (pdata->algo__crosstalk_compensation_x_plane_gradient_kcps
				== 0x00)
		&& (pdata->algo__crosstalk_compensation_y_plane_gradient_kcps
				== 0x00))
		pdata->global_crosstalk_compensation_enable = 0x00;
	else
		pdata->global_crosstalk_compensation_enable = 0x01;


	if ((status == VL53LX_ERROR_NONE) &&
		(pdata->global_crosstalk_compensation_enable == 0x01)) {
		pdata->crosstalk_range_ignore_threshold_rate_mcps =
		VL53LX_calc_range_ignore_threshold(
		pdata->algo__crosstalk_compensation_plane_offset_kcps,
		pdata->algo__crosstalk_compensation_x_plane_gradient_kcps,
		pdata->algo__crosstalk_compensation_y_plane_gradient_kcps,
		pdata->crosstalk_range_ignore_threshold_mult);
	} else {
		pdata->crosstalk_range_ignore_threshold_rate_mcps = 0;
	}




	pdata->algo__crosstalk_detect_min_valid_range_mm  =
		VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM_DEFAULT;
	pdata->algo__crosstalk_detect_max_valid_range_mm  =
		VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM_DEFAULT;
	pdata->algo__crosstalk_detect_max_valid_rate_kcps =
		VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS_DEFAULT;
	pdata->algo__crosstalk_detect_max_sigma_mm        =
			VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_init_xtalk_extract_config_struct(
	VL53LX_xtalkextract_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	pdata->dss_config__target_total_rate_mcps          =
			VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_RATE_MCPS_DEFAULT;

	pdata->mm_config_timeout_us                        =
			VL53LX_TUNINGPARM_XTALK_EXTRACT_DSS_TIMEOUT_US_DEFAULT;

	pdata->num_of_samples                              =
			VL53LX_TUNINGPARM_XTALK_EXTRACT_NUM_OF_SAMPLES_DEFAULT;

	pdata->phasecal_config_timeout_us                  =
		VL53LX_TUNINGPARM_XTALK_EXTRACT_PHASECAL_TIMEOUT_US_DEFAULT;

	pdata->range_config_timeout_us                     =
			VL53LX_TUNINGPARM_XTALK_EXTRACT_BIN_TIMEOUT_US_DEFAULT;




	pdata->algo__crosstalk_extract_min_valid_range_mm  =
		VL53LX_TUNINGPARM_XTALK_EXTRACT_MIN_FILTER_THRESH_MM_DEFAULT;
	pdata->algo__crosstalk_extract_max_valid_range_mm  =
		VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_FILTER_THRESH_MM_DEFAULT;
	pdata->algo__crosstalk_extract_max_valid_rate_kcps =
		VL53LX_TUNINGPARM_XTALK_EXTRACT_MAX_VALID_RATE_KCPS_DEFAULT;
	pdata->algo__crosstalk_extract_max_sigma_mm        =
		VL53LX_TUNINGPARM_XTALK_EXTRACT_SIGMA_THRESHOLD_MM_DEFAULT;


	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_offset_cal_config_struct(
	VL53LX_offsetcal_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	pdata->dss_config__target_total_rate_mcps          =
			VL53LX_TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT;

	pdata->phasecal_config_timeout_us                  =
		VL53LX_TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT;

	pdata->range_config_timeout_us                     =
			VL53LX_TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT;

	pdata->mm_config_timeout_us                        =
			VL53LX_TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT;




	pdata->pre_num_of_samples                          =
			VL53LX_TUNINGPARM_OFFSET_CAL_PRE_SAMPLES_DEFAULT;
	pdata->mm1_num_of_samples                          =
			VL53LX_TUNINGPARM_OFFSET_CAL_MM1_SAMPLES_DEFAULT;
	pdata->mm2_num_of_samples                          =
			VL53LX_TUNINGPARM_OFFSET_CAL_MM2_SAMPLES_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_init_zone_cal_config_struct(
	VL53LX_zonecal_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	pdata->dss_config__target_total_rate_mcps          =
			VL53LX_TUNINGPARM_ZONE_CAL_DSS_RATE_MCPS_DEFAULT;

	pdata->phasecal_config_timeout_us                  =
			VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_TIMEOUT_US_DEFAULT;

	pdata->range_config_timeout_us                     =
			VL53LX_TUNINGPARM_ZONE_CAL_RANGE_TIMEOUT_US_DEFAULT;

	pdata->mm_config_timeout_us                        =
			VL53LX_TUNINGPARM_ZONE_CAL_DSS_TIMEOUT_US_DEFAULT;




	pdata->phasecal_num_of_samples                     =
			VL53LX_TUNINGPARM_ZONE_CAL_PHASECAL_NUM_SAMPLES_DEFAULT;
	pdata->zone_num_of_samples                         =
			VL53LX_TUNINGPARM_ZONE_CAL_ZONE_NUM_SAMPLES_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_hist_post_process_config_struct(
	uint8_t                             xtalk_compensation_enable,
	VL53LX_hist_post_process_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	pdata->hist_algo_select =
			VL53LX_TUNINGPARM_HIST_ALGO_SELECT_DEFAULT;



	pdata->hist_target_order =
			VL53LX_TUNINGPARM_HIST_TARGET_ORDER_DEFAULT;



	pdata->filter_woi0                   =
			VL53LX_TUNINGPARM_HIST_FILTER_WOI_0_DEFAULT;
	pdata->filter_woi1                   =
			VL53LX_TUNINGPARM_HIST_FILTER_WOI_1_DEFAULT;


	pdata->hist_amb_est_method =
			VL53LX_TUNINGPARM_HIST_AMB_EST_METHOD_DEFAULT;

	pdata->ambient_thresh_sigma0         =
			VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_0_DEFAULT;
	pdata->ambient_thresh_sigma1         =
			VL53LX_TUNINGPARM_HIST_AMB_THRESH_SIGMA_1_DEFAULT;


	pdata->ambient_thresh_events_scaler     =
			VL53LX_TUNINGPARM_HIST_AMB_EVENTS_SCALER_DEFAULT;


	pdata->min_ambient_thresh_events     =
			VL53LX_TUNINGPARM_HIST_MIN_AMB_THRESH_EVENTS_DEFAULT;

	pdata->noise_threshold               =
			VL53LX_TUNINGPARM_HIST_NOISE_THRESHOLD_DEFAULT;

	pdata->signal_total_events_limit     =
		VL53LX_TUNINGPARM_HIST_SIGNAL_TOTAL_EVENTS_LIMIT_DEFAULT;
	pdata->sigma_estimator__sigma_ref_mm =
		VL53LX_TUNINGPARM_HIST_SIGMA_EST_REF_MM_DEFAULT;


	pdata->sigma_thresh                  =
			VL53LX_TUNINGPARM_HIST_SIGMA_THRESH_MM_DEFAULT;

	pdata->range_offset_mm            =      0;

	pdata->gain_factor                =
			VL53LX_TUNINGPARM_HIST_GAIN_FACTOR_DEFAULT;



	pdata->valid_phase_low = 0x08;
	pdata->valid_phase_high = 0x88;



	pdata->algo__consistency_check__phase_tolerance =
		VL53LX_TUNINGPARM_CONSISTENCY_HIST_PHASE_TOLERANCE_DEFAULT;



	pdata->algo__consistency_check__event_sigma =
		VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_DEFAULT;


	pdata->algo__consistency_check__event_min_spad_count =
	VL53LX_TUNINGPARM_CONSISTENCY_HIST_EVENT_SIGMA_MIN_SPAD_LIMIT_DEFAULT;



	pdata->algo__consistency_check__min_max_tolerance =
		VL53LX_TUNINGPARM_CONSISTENCY_HIST_MIN_MAX_TOLERANCE_MM_DEFAULT;


	pdata->algo__crosstalk_compensation_enable = xtalk_compensation_enable;


	pdata->algo__crosstalk_detect_min_valid_range_mm  =
		VL53LX_TUNINGPARM_XTALK_DETECT_MIN_VALID_RANGE_MM_DEFAULT;
	pdata->algo__crosstalk_detect_max_valid_range_mm  =
		VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RANGE_MM_DEFAULT;
	pdata->algo__crosstalk_detect_max_valid_rate_kcps =
		VL53LX_TUNINGPARM_XTALK_DETECT_MAX_VALID_RATE_KCPS_DEFAULT;
	pdata->algo__crosstalk_detect_max_sigma_mm        =
		VL53LX_TUNINGPARM_XTALK_DETECT_MAX_SIGMA_MM_DEFAULT;





	pdata->algo__crosstalk_detect_event_sigma       =
		VL53LX_TUNINGPARM_XTALK_DETECT_EVENT_SIGMA_DEFAULT;



	pdata->algo__crosstalk_detect_min_max_tolerance   =
		VL53LX_TUNINGPARM_XTALK_DETECT_MIN_MAX_TOLERANCE_DEFAULT;



	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_dmax_calibration_data_struct(
	VL53LX_dmax_calibration_data_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");




	pdata->ref__actual_effective_spads = 0x5F2D;

	pdata->ref__peak_signal_count_rate_mcps = 0x0844;

	pdata->ref__distance_mm = 0x08A5;


	pdata->ref_reflectance_pc = 0x0014;


	pdata->coverglass_transmission = 0x0100;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_tuning_parm_storage_struct(
	VL53LX_tuning_parm_storage_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	pdata->tp_tuning_parm_version              =
			VL53LX_TUNINGPARM_VERSION_DEFAULT;
	pdata->tp_tuning_parm_key_table_version    =
			VL53LX_TUNINGPARM_KEY_TABLE_VERSION_DEFAULT;
	pdata->tp_tuning_parm_lld_version          =
			VL53LX_TUNINGPARM_LLD_VERSION_DEFAULT;
	pdata->tp_init_phase_rtn_lite_long         =
		VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT;
	pdata->tp_init_phase_rtn_lite_med          =
		VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT;
	pdata->tp_init_phase_rtn_lite_short        =
		VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT;
	pdata->tp_init_phase_ref_lite_long         =
		VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT;
	pdata->tp_init_phase_ref_lite_med          =
		VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT;
	pdata->tp_init_phase_ref_lite_short        =
		VL53LX_TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT;
	pdata->tp_init_phase_rtn_hist_long         =
		VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_LONG_RANGE_DEFAULT;
	pdata->tp_init_phase_rtn_hist_med          =
		VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_MED_RANGE_DEFAULT;
	pdata->tp_init_phase_rtn_hist_short        =
		VL53LX_TUNINGPARM_INITIAL_PHASE_RTN_HISTO_SHORT_RANGE_DEFAULT;
	pdata->tp_init_phase_ref_hist_long         =
		VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_LONG_RANGE_DEFAULT;
	pdata->tp_init_phase_ref_hist_med          =
		VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_MED_RANGE_DEFAULT;
	pdata->tp_init_phase_ref_hist_short        =
		VL53LX_TUNINGPARM_INITIAL_PHASE_REF_HISTO_SHORT_RANGE_DEFAULT;
	pdata->tp_consistency_lite_phase_tolerance =
		VL53LX_TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT;
	pdata->tp_phasecal_target                  =
			VL53LX_TUNINGPARM_PHASECAL_TARGET_DEFAULT;
	pdata->tp_cal_repeat_rate                  =
			VL53LX_TUNINGPARM_LITE_CAL_REPEAT_RATE_DEFAULT;
	pdata->tp_lite_min_clip                    =
			VL53LX_TUNINGPARM_LITE_MIN_CLIP_MM_DEFAULT;
	pdata->tp_lite_long_sigma_thresh_mm        =
			VL53LX_TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT;
	pdata->tp_lite_med_sigma_thresh_mm         =
			VL53LX_TUNINGPARM_LITE_MED_SIGMA_THRESH_MM_DEFAULT;
	pdata->tp_lite_short_sigma_thresh_mm       =
			VL53LX_TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT;
	pdata->tp_lite_long_min_count_rate_rtn_mcps  =
		VL53LX_TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
	pdata->tp_lite_med_min_count_rate_rtn_mcps   =
		VL53LX_TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
	pdata->tp_lite_short_min_count_rate_rtn_mcps =
		VL53LX_TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
	pdata->tp_lite_sigma_est_pulse_width_ns      =
			VL53LX_TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT;
	pdata->tp_lite_sigma_est_amb_width_ns        =
			VL53LX_TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT;
	pdata->tp_lite_sigma_ref_mm                  =
			VL53LX_TUNINGPARM_LITE_SIGMA_REF_MM_DEFAULT;
	pdata->tp_lite_seed_cfg                      =
			VL53LX_TUNINGPARM_LITE_SEED_CONFIG_DEFAULT;
	pdata->tp_timed_seed_cfg                     =
			VL53LX_TUNINGPARM_TIMED_SEED_CONFIG_DEFAULT;
	pdata->tp_lite_quantifier                    =
			VL53LX_TUNINGPARM_LITE_QUANTIFIER_DEFAULT;
	pdata->tp_lite_first_order_select            =
			VL53LX_TUNINGPARM_LITE_FIRST_ORDER_SELECT_DEFAULT;
	pdata->tp_uwr_enable =
		VL53LX_TUNINGPARM_UWR_ENABLE_DEFAULT;
	pdata->tp_uwr_med_z_1_min =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MIN_DEFAULT;
	pdata->tp_uwr_med_z_1_max =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_1_MAX_DEFAULT;
	pdata->tp_uwr_med_z_2_min =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MIN_DEFAULT;
	pdata->tp_uwr_med_z_2_max =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_2_MAX_DEFAULT;
	pdata->tp_uwr_med_z_3_min =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MIN_DEFAULT;
	pdata->tp_uwr_med_z_3_max =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_3_MAX_DEFAULT;
	pdata->tp_uwr_med_z_4_min =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MIN_DEFAULT;
	pdata->tp_uwr_med_z_4_max =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_4_MAX_DEFAULT;
	pdata->tp_uwr_med_z_5_min =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MIN_DEFAULT;
	pdata->tp_uwr_med_z_5_max =
		VL53LX_TUNINGPARM_UWR_MEDIUM_ZONE_5_MAX_DEFAULT;
	pdata->tp_uwr_med_corr_z_1_rangea =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEA_DEFAULT;
	pdata->tp_uwr_med_corr_z_1_rangeb =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_1_RANGEB_DEFAULT;
	pdata->tp_uwr_med_corr_z_2_rangea =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEA_DEFAULT;
	pdata->tp_uwr_med_corr_z_2_rangeb =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_2_RANGEB_DEFAULT;
	pdata->tp_uwr_med_corr_z_3_rangea =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEA_DEFAULT;
	pdata->tp_uwr_med_corr_z_3_rangeb =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_3_RANGEB_DEFAULT;
	pdata->tp_uwr_med_corr_z_4_rangea =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEA_DEFAULT;
	pdata->tp_uwr_med_corr_z_4_rangeb =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_4_RANGEB_DEFAULT;
	pdata->tp_uwr_med_corr_z_5_rangea =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEA_DEFAULT;
	pdata->tp_uwr_med_corr_z_5_rangeb =
		VL53LX_TUNINGPARM_UWR_MEDIUM_CORRECTION_ZONE_5_RANGEB_DEFAULT;
	pdata->tp_uwr_lng_z_1_min =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_1_MIN_DEFAULT;
	pdata->tp_uwr_lng_z_1_max =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_1_MAX_DEFAULT;
	pdata->tp_uwr_lng_z_2_min =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_2_MIN_DEFAULT;
	pdata->tp_uwr_lng_z_2_max =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_2_MAX_DEFAULT;
	pdata->tp_uwr_lng_z_3_min =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_3_MIN_DEFAULT;
	pdata->tp_uwr_lng_z_3_max =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_3_MAX_DEFAULT;
	pdata->tp_uwr_lng_z_4_min =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_4_MIN_DEFAULT;
	pdata->tp_uwr_lng_z_4_max =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_4_MAX_DEFAULT;
	pdata->tp_uwr_lng_z_5_min =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_5_MIN_DEFAULT;
	pdata->tp_uwr_lng_z_5_max =
		VL53LX_TUNINGPARM_UWR_LONG_ZONE_5_MAX_DEFAULT;
	pdata->tp_uwr_lng_corr_z_1_rangea =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEA_DEFAULT;
	pdata->tp_uwr_lng_corr_z_1_rangeb =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_1_RANGEB_DEFAULT;
	pdata->tp_uwr_lng_corr_z_2_rangea =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEA_DEFAULT;
	pdata->tp_uwr_lng_corr_z_2_rangeb =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_2_RANGEB_DEFAULT;
	pdata->tp_uwr_lng_corr_z_3_rangea =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEA_DEFAULT;
	pdata->tp_uwr_lng_corr_z_3_rangeb =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_3_RANGEB_DEFAULT;
	pdata->tp_uwr_lng_corr_z_4_rangea =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEA_DEFAULT;
	pdata->tp_uwr_lng_corr_z_4_rangeb =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_4_RANGEB_DEFAULT;
	pdata->tp_uwr_lng_corr_z_5_rangea =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEA_DEFAULT;
	pdata->tp_uwr_lng_corr_z_5_rangeb =
		VL53LX_TUNINGPARM_UWR_LONG_CORRECTION_ZONE_5_RANGEB_DEFAULT;




	pdata->tp_dss_target_lite_mcps               =
	VL53LX_TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
	pdata->tp_dss_target_histo_mcps              =
	VL53LX_TUNINGPARM_RANGING_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
	pdata->tp_dss_target_histo_mz_mcps           =
	VL53LX_TUNINGPARM_MZ_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
	pdata->tp_dss_target_timed_mcps              =
	VL53LX_TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
	pdata->tp_phasecal_timeout_lite_us           =
		VL53LX_TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_hist_long_us      =
	VL53LX_TUNINGPARM_RANGING_LONG_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_hist_med_us       =
	VL53LX_TUNINGPARM_RANGING_MED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_hist_short_us     =
	VL53LX_TUNINGPARM_RANGING_SHORT_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_mz_long_us        =
	VL53LX_TUNINGPARM_MZ_LONG_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_mz_med_us         =
		VL53LX_TUNINGPARM_MZ_MED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_mz_short_us       =
		VL53LX_TUNINGPARM_MZ_SHORT_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_phasecal_timeout_timed_us          =
		VL53LX_TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_mm_timeout_lite_us                 =
			VL53LX_TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_mm_timeout_histo_us                =
			VL53LX_TUNINGPARM_RANGING_MM_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_mm_timeout_mz_us                   =
			VL53LX_TUNINGPARM_MZ_MM_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_mm_timeout_timed_us                =
			VL53LX_TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_range_timeout_lite_us              =
			VL53LX_TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_range_timeout_histo_us             =
		VL53LX_TUNINGPARM_RANGING_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_range_timeout_mz_us                =
			VL53LX_TUNINGPARM_MZ_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_range_timeout_timed_us             =
		VL53LX_TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT;



	pdata->tp_mm_timeout_lpa_us =
		VL53LX_TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT;
	pdata->tp_range_timeout_lpa_us =
		VL53LX_TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

	pdata->tp_dss_target_very_short_mcps =
		VL53LX_TUNINGPARM_VERY_SHORT_DSS_RATE_MCPS_DEFAULT;

	pdata->tp_phasecal_patch_power =
		VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER_DEFAULT;

	pdata->tp_hist_merge =
		VL53LX_TUNINGPARM_HIST_MERGE_DEFAULT;

	pdata->tp_reset_merge_threshold =
		VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD_DEFAULT;

	pdata->tp_hist_merge_max_size =
		VL53LX_TUNINGPARM_HIST_MERGE_MAX_SIZE_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_init_hist_gen3_dmax_config_struct(
	VL53LX_hist_gen3_dmax_config_t   *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");


	pdata->dss_config__target_total_rate_mcps = 0x1400;
	pdata->dss_config__aperture_attenuation = 0x38;

	pdata->signal_thresh_sigma                 =
			VL53LX_TUNINGPARM_DMAX_CFG_SIGNAL_THRESH_SIGMA_DEFAULT;
	pdata->ambient_thresh_sigma = 0x70;
	pdata->min_ambient_thresh_events           = 16;
	pdata->signal_total_events_limit           = 100;
	pdata->max_effective_spads = 0xFFFF;



	pdata->target_reflectance_for_dmax_calc[0] =
			VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_0_DEFAULT;
	pdata->target_reflectance_for_dmax_calc[1] =
			VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_1_DEFAULT;
	pdata->target_reflectance_for_dmax_calc[2] =
			VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_2_DEFAULT;
	pdata->target_reflectance_for_dmax_calc[3] =
			VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_3_DEFAULT;
	pdata->target_reflectance_for_dmax_calc[4] =
			VL53LX_TUNINGPARM_DMAX_CFG_REFLECTANCE_ARRAY_4_DEFAULT;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_preset_mode_standard_ranging(
	VL53LX_static_config_t    *pstatic,
	VL53LX_histogram_config_t *phistogram,
	VL53LX_general_config_t   *pgeneral,
	VL53LX_timing_config_t    *ptiming,
	VL53LX_dynamic_config_t   *pdynamic,
	VL53LX_system_control_t   *psystem,
	VL53LX_tuning_parm_storage_t *ptuning_parms,
	VL53LX_zone_config_t      *pzone_cfg)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");




	pstatic->dss_config__target_total_rate_mcps = 0x0A00;
	pstatic->debug__ctrl = 0x00;
	pstatic->test_mode__ctrl = 0x00;
	pstatic->clk_gating__ctrl = 0x00;
	pstatic->nvm_bist__ctrl = 0x00;
	pstatic->nvm_bist__num_nvm_words = 0x00;
	pstatic->nvm_bist__start_address = 0x00;
	pstatic->host_if__status = 0x00;
	pstatic->pad_i2c_hv__config = 0x00;
	pstatic->pad_i2c_hv__extsup_config = 0x00;


	pstatic->gpio_hv_pad__ctrl = 0x00;


	pstatic->gpio_hv_mux__ctrl  =
			VL53LX_DEVICEINTERRUPTPOLARITY_ACTIVE_LOW |
			VL53LX_DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS;

	pstatic->gpio__tio_hv_status = 0x02;
	pstatic->gpio__fio_hv_status = 0x00;
	pstatic->ana_config__spad_sel_pswidth = 0x02;
	pstatic->ana_config__vcsel_pulse_width_offset = 0x08;
	pstatic->ana_config__fast_osc__config_ctrl = 0x00;

	pstatic->sigma_estimator__effective_pulse_width_ns        =
			ptuning_parms->tp_lite_sigma_est_pulse_width_ns;
	pstatic->sigma_estimator__effective_ambient_width_ns      =
			ptuning_parms->tp_lite_sigma_est_amb_width_ns;
	pstatic->sigma_estimator__sigma_ref_mm                    =
			ptuning_parms->tp_lite_sigma_ref_mm;

	pstatic->algo__crosstalk_compensation_valid_height_mm = 0x01;
	pstatic->spare_host_config__static_config_spare_0 = 0x00;
	pstatic->spare_host_config__static_config_spare_1 = 0x00;

	pstatic->algo__range_ignore_threshold_mcps = 0x0000;


	pstatic->algo__range_ignore_valid_height_mm = 0xff;
	pstatic->algo__range_min_clip                             =
			ptuning_parms->tp_lite_min_clip;

	pstatic->algo__consistency_check__tolerance               =
			ptuning_parms->tp_consistency_lite_phase_tolerance;
	pstatic->spare_host_config__static_config_spare_2 = 0x00;
	pstatic->sd_config__reset_stages_msb = 0x00;
	pstatic->sd_config__reset_stages_lsb = 0x00;

	pgeneral->gph_config__stream_count_update_value = 0x00;
	pgeneral->global_config__stream_divider = 0x00;
	pgeneral->system__interrupt_config_gpio =
			VL53LX_INTERRUPT_CONFIG_NEW_SAMPLE_READY;
	pgeneral->cal_config__vcsel_start = 0x0B;


	pgeneral->cal_config__repeat_rate                         =
			ptuning_parms->tp_cal_repeat_rate;
	pgeneral->global_config__vcsel_width = 0x02;

	pgeneral->phasecal_config__timeout_macrop = 0x0D;

	pgeneral->phasecal_config__target                         =
			ptuning_parms->tp_phasecal_target;
	pgeneral->phasecal_config__override = 0x00;
	pgeneral->dss_config__roi_mode_control =
			VL53LX_DEVICEDSSMODE__TARGET_RATE;

	pgeneral->system__thresh_rate_high = 0x0000;
	pgeneral->system__thresh_rate_low = 0x0000;

	pgeneral->dss_config__manual_effective_spads_select = 0x8C00;
	pgeneral->dss_config__manual_block_select = 0x00;


	pgeneral->dss_config__aperture_attenuation = 0x38;
	pgeneral->dss_config__max_spads_limit = 0xFF;
	pgeneral->dss_config__min_spads_limit = 0x01;




	ptiming->mm_config__timeout_macrop_a_hi = 0x00;
	ptiming->mm_config__timeout_macrop_a_lo = 0x1a;
	ptiming->mm_config__timeout_macrop_b_hi = 0x00;
	ptiming->mm_config__timeout_macrop_b_lo = 0x20;

	ptiming->range_config__timeout_macrop_a_hi = 0x01;
	ptiming->range_config__timeout_macrop_a_lo = 0xCC;

	ptiming->range_config__vcsel_period_a = 0x0B;

	ptiming->range_config__timeout_macrop_b_hi = 0x01;
	ptiming->range_config__timeout_macrop_b_lo = 0xF5;

	ptiming->range_config__vcsel_period_b = 0x09;

	ptiming->range_config__sigma_thresh                       =
			ptuning_parms->tp_lite_med_sigma_thresh_mm;

	ptiming->range_config__min_count_rate_rtn_limit_mcps      =
			ptuning_parms->tp_lite_med_min_count_rate_rtn_mcps;


	ptiming->range_config__valid_phase_low = 0x08;
	ptiming->range_config__valid_phase_high = 0x78;
	ptiming->system__intermeasurement_period = 0x00000000;
	ptiming->system__fractional_enable = 0x00;



	phistogram->histogram_config__low_amb_even_bin_0_1 = 0x07;
	phistogram->histogram_config__low_amb_even_bin_2_3 = 0x21;
	phistogram->histogram_config__low_amb_even_bin_4_5 = 0x43;

	phistogram->histogram_config__low_amb_odd_bin_0_1 = 0x10;
	phistogram->histogram_config__low_amb_odd_bin_2_3 = 0x32;
	phistogram->histogram_config__low_amb_odd_bin_4_5 = 0x54;

	phistogram->histogram_config__mid_amb_even_bin_0_1 = 0x07;
	phistogram->histogram_config__mid_amb_even_bin_2_3 = 0x21;
	phistogram->histogram_config__mid_amb_even_bin_4_5 = 0x43;

	phistogram->histogram_config__mid_amb_odd_bin_0_1 = 0x10;
	phistogram->histogram_config__mid_amb_odd_bin_2 = 0x02;
	phistogram->histogram_config__mid_amb_odd_bin_3_4 = 0x43;
	phistogram->histogram_config__mid_amb_odd_bin_5 = 0x05;

	phistogram->histogram_config__user_bin_offset = 0x00;

	phistogram->histogram_config__high_amb_even_bin_0_1 = 0x07;
	phistogram->histogram_config__high_amb_even_bin_2_3 = 0x21;
	phistogram->histogram_config__high_amb_even_bin_4_5 = 0x43;

	phistogram->histogram_config__high_amb_odd_bin_0_1 = 0x10;
	phistogram->histogram_config__high_amb_odd_bin_2_3 = 0x32;
	phistogram->histogram_config__high_amb_odd_bin_4_5 = 0x54;

	phistogram->histogram_config__amb_thresh_low = 0xFFFF;
	phistogram->histogram_config__amb_thresh_high = 0xFFFF;

	phistogram->histogram_config__spad_array_selection = 0x00;


	pzone_cfg->max_zones                     = VL53LX_MAX_USER_ZONES;
	pzone_cfg->active_zones = 0x00;
	pzone_cfg->user_zones[0].height = 0x0f;
	pzone_cfg->user_zones[0].width = 0x0f;
	pzone_cfg->user_zones[0].x_centre = 0x08;
	pzone_cfg->user_zones[0].y_centre = 0x08;



	pdynamic->system__grouped_parameter_hold_0 = 0x01;

	pdynamic->system__thresh_high = 0x0000;
	pdynamic->system__thresh_low = 0x0000;
	pdynamic->system__enable_xtalk_per_quadrant = 0x00;
	pdynamic->system__seed_config =
			ptuning_parms->tp_lite_seed_cfg;


	pdynamic->sd_config__woi_sd0 = 0x0B;

	pdynamic->sd_config__woi_sd1 = 0x09;

	pdynamic->sd_config__initial_phase_sd0                     =
			ptuning_parms->tp_init_phase_rtn_lite_med;
	pdynamic->sd_config__initial_phase_sd1                     =
			ptuning_parms->tp_init_phase_ref_lite_med;

	pdynamic->system__grouped_parameter_hold_1 = 0x01;



	pdynamic->sd_config__first_order_select =
			ptuning_parms->tp_lite_first_order_select;
	pdynamic->sd_config__quantifier         =
			ptuning_parms->tp_lite_quantifier;


	pdynamic->roi_config__user_roi_centre_spad = 0xC7;

	pdynamic->roi_config__user_roi_requested_global_xy_size = 0xFF;


	pdynamic->system__sequence_config                          =
			VL53LX_SEQUENCE_VHV_EN |
			VL53LX_SEQUENCE_PHASECAL_EN |
			VL53LX_SEQUENCE_DSS1_EN |
			VL53LX_SEQUENCE_DSS2_EN |
			VL53LX_SEQUENCE_MM2_EN |
			VL53LX_SEQUENCE_RANGE_EN;

	pdynamic->system__grouped_parameter_hold = 0x02;




	psystem->system__stream_count_ctrl = 0x00;
	psystem->firmware__enable = 0x01;
	psystem->system__interrupt_clear                           =
			VL53LX_CLEAR_RANGE_INT;

	psystem->system__mode_start                                =
			VL53LX_DEVICESCHEDULERMODE_STREAMING |
			VL53LX_DEVICEREADOUTMODE_SINGLE_SD |
			VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_preset_mode_histogram_ranging(
	VL53LX_hist_post_process_config_t  *phistpostprocess,
	VL53LX_static_config_t             *pstatic,
	VL53LX_histogram_config_t          *phistogram,
	VL53LX_general_config_t            *pgeneral,
	VL53LX_timing_config_t             *ptiming,
	VL53LX_dynamic_config_t            *pdynamic,
	VL53LX_system_control_t            *psystem,
	VL53LX_tuning_parm_storage_t       *ptuning_parms,
	VL53LX_zone_config_t               *pzone_cfg)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	status =
		VL53LX_preset_mode_standard_ranging(
			pstatic,
			phistogram,
			pgeneral,
			ptiming,
			pdynamic,
			psystem,
			ptuning_parms,
			pzone_cfg);



	if (status == VL53LX_ERROR_NONE) {



		pstatic->dss_config__target_total_rate_mcps = 0x1400;



		VL53LX_init_histogram_config_structure(
				7, 0, 1, 2, 3, 4,
				0, 1, 2, 3, 4, 5,
				phistogram);


		VL53LX_init_histogram_multizone_config_structure(
				7, 0, 1, 2, 3, 4,
				0, 1, 2, 3, 4, 5,
				&(pzone_cfg->multizone_hist_cfg));




		ptiming->range_config__vcsel_period_a = 0x09;
		ptiming->range_config__vcsel_period_b = 0x0B;
		pdynamic->sd_config__woi_sd0 = 0x09;
		pdynamic->sd_config__woi_sd1 = 0x0B;




		ptiming->mm_config__timeout_macrop_a_hi = 0x00;
		ptiming->mm_config__timeout_macrop_a_lo = 0x20;
		ptiming->mm_config__timeout_macrop_b_hi = 0x00;
		ptiming->mm_config__timeout_macrop_b_lo = 0x1A;


		ptiming->range_config__timeout_macrop_a_hi = 0x00;
		ptiming->range_config__timeout_macrop_a_lo = 0x28;


		ptiming->range_config__timeout_macrop_b_hi = 0x00;
		ptiming->range_config__timeout_macrop_b_lo = 0x21;


		pgeneral->phasecal_config__timeout_macrop = 0xF5;



		phistpostprocess->valid_phase_low = 0x08;
		phistpostprocess->valid_phase_high = 0x88;



		VL53LX_copy_hist_cfg_to_static_cfg(
				phistogram,
				pstatic,
				pgeneral,
				ptiming,
				pdynamic);




		pdynamic->system__sequence_config =
				VL53LX_SEQUENCE_VHV_EN |
				VL53LX_SEQUENCE_PHASECAL_EN |
				VL53LX_SEQUENCE_DSS1_EN |
				VL53LX_SEQUENCE_DSS2_EN |


				VL53LX_SEQUENCE_RANGE_EN;




		psystem->system__mode_start =
				VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
				VL53LX_DEVICEREADOUTMODE_DUAL_SD |
				VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_preset_mode_histogram_long_range(
	VL53LX_hist_post_process_config_t  *phistpostprocess,
	VL53LX_static_config_t             *pstatic,
	VL53LX_histogram_config_t          *phistogram,
	VL53LX_general_config_t            *pgeneral,
	VL53LX_timing_config_t             *ptiming,
	VL53LX_dynamic_config_t            *pdynamic,
	VL53LX_system_control_t            *psystem,
	VL53LX_tuning_parm_storage_t       *ptuning_parms,
	VL53LX_zone_config_t               *pzone_cfg)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	status =
		VL53LX_preset_mode_histogram_ranging(
			phistpostprocess,
			pstatic,
			phistogram,
			pgeneral,
			ptiming,
			pdynamic,
			psystem,
			ptuning_parms,
			pzone_cfg);



	if (status == VL53LX_ERROR_NONE) {





		VL53LX_init_histogram_config_structure(
			7, 0, 1, 2, 3, 4,
			0, 1, 2, 3, 4, 5,
			phistogram);


		VL53LX_init_histogram_multizone_config_structure(
			7, 0, 1, 2, 3, 4,
			0, 1, 2, 3, 4, 5,
			&(pzone_cfg->multizone_hist_cfg));



		VL53LX_copy_hist_cfg_to_static_cfg(
			phistogram,
			pstatic,
			pgeneral,
			ptiming,
			pdynamic);



		ptiming->range_config__vcsel_period_a = 0x09;
		ptiming->range_config__vcsel_period_b = 0x0b;



		ptiming->mm_config__timeout_macrop_a_hi = 0x00;
		ptiming->mm_config__timeout_macrop_a_lo = 0x21;
		ptiming->mm_config__timeout_macrop_b_hi = 0x00;
		ptiming->mm_config__timeout_macrop_b_lo = 0x1b;



		ptiming->range_config__timeout_macrop_a_hi = 0x00;
		ptiming->range_config__timeout_macrop_a_lo = 0x29;
		ptiming->range_config__timeout_macrop_b_hi = 0x00;
		ptiming->range_config__timeout_macrop_b_lo = 0x22;



		pgeneral->cal_config__vcsel_start = 0x09;



		pgeneral->phasecal_config__timeout_macrop = 0xF5;



		pdynamic->sd_config__woi_sd0 = 0x09;
		pdynamic->sd_config__woi_sd1 = 0x0B;
		pdynamic->sd_config__initial_phase_sd0            =
				ptuning_parms->tp_init_phase_rtn_hist_long;
		pdynamic->sd_config__initial_phase_sd1            =
				ptuning_parms->tp_init_phase_ref_hist_long;



		phistpostprocess->valid_phase_low = 0x08;
		phistpostprocess->valid_phase_high = 0x88;

		pdynamic->system__sequence_config =
				VL53LX_SEQUENCE_VHV_EN |
				VL53LX_SEQUENCE_PHASECAL_EN |
				VL53LX_SEQUENCE_DSS1_EN |
				VL53LX_SEQUENCE_DSS2_EN |
				VL53LX_SEQUENCE_RANGE_EN;




		psystem->system__mode_start =
				VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
				VL53LX_DEVICEREADOUTMODE_DUAL_SD |
				VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_preset_mode_histogram_medium_range(
	VL53LX_hist_post_process_config_t  *phistpostprocess,
	VL53LX_static_config_t             *pstatic,
	VL53LX_histogram_config_t          *phistogram,
	VL53LX_general_config_t            *pgeneral,
	VL53LX_timing_config_t             *ptiming,
	VL53LX_dynamic_config_t            *pdynamic,
	VL53LX_system_control_t            *psystem,
	VL53LX_tuning_parm_storage_t       *ptuning_parms,
	VL53LX_zone_config_t               *pzone_cfg)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	status =
		VL53LX_preset_mode_histogram_ranging(
			phistpostprocess,
			pstatic,
			phistogram,
			pgeneral,
			ptiming,
			pdynamic,
			psystem,
			ptuning_parms,
			pzone_cfg);



	if (status == VL53LX_ERROR_NONE) {





		VL53LX_init_histogram_config_structure(
				7, 0, 1, 1, 2, 2,
				0, 1, 2, 1, 2, 3,
				phistogram);


		VL53LX_init_histogram_multizone_config_structure(
				7, 0, 1, 1, 2, 2,
				0, 1, 2, 1, 2, 3,
				&(pzone_cfg->multizone_hist_cfg));



		VL53LX_copy_hist_cfg_to_static_cfg(
				phistogram,
				pstatic,
				pgeneral,
				ptiming,
				pdynamic);



		ptiming->range_config__vcsel_period_a = 0x05;
		ptiming->range_config__vcsel_period_b = 0x07;



		ptiming->mm_config__timeout_macrop_a_hi = 0x00;
		ptiming->mm_config__timeout_macrop_a_lo = 0x36;
		ptiming->mm_config__timeout_macrop_b_hi = 0x00;
		ptiming->mm_config__timeout_macrop_b_lo = 0x28;



		ptiming->range_config__timeout_macrop_a_hi = 0x00;
		ptiming->range_config__timeout_macrop_a_lo = 0x44;
		ptiming->range_config__timeout_macrop_b_hi = 0x00;
		ptiming->range_config__timeout_macrop_b_lo = 0x33;



		pgeneral->cal_config__vcsel_start = 0x05;



		pgeneral->phasecal_config__timeout_macrop = 0xF5;



		pdynamic->sd_config__woi_sd0 = 0x05;
		pdynamic->sd_config__woi_sd1 = 0x07;
		pdynamic->sd_config__initial_phase_sd0            =
			ptuning_parms->tp_init_phase_rtn_hist_med;
		pdynamic->sd_config__initial_phase_sd1            =
			ptuning_parms->tp_init_phase_ref_hist_med;



		phistpostprocess->valid_phase_low = 0x08;
		phistpostprocess->valid_phase_high = 0x48;

		pdynamic->system__sequence_config =
				VL53LX_SEQUENCE_VHV_EN |
				VL53LX_SEQUENCE_PHASECAL_EN |
				VL53LX_SEQUENCE_DSS1_EN |
				VL53LX_SEQUENCE_DSS2_EN |
				VL53LX_SEQUENCE_RANGE_EN;




		psystem->system__mode_start =
				VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
				VL53LX_DEVICEREADOUTMODE_DUAL_SD |
				VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_preset_mode_histogram_short_range(
	VL53LX_hist_post_process_config_t  *phistpostprocess,
	VL53LX_static_config_t             *pstatic,
	VL53LX_histogram_config_t          *phistogram,
	VL53LX_general_config_t            *pgeneral,
	VL53LX_timing_config_t             *ptiming,
	VL53LX_dynamic_config_t            *pdynamic,
	VL53LX_system_control_t            *psystem,
	VL53LX_tuning_parm_storage_t       *ptuning_parms,
	VL53LX_zone_config_t               *pzone_cfg)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");



	status =
		VL53LX_preset_mode_histogram_ranging(
			phistpostprocess,
			pstatic,
			phistogram,
			pgeneral,
			ptiming,
			pdynamic,
			psystem,
			ptuning_parms,
			pzone_cfg);



	if (status == VL53LX_ERROR_NONE) {





		VL53LX_init_histogram_config_structure(
				7, 7, 0, 1, 1, 1,
				0, 1, 1, 1, 2, 2,
				phistogram);


		VL53LX_init_histogram_multizone_config_structure(
				7, 7, 0, 1, 1, 1,
				0, 1, 1, 1, 2, 2,
				&(pzone_cfg->multizone_hist_cfg));



		VL53LX_copy_hist_cfg_to_static_cfg(
				phistogram,
				pstatic,
				pgeneral,
				ptiming,
				pdynamic);



		ptiming->range_config__vcsel_period_a = 0x03;
		ptiming->range_config__vcsel_period_b = 0x05;



		ptiming->mm_config__timeout_macrop_a_hi = 0x00;
		ptiming->mm_config__timeout_macrop_a_lo = 0x52;
		ptiming->mm_config__timeout_macrop_b_hi = 0x00;
		ptiming->mm_config__timeout_macrop_b_lo = 0x37;



		ptiming->range_config__timeout_macrop_a_hi = 0x00;
		ptiming->range_config__timeout_macrop_a_lo = 0x66;
		ptiming->range_config__timeout_macrop_b_hi = 0x00;
		ptiming->range_config__timeout_macrop_b_lo = 0x44;



		pgeneral->cal_config__vcsel_start = 0x03;



		pgeneral->phasecal_config__timeout_macrop = 0xF5;



		pdynamic->sd_config__woi_sd0 = 0x03;
		pdynamic->sd_config__woi_sd1 = 0x05;
		pdynamic->sd_config__initial_phase_sd0            =
			ptuning_parms->tp_init_phase_rtn_hist_short;
		pdynamic->sd_config__initial_phase_sd1            =
			ptuning_parms->tp_init_phase_ref_hist_short;


		phistpostprocess->valid_phase_low = 0x08;
		phistpostprocess->valid_phase_high = 0x28;

		pdynamic->system__sequence_config =
				VL53LX_SEQUENCE_VHV_EN |
				VL53LX_SEQUENCE_PHASECAL_EN |
				VL53LX_SEQUENCE_DSS1_EN |
				VL53LX_SEQUENCE_DSS2_EN |
				VL53LX_SEQUENCE_MM1_EN  |

				VL53LX_SEQUENCE_RANGE_EN;




		psystem->system__mode_start =
				VL53LX_DEVICESCHEDULERMODE_HISTOGRAM |
				VL53LX_DEVICEREADOUTMODE_DUAL_SD |
				VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK;
	}

	LOG_FUNCTION_END(status);

	return status;
}


void VL53LX_copy_hist_cfg_to_static_cfg(
	VL53LX_histogram_config_t *phistogram,
	VL53LX_static_config_t    *pstatic,
	VL53LX_general_config_t   *pgeneral,
	VL53LX_timing_config_t    *ptiming,
	VL53LX_dynamic_config_t   *pdynamic)
{


	LOG_FUNCTION_START("");

	SUPPRESS_UNUSED_WARNING(pgeneral);

	pstatic->sigma_estimator__effective_pulse_width_ns =
			phistogram->histogram_config__high_amb_even_bin_0_1;
	pstatic->sigma_estimator__effective_ambient_width_ns =
			phistogram->histogram_config__high_amb_even_bin_2_3;
	pstatic->sigma_estimator__sigma_ref_mm =
			phistogram->histogram_config__high_amb_even_bin_4_5;

	pstatic->algo__crosstalk_compensation_valid_height_mm =
			phistogram->histogram_config__high_amb_odd_bin_0_1;

	pstatic->spare_host_config__static_config_spare_0 =
			phistogram->histogram_config__high_amb_odd_bin_2_3;
	pstatic->spare_host_config__static_config_spare_1 =
			phistogram->histogram_config__high_amb_odd_bin_4_5;

	pstatic->algo__range_ignore_threshold_mcps =
		(((uint16_t)phistogram->histogram_config__mid_amb_even_bin_0_1)
				<< 8)
		+ (uint16_t)phistogram->histogram_config__mid_amb_even_bin_2_3;

	pstatic->algo__range_ignore_valid_height_mm =
			phistogram->histogram_config__mid_amb_even_bin_4_5;
	pstatic->algo__range_min_clip =
			phistogram->histogram_config__mid_amb_odd_bin_0_1;
	pstatic->algo__consistency_check__tolerance =
			phistogram->histogram_config__mid_amb_odd_bin_2;

	pstatic->spare_host_config__static_config_spare_2 =
			phistogram->histogram_config__mid_amb_odd_bin_3_4;
	pstatic->sd_config__reset_stages_msb =
			phistogram->histogram_config__mid_amb_odd_bin_5;

	pstatic->sd_config__reset_stages_lsb =
			phistogram->histogram_config__user_bin_offset;

	ptiming->range_config__sigma_thresh =
		(((uint16_t)phistogram->histogram_config__low_amb_even_bin_0_1)
				<< 8)
		+ (uint16_t)phistogram->histogram_config__low_amb_even_bin_2_3;

	ptiming->range_config__min_count_rate_rtn_limit_mcps =
		(((uint16_t)phistogram->histogram_config__low_amb_even_bin_4_5)
				<< 8)
		+ (uint16_t)phistogram->histogram_config__low_amb_odd_bin_0_1;

	ptiming->range_config__valid_phase_low =
			phistogram->histogram_config__low_amb_odd_bin_2_3;
	ptiming->range_config__valid_phase_high =
			phistogram->histogram_config__low_amb_odd_bin_4_5;

	pdynamic->system__thresh_high =
			phistogram->histogram_config__amb_thresh_low;

	pdynamic->system__thresh_low =
			phistogram->histogram_config__amb_thresh_high;

	pdynamic->system__enable_xtalk_per_quadrant =
			phistogram->histogram_config__spad_array_selection;

	LOG_FUNCTION_END(0);

}

void VL53LX_copy_hist_bins_to_static_cfg(
	VL53LX_histogram_config_t *phistogram,
	VL53LX_static_config_t    *pstatic,
	VL53LX_timing_config_t    *ptiming)
{


	LOG_FUNCTION_START("");

	pstatic->sigma_estimator__effective_pulse_width_ns =
			phistogram->histogram_config__high_amb_even_bin_0_1;
	pstatic->sigma_estimator__effective_ambient_width_ns =
			phistogram->histogram_config__high_amb_even_bin_2_3;
	pstatic->sigma_estimator__sigma_ref_mm =
			phistogram->histogram_config__high_amb_even_bin_4_5;

	pstatic->algo__crosstalk_compensation_valid_height_mm =
			phistogram->histogram_config__high_amb_odd_bin_0_1;

	pstatic->spare_host_config__static_config_spare_0 =
			phistogram->histogram_config__high_amb_odd_bin_2_3;
	pstatic->spare_host_config__static_config_spare_1 =
			phistogram->histogram_config__high_amb_odd_bin_4_5;

	pstatic->algo__range_ignore_threshold_mcps =
		(((uint16_t)phistogram->histogram_config__mid_amb_even_bin_0_1)
				<< 8)
		+ (uint16_t)phistogram->histogram_config__mid_amb_even_bin_2_3;

	pstatic->algo__range_ignore_valid_height_mm =
			phistogram->histogram_config__mid_amb_even_bin_4_5;
	pstatic->algo__range_min_clip =
			phistogram->histogram_config__mid_amb_odd_bin_0_1;
	pstatic->algo__consistency_check__tolerance =
			phistogram->histogram_config__mid_amb_odd_bin_2;

	pstatic->spare_host_config__static_config_spare_2 =
			phistogram->histogram_config__mid_amb_odd_bin_3_4;
	pstatic->sd_config__reset_stages_msb =
			phistogram->histogram_config__mid_amb_odd_bin_5;

	ptiming->range_config__sigma_thresh =
		(((uint16_t)phistogram->histogram_config__low_amb_even_bin_0_1)
				<< 8)
		+ (uint16_t)phistogram->histogram_config__low_amb_even_bin_2_3;

	ptiming->range_config__min_count_rate_rtn_limit_mcps =
		(((uint16_t)phistogram->histogram_config__low_amb_even_bin_4_5)
				<< 8)
		+ (uint16_t)phistogram->histogram_config__low_amb_odd_bin_0_1;

	ptiming->range_config__valid_phase_low =
			phistogram->histogram_config__low_amb_odd_bin_2_3;
	ptiming->range_config__valid_phase_high =
			phistogram->histogram_config__low_amb_odd_bin_4_5;

	LOG_FUNCTION_END(0);

}



