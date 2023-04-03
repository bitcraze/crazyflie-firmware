
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#pragma once


#ifndef _VL53LX_LL_DEF_H_
#define _VL53LX_LL_DEF_H_

#include "vl53lx_platform_user_config.h"
#include "vl53lx_platform_user_defines.h"
#include "vl53lx_error_codes.h"
#include "vl53lx_register_structs.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_dmax_structs.h"
#include "vl53lx_error_exceptions.h"

#ifdef __cplusplus
extern "C" {
#endif




#define VL53LX_LL_API_IMPLEMENTATION_VER_MAJOR       1

#define VL53LX_LL_API_IMPLEMENTATION_VER_MINOR       1

#define VL53LX_LL_API_IMPLEMENTATION_VER_SUB         1

#define VL53LX_LL_API_IMPLEMENTATION_VER_REVISION   0

#define VL53LX_LL_API_IMPLEMENTATION_VER_STRING "1.1.1"


#define VL53LX_FIRMWARE_VER_MINIMUM         398
#define VL53LX_FIRMWARE_VER_MAXIMUM         400




#define VL53LX_LL_CALIBRATION_DATA_STRUCT_VERSION       0xECAB0102




#define VL53LX_LL_ZONE_CALIBRATION_DATA_STRUCT_VERSION  0xECAE0101





#define VL53LX_BIN_REC_SIZE 6

#define VL53LX_TIMING_CONF_A_B_SIZE 2

#define VL53LX_FRAME_WAIT_EVENT	6




#define VL53LX_MAX_XTALK_RANGE_RESULTS        5


#define VL53LX_MAX_OFFSET_RANGE_RESULTS       3


#define VL53LX_NVM_MAX_FMT_RANGE_DATA         4


#define VL53LX_NVM_PEAK_RATE_MAP_SAMPLES  25

#define VL53LX_NVM_PEAK_RATE_MAP_WIDTH     5

#define VL53LX_NVM_PEAK_RATE_MAP_HEIGHT     5




#define VL53LX_ERROR_DEVICE_FIRMWARE_TOO_OLD           ((VL53LX_Error) - 80)

#define VL53LX_ERROR_DEVICE_FIRMWARE_TOO_NEW           ((VL53LX_Error) - 85)

#define VL53LX_ERROR_UNIT_TEST_FAIL                    ((VL53LX_Error) - 90)

#define VL53LX_ERROR_FILE_READ_FAIL                    ((VL53LX_Error) - 95)

#define VL53LX_ERROR_FILE_WRITE_FAIL                   ((VL53LX_Error) - 96)






typedef struct {
	uint32_t     ll_revision;
	uint8_t      ll_major;
	uint8_t      ll_minor;
	uint8_t      ll_build;
} VL53LX_ll_version_t;




typedef struct {

	uint8_t    device_test_mode;
	uint8_t    VL53LX_p_005;
	uint32_t   timeout_us;
	uint16_t   target_count_rate_mcps;

	uint16_t   min_count_rate_limit_mcps;

	uint16_t   max_count_rate_limit_mcps;


} VL53LX_refspadchar_config_t;




typedef struct {

	uint16_t  dss_config__target_total_rate_mcps;

	uint32_t  phasecal_config_timeout_us;

	uint32_t  mm_config_timeout_us;

	uint32_t  range_config_timeout_us;

	uint8_t   num_of_samples;

	int16_t   algo__crosstalk_extract_min_valid_range_mm;

	int16_t   algo__crosstalk_extract_max_valid_range_mm;

	uint16_t  algo__crosstalk_extract_max_valid_rate_kcps;

	uint16_t  algo__crosstalk_extract_max_sigma_mm;


} VL53LX_xtalkextract_config_t;




typedef struct {

	uint16_t  dss_config__target_total_rate_mcps;

	uint32_t  phasecal_config_timeout_us;

	uint32_t  range_config_timeout_us;

	uint32_t  mm_config_timeout_us;

	uint8_t   pre_num_of_samples;

	uint8_t   mm1_num_of_samples;

	uint8_t   mm2_num_of_samples;


} VL53LX_offsetcal_config_t;




typedef struct {

	uint16_t   dss_config__target_total_rate_mcps;

	uint32_t   phasecal_config_timeout_us;

	uint32_t   mm_config_timeout_us;

	uint32_t   range_config_timeout_us;

	uint16_t   phasecal_num_of_samples;

	uint16_t   zone_num_of_samples;


} VL53LX_zonecal_config_t;





typedef struct {

	VL53LX_DeviceSscArray  array_select;

	uint8_t    VL53LX_p_005;

	uint8_t    vcsel_start;

	uint8_t    vcsel_width;

	uint32_t   timeout_us;

	uint16_t   rate_limit_mcps;


} VL53LX_ssc_config_t;




typedef struct {


	uint32_t  algo__crosstalk_compensation_plane_offset_kcps;

	int16_t   algo__crosstalk_compensation_x_plane_gradient_kcps;

	int16_t   algo__crosstalk_compensation_y_plane_gradient_kcps;

	uint32_t  nvm_default__crosstalk_compensation_plane_offset_kcps;

	int16_t   nvm_default__crosstalk_compensation_x_plane_gradient_kcps;

	int16_t   nvm_default__crosstalk_compensation_y_plane_gradient_kcps;

	uint8_t   global_crosstalk_compensation_enable;

	int16_t   histogram_mode_crosstalk_margin_kcps;

	int16_t   lite_mode_crosstalk_margin_kcps;

	uint8_t   crosstalk_range_ignore_threshold_mult;

	uint16_t  crosstalk_range_ignore_threshold_rate_mcps;

	int16_t   algo__crosstalk_detect_min_valid_range_mm;

	int16_t   algo__crosstalk_detect_max_valid_range_mm;

	uint16_t  algo__crosstalk_detect_max_valid_rate_kcps;

	uint16_t  algo__crosstalk_detect_max_sigma_mm;



} VL53LX_xtalk_config_t;




typedef struct {


	uint16_t  tp_tuning_parm_version;

	uint16_t  tp_tuning_parm_key_table_version;

	uint16_t  tp_tuning_parm_lld_version;

	uint8_t   tp_init_phase_rtn_lite_long;

	uint8_t   tp_init_phase_rtn_lite_med;

	uint8_t   tp_init_phase_rtn_lite_short;

	uint8_t   tp_init_phase_ref_lite_long;

	uint8_t   tp_init_phase_ref_lite_med;

	uint8_t   tp_init_phase_ref_lite_short;


	uint8_t   tp_init_phase_rtn_hist_long;

	uint8_t   tp_init_phase_rtn_hist_med;

	uint8_t   tp_init_phase_rtn_hist_short;

	uint8_t   tp_init_phase_ref_hist_long;

	uint8_t   tp_init_phase_ref_hist_med;

	uint8_t   tp_init_phase_ref_hist_short;


	uint8_t   tp_consistency_lite_phase_tolerance;

	uint8_t   tp_phasecal_target;

	uint16_t  tp_cal_repeat_rate;

	uint8_t   tp_lite_min_clip;


	uint16_t  tp_lite_long_sigma_thresh_mm;

	uint16_t  tp_lite_med_sigma_thresh_mm;

	uint16_t  tp_lite_short_sigma_thresh_mm;


	uint16_t  tp_lite_long_min_count_rate_rtn_mcps;

	uint16_t  tp_lite_med_min_count_rate_rtn_mcps;

	uint16_t  tp_lite_short_min_count_rate_rtn_mcps;


	uint8_t   tp_lite_sigma_est_pulse_width_ns;

	uint8_t   tp_lite_sigma_est_amb_width_ns;

	uint8_t   tp_lite_sigma_ref_mm;

	uint8_t   tp_lite_seed_cfg;

	uint8_t   tp_timed_seed_cfg;


	uint8_t   tp_lite_quantifier;

	uint8_t   tp_lite_first_order_select;


	uint16_t  tp_dss_target_lite_mcps;

	uint16_t  tp_dss_target_histo_mcps;

	uint16_t  tp_dss_target_histo_mz_mcps;

	uint16_t  tp_dss_target_timed_mcps;

	uint16_t  tp_dss_target_very_short_mcps;


	uint32_t  tp_phasecal_timeout_lite_us;

	uint32_t  tp_phasecal_timeout_hist_long_us;

	uint32_t  tp_phasecal_timeout_hist_med_us;

	uint32_t  tp_phasecal_timeout_hist_short_us;


	uint32_t  tp_phasecal_timeout_mz_long_us;

	uint32_t  tp_phasecal_timeout_mz_med_us;

	uint32_t  tp_phasecal_timeout_mz_short_us;

	uint32_t  tp_phasecal_timeout_timed_us;


	uint32_t  tp_mm_timeout_lite_us;

	uint32_t  tp_mm_timeout_histo_us;

	uint32_t  tp_mm_timeout_mz_us;

	uint32_t  tp_mm_timeout_timed_us;

	uint32_t  tp_mm_timeout_lpa_us;


	uint32_t  tp_range_timeout_lite_us;

	uint32_t  tp_range_timeout_histo_us;

	uint32_t  tp_range_timeout_mz_us;

	uint32_t  tp_range_timeout_timed_us;

	uint32_t  tp_range_timeout_lpa_us;

	uint32_t tp_phasecal_patch_power;

	uint8_t tp_hist_merge;

	uint32_t tp_reset_merge_threshold;

	uint8_t tp_hist_merge_max_size;


	uint8_t tp_uwr_enable;
	int16_t tp_uwr_med_z_1_min;
	int16_t tp_uwr_med_z_1_max;
	int16_t tp_uwr_med_z_2_min;
	int16_t tp_uwr_med_z_2_max;
	int16_t tp_uwr_med_z_3_min;
	int16_t tp_uwr_med_z_3_max;
	int16_t tp_uwr_med_z_4_min;
	int16_t tp_uwr_med_z_4_max;
	int16_t tp_uwr_med_z_5_min;
	int16_t tp_uwr_med_z_5_max;
	int16_t tp_uwr_med_corr_z_1_rangea;
	int16_t tp_uwr_med_corr_z_1_rangeb;
	int16_t tp_uwr_med_corr_z_2_rangea;
	int16_t tp_uwr_med_corr_z_2_rangeb;
	int16_t tp_uwr_med_corr_z_3_rangea;
	int16_t tp_uwr_med_corr_z_3_rangeb;
	int16_t tp_uwr_med_corr_z_4_rangea;
	int16_t tp_uwr_med_corr_z_4_rangeb;
	int16_t tp_uwr_med_corr_z_5_rangea;
	int16_t tp_uwr_med_corr_z_5_rangeb;
	int16_t tp_uwr_lng_z_1_min;
	int16_t tp_uwr_lng_z_1_max;
	int16_t tp_uwr_lng_z_2_min;
	int16_t tp_uwr_lng_z_2_max;
	int16_t tp_uwr_lng_z_3_min;
	int16_t tp_uwr_lng_z_3_max;
	int16_t tp_uwr_lng_z_4_min;
	int16_t tp_uwr_lng_z_4_max;
	int16_t tp_uwr_lng_z_5_min;
	int16_t tp_uwr_lng_z_5_max;
	int16_t tp_uwr_lng_corr_z_1_rangea;
	int16_t tp_uwr_lng_corr_z_1_rangeb;
	int16_t tp_uwr_lng_corr_z_2_rangea;
	int16_t tp_uwr_lng_corr_z_2_rangeb;
	int16_t tp_uwr_lng_corr_z_3_rangea;
	int16_t tp_uwr_lng_corr_z_3_rangeb;
	int16_t tp_uwr_lng_corr_z_4_rangea;
	int16_t tp_uwr_lng_corr_z_4_rangeb;
	int16_t tp_uwr_lng_corr_z_5_rangea;
	int16_t tp_uwr_lng_corr_z_5_rangeb;

} VL53LX_tuning_parm_storage_t;





typedef struct {

	uint8_t   x_centre;
	uint8_t   y_centre;

} VL53LX_optical_centre_t;




typedef struct {

	uint8_t   x_centre;
	uint8_t   y_centre;
	uint8_t   width;
	uint8_t   height;

} VL53LX_user_zone_t;




typedef struct {

	uint8_t             max_zones;
	uint8_t             active_zones;



VL53LX_histogram_config_t multizone_hist_cfg;

	VL53LX_user_zone_t user_zones[VL53LX_MAX_USER_ZONES];


	uint8_t bin_config[VL53LX_MAX_USER_ZONES];


} VL53LX_zone_config_t;



typedef struct {


	VL53LX_GPIO_Interrupt_Mode	intr_mode_distance;


	VL53LX_GPIO_Interrupt_Mode	intr_mode_rate;


	uint8_t				intr_new_measure_ready;


	uint8_t				intr_no_target;


	uint8_t				intr_combined_mode;





	uint16_t			threshold_distance_high;


	uint16_t			threshold_distance_low;


	uint16_t			threshold_rate_high;


	uint16_t			threshold_rate_low;

} VL53LX_GPIO_interrupt_config_t;




typedef struct {


	uint8_t		vhv_loop_bound;


	uint8_t		is_low_power_auto_mode;


	uint8_t		low_power_auto_range_count;


	uint8_t		saved_interrupt_config;


	uint8_t		saved_vhv_init;


	uint8_t		saved_vhv_timeout;


	uint8_t		first_run_phasecal_result;


	uint32_t	dss__total_rate_per_spad_mcps;


	uint16_t	dss__required_spads;

} VL53LX_low_power_auto_data_t;







typedef struct {


	uint8_t	smudge_corr_enabled;


	uint8_t	smudge_corr_apply_enabled;


	uint8_t	smudge_corr_single_apply;




	uint16_t	smudge_margin;


	uint32_t	noise_margin;


	uint32_t	user_xtalk_offset_limit;


	uint8_t	user_xtalk_offset_limit_hi;


	uint32_t	sample_limit;


	uint32_t	single_xtalk_delta;


	uint32_t	averaged_xtalk_delta;


	uint32_t	smudge_corr_clip_limit;


	uint32_t	smudge_corr_ambient_threshold;


	uint8_t	scaler_calc_method;


	int16_t	x_gradient_scaler;


	int16_t	y_gradient_scaler;


	uint8_t	user_scaler_set;


	uint32_t nodetect_ambient_threshold;


	uint32_t nodetect_sample_limit;


	uint32_t nodetect_xtalk_offset;


	uint16_t nodetect_min_range_mm;


	uint32_t max_smudge_factor;

} VL53LX_smudge_corrector_config_t;



typedef struct {


	uint32_t	current_samples;


	uint32_t	required_samples;


	uint64_t	accumulator;


	uint32_t	nodetect_counter;

} VL53LX_smudge_corrector_internals_t;



typedef struct {


	uint8_t	smudge_corr_valid;


	uint8_t	smudge_corr_clipped;


	uint8_t	single_xtalk_delta_flag;


	uint8_t	averaged_xtalk_delta_flag;


	uint8_t	sample_limit_exceeded_flag;


	uint8_t gradient_zero_flag;


	uint8_t new_xtalk_applied_flag;


	uint32_t  algo__crosstalk_compensation_plane_offset_kcps;


	int16_t   algo__crosstalk_compensation_x_plane_gradient_kcps;


	int16_t   algo__crosstalk_compensation_y_plane_gradient_kcps;


} VL53LX_smudge_corrector_data_t;





typedef struct {



	uint8_t  range_id;

	uint32_t time_stamp;

	uint8_t  VL53LX_p_012;

	uint8_t  VL53LX_p_019;

	uint8_t  VL53LX_p_023;

	uint8_t  VL53LX_p_024;

	uint8_t  VL53LX_p_013;

	uint8_t  VL53LX_p_025;


	uint16_t   width;

	uint8_t    VL53LX_p_029;


	uint16_t   fast_osc_frequency;

	uint16_t   zero_distance_phase;

	uint16_t   VL53LX_p_004;


	uint32_t   total_periods_elapsed;


	uint32_t   peak_duration_us;


	uint32_t   woi_duration_us;





	uint32_t   VL53LX_p_016;

	uint32_t   VL53LX_p_017;

	int32_t    VL53LX_p_010;




	uint16_t    peak_signal_count_rate_mcps;

	uint16_t    avg_signal_count_rate_mcps;

	uint16_t    ambient_count_rate_mcps;

	uint16_t    total_rate_per_spad_mcps;

	uint32_t    VL53LX_p_009;




	uint16_t   VL53LX_p_002;




	uint16_t   VL53LX_p_026;

	uint16_t   VL53LX_p_011;

	uint16_t   VL53LX_p_027;




	int16_t    min_range_mm;

	int16_t    median_range_mm;

	int16_t    max_range_mm;




	uint8_t    range_status;

} VL53LX_range_data_t;




typedef struct {

	VL53LX_DeviceState     cfg_device_state;

	VL53LX_DeviceState     rd_device_state;

	uint8_t                zone_id;

	uint8_t                stream_count;


	int16_t                VL53LX_p_022[VL53LX_MAX_AMBIENT_DMAX_VALUES];

	int16_t                wrap_dmax_mm;


	uint8_t                device_status;


	uint8_t                max_results;

	uint8_t                active_results;

	VL53LX_range_data_t    VL53LX_p_003[VL53LX_MAX_RANGE_RESULTS];

	VL53LX_range_data_t    xmonitor;

	VL53LX_smudge_corrector_data_t smudge_corrector_data;


} VL53LX_range_results_t;




typedef struct {

	uint8_t    no_of_samples;

	uint32_t   rate_per_spad_kcps_sum;

	uint32_t   rate_per_spad_kcps_avg;

	int32_t    signal_total_events_sum;

	int32_t    signal_total_events_avg;

	uint32_t   sigma_mm_sum;

	uint32_t   sigma_mm_avg;

	uint32_t   median_phase_sum;

	uint32_t   median_phase_avg;


} VL53LX_xtalk_range_data_t;




typedef struct {

	VL53LX_Error                cal_status;

	uint8_t                     num_of_samples_status;

	uint8_t                     zero_samples_status;

	uint8_t                     max_sigma_status;

	uint8_t                     max_results;

	uint8_t                     active_results;


	VL53LX_xtalk_range_data_t
		VL53LX_p_003[VL53LX_MAX_XTALK_RANGE_RESULTS];

	VL53LX_histogram_bin_data_t central_histogram_sum;

	VL53LX_histogram_bin_data_t central_histogram_avg;

	uint8_t central_histogram__window_start;

	uint8_t central_histogram__window_end;

	VL53LX_histogram_bin_data_t
		histogram_avg_1[VL53LX_MAX_XTALK_RANGE_RESULTS];

	VL53LX_histogram_bin_data_t
		histogram_avg_2[VL53LX_MAX_XTALK_RANGE_RESULTS];

	VL53LX_histogram_bin_data_t
		xtalk_avg[VL53LX_MAX_XTALK_RANGE_RESULTS];


} VL53LX_xtalk_range_results_t;




typedef struct {

	uint8_t    preset_mode;

	uint8_t    dss_config__roi_mode_control;

	uint16_t   dss_config__manual_effective_spads_select;

	uint8_t    no_of_samples;

	uint32_t   effective_spads;

	uint32_t   peak_rate_mcps;

	uint32_t   VL53LX_p_002;

	int32_t    median_range_mm;

	int32_t    range_mm_offset;


} VL53LX_offset_range_data_t;




typedef struct {

	int16_t      cal_distance_mm;

	uint16_t     cal_reflectance_pc;

	VL53LX_Error cal_status;

	uint8_t      cal_report;

	uint8_t      max_results;

	uint8_t      active_results;

	VL53LX_offset_range_data_t
		VL53LX_p_003[VL53LX_MAX_OFFSET_RANGE_RESULTS];


} VL53LX_offset_range_results_t;




typedef struct {

	uint16_t  result__mm_inner_actual_effective_spads;

	uint16_t  result__mm_outer_actual_effective_spads;

	uint16_t  result__mm_inner_peak_signal_count_rtn_mcps;

	uint16_t  result__mm_outer_peak_signal_count_rtn_mcps;


} VL53LX_additional_offset_cal_data_t;



typedef struct {
	int16_t   short_a_offset_mm;
	int16_t   short_b_offset_mm;
	int16_t   medium_a_offset_mm;
	int16_t   medium_b_offset_mm;
	int16_t   long_a_offset_mm;
	int16_t   long_b_offset_mm;
} VL53LX_per_vcsel_period_offset_cal_data_t;





typedef struct {

	uint32_t   VL53LX_p_016;

	uint32_t   VL53LX_p_017;

	uint16_t   VL53LX_p_011;

	uint8_t    range_status;


} VL53LX_object_data_t;




typedef struct {

	VL53LX_DeviceState     cfg_device_state;

	VL53LX_DeviceState     rd_device_state;

	uint8_t                zone_id;

	uint8_t                stream_count;

	uint8_t                max_objects;

	uint8_t                active_objects;

	VL53LX_object_data_t   VL53LX_p_003[VL53LX_MAX_RANGE_RESULTS];


	VL53LX_object_data_t   xmonitor;


} VL53LX_zone_objects_t;






typedef struct {

	uint8_t                max_zones;

	uint8_t                active_zones;

	VL53LX_zone_objects_t VL53LX_p_003[VL53LX_MAX_USER_ZONES];


} VL53LX_zone_results_t;




typedef struct {

	VL53LX_DeviceState     rd_device_state;


	uint8_t  number_of_ambient_bins;


	uint16_t result__dss_actual_effective_spads;

	uint8_t  VL53LX_p_005;

	uint32_t total_periods_elapsed;


	int32_t  ambient_events_sum;


} VL53LX_zone_hist_info_t;




typedef struct {

	uint8_t                     max_zones;

	uint8_t                     active_zones;

	VL53LX_zone_hist_info_t     VL53LX_p_003[VL53LX_MAX_USER_ZONES];


} VL53LX_zone_histograms_t;




typedef struct {

	uint32_t   no_of_samples;

	uint32_t   effective_spads;

	uint32_t   peak_rate_mcps;

	uint32_t   VL53LX_p_011;

	uint32_t   VL53LX_p_002;

	int32_t    median_range_mm;

	int32_t    range_mm_offset;


} VL53LX_zone_calibration_data_t;






typedef struct {

	uint32_t                         struct_version;

	VL53LX_DevicePresetModes         preset_mode;

	VL53LX_DeviceZonePreset          zone_preset;

	int16_t                          cal_distance_mm;

	uint16_t                         cal_reflectance_pc;

	uint16_t                         phasecal_result__reference_phase;

	uint16_t                         zero_distance_phase;

	VL53LX_Error                     cal_status;

	uint8_t                          max_zones;

	uint8_t                          active_zones;

	VL53LX_zone_calibration_data_t   VL53LX_p_003[VL53LX_MAX_USER_ZONES];


} VL53LX_zone_calibration_results_t;





typedef struct {

	int16_t     cal_distance_mm;

	uint16_t    cal_reflectance_pc;

	uint16_t    max_samples;

	uint16_t    width;

	uint16_t    height;

	uint16_t    peak_rate_mcps[VL53LX_NVM_PEAK_RATE_MAP_SAMPLES];


} VL53LX_cal_peak_rate_map_t;




typedef struct {

	uint8_t      expected_stream_count;

	uint8_t      expected_gph_id;

	uint8_t      dss_mode;

	uint16_t     dss_requested_effective_spad_count;

	uint8_t      seed_cfg;

	uint8_t      initial_phase_seed;


	uint8_t  roi_config__user_roi_centre_spad;

	uint8_t  roi_config__user_roi_requested_global_xy_size;


} VL53LX_zone_private_dyn_cfg_t;




typedef struct {

	uint8_t                     max_zones;

	uint8_t                     active_zones;

	VL53LX_zone_private_dyn_cfg_t VL53LX_p_003[VL53LX_MAX_USER_ZONES];


} VL53LX_zone_private_dyn_cfgs_t;



typedef struct {

	uint32_t  algo__crosstalk_compensation_plane_offset_kcps;

	int16_t   algo__crosstalk_compensation_x_plane_gradient_kcps;

	int16_t   algo__crosstalk_compensation_y_plane_gradient_kcps;

	uint32_t  algo__xtalk_cpo_HistoMerge_kcps[VL53LX_BIN_REC_SIZE];


} VL53LX_xtalk_calibration_results_t;




typedef struct {


	uint32_t   sample_count;


	uint32_t   pll_period_mm;


	uint32_t   peak_duration_us_sum;


	uint32_t   effective_spad_count_sum;


	uint32_t   zero_distance_phase_sum;


	uint32_t   zero_distance_phase_avg;


	int32_t    event_scaler_sum;


	int32_t    event_scaler_avg;


	int32_t   signal_events_sum;


	uint32_t  xtalk_rate_kcps_per_spad;


	int32_t   xtalk_start_phase;


	int32_t   xtalk_end_phase;


	int32_t   xtalk_width_phase;


	int32_t   target_start_phase;


	int32_t   target_end_phase;


	int32_t   target_width_phase;


	int32_t   effective_width;


	int32_t   event_scaler;


	uint8_t   VL53LX_p_012;


	uint8_t   VL53LX_p_013;


	uint8_t   target_start;


	int32_t   max_shape_value;


	int32_t   bin_data_sums[VL53LX_XTALK_HISTO_BINS];

} VL53LX_hist_xtalk_extract_data_t;




typedef struct {

	uint16_t   standard_ranging_gain_factor;

	uint16_t   histogram_ranging_gain_factor;


} VL53LX_gain_calibration_data_t;




typedef struct {

	VL53LX_DeviceState   cfg_device_state;

	uint8_t   cfg_stream_count;

	uint8_t   cfg_internal_stream_count;

	uint8_t   cfg_internal_stream_count_val;

	uint8_t   cfg_gph_id;

	uint8_t   cfg_timing_status;

	uint8_t   cfg_zone_id;


	VL53LX_DeviceState   rd_device_state;

	uint8_t   rd_stream_count;

	uint8_t   rd_internal_stream_count;

	uint8_t   rd_internal_stream_count_val;

	uint8_t   rd_gph_id;

	uint8_t   rd_timing_status;

	uint8_t   rd_zone_id;


} VL53LX_ll_driver_state_t;




typedef struct {

	uint8_t   wait_method;

	VL53LX_DevicePresetModes        preset_mode;

	VL53LX_DeviceZonePreset         zone_preset;

	VL53LX_DeviceMeasurementModes   measurement_mode;

	VL53LX_OffsetCalibrationMode    offset_calibration_mode;

	VL53LX_OffsetCorrectionMode     offset_correction_mode;

	VL53LX_DeviceDmaxMode           dmax_mode;

	uint32_t  phasecal_config_timeout_us;

	uint32_t  mm_config_timeout_us;

	uint32_t  range_config_timeout_us;

	uint32_t  inter_measurement_period_ms;

	uint16_t  dss_config__target_total_rate_mcps;

	uint32_t  fw_ready_poll_duration_ms;

	uint8_t   fw_ready;

	uint8_t   debug_mode;



	VL53LX_ll_version_t                 version;


	VL53LX_ll_driver_state_t            ll_state;


	VL53LX_GPIO_interrupt_config_t	    gpio_interrupt_config;


	VL53LX_customer_nvm_managed_t       customer;
	VL53LX_cal_peak_rate_map_t          cal_peak_rate_map;
	VL53LX_additional_offset_cal_data_t add_off_cal_data;
	VL53LX_dmax_calibration_data_t      fmt_dmax_cal;
	VL53LX_dmax_calibration_data_t      cust_dmax_cal;
	VL53LX_gain_calibration_data_t      gain_cal;
	VL53LX_user_zone_t                  mm_roi;
	VL53LX_optical_centre_t             optical_centre;
	VL53LX_zone_config_t                zone_cfg;


	VL53LX_tuning_parm_storage_t        tuning_parms;


	uint8_t rtn_good_spads[VL53LX_RTN_SPAD_BUFFER_SIZE];


	VL53LX_refspadchar_config_t         refspadchar;
	VL53LX_ssc_config_t                 ssc_cfg;
	VL53LX_hist_post_process_config_t   histpostprocess;
	VL53LX_hist_gen3_dmax_config_t      dmax_cfg;
	VL53LX_xtalkextract_config_t        xtalk_extract_cfg;
	VL53LX_xtalk_config_t               xtalk_cfg;
	VL53LX_offsetcal_config_t           offsetcal_cfg;
	VL53LX_zonecal_config_t             zonecal_cfg;


	VL53LX_static_nvm_managed_t         stat_nvm;
	VL53LX_histogram_config_t           hist_cfg;
	VL53LX_static_config_t              stat_cfg;
	VL53LX_general_config_t             gen_cfg;
	VL53LX_timing_config_t              tim_cfg;
	VL53LX_dynamic_config_t             dyn_cfg;
	VL53LX_system_control_t             sys_ctrl;
	VL53LX_system_results_t             sys_results;
	VL53LX_nvm_copy_data_t              nvm_copy_data;


	VL53LX_histogram_bin_data_t         hist_data;
	VL53LX_histogram_bin_data_t         hist_xtalk;


	VL53LX_xtalk_histogram_data_t       xtalk_shapes;
	// VL53LX_xtalk_range_results_t        xtalk_results;
	VL53LX_xtalk_calibration_results_t  xtalk_cal;
	VL53LX_hist_xtalk_extract_data_t    xtalk_extract;


	VL53LX_offset_range_results_t       offset_results;


	VL53LX_core_results_t               core_results;
	VL53LX_debug_results_t              dbg_results;

	VL53LX_smudge_corrector_config_t	smudge_correct_config;

	VL53LX_smudge_corrector_internals_t smudge_corrector_internals;




	VL53LX_low_power_auto_data_t		low_power_auto_data;

	uint8_t  wArea1[1536];
	uint8_t  wArea2[512];
	VL53LX_per_vcsel_period_offset_cal_data_t per_vcsel_cal_data;

	uint8_t bin_rec_pos;

	uint8_t pos_before_next_recom;

	int32_t  multi_bins_rec[VL53LX_BIN_REC_SIZE]
		[VL53LX_TIMING_CONF_A_B_SIZE][VL53LX_HISTOGRAM_BUFFER_SIZE];

	int16_t PreviousRangeMilliMeter[VL53LX_MAX_RANGE_RESULTS];
	uint8_t PreviousRangeStatus[VL53LX_MAX_RANGE_RESULTS];
	uint8_t PreviousExtendedRange[VL53LX_MAX_RANGE_RESULTS];
	uint8_t PreviousRangeActiveResults;
	uint8_t PreviousStreamCount;
} VL53LX_LLDriverData_t;

typedef struct {


	VL53LX_range_results_t             range_results;


	VL53LX_zone_private_dyn_cfgs_t     zone_dyn_cfgs;


	VL53LX_zone_results_t              zone_results;
	VL53LX_zone_histograms_t           zone_hists;
	VL53LX_zone_calibration_results_t  zone_cal;

} VL53LX_LLDriverResults_t;




typedef struct {

	uint32_t                             struct_version;
	VL53LX_customer_nvm_managed_t        customer;
	VL53LX_dmax_calibration_data_t       fmt_dmax_cal;
	VL53LX_dmax_calibration_data_t       cust_dmax_cal;
	VL53LX_additional_offset_cal_data_t  add_off_cal_data;
	VL53LX_optical_centre_t              optical_centre;
	VL53LX_xtalk_histogram_data_t        xtalkhisto;
	VL53LX_gain_calibration_data_t       gain_cal;
	VL53LX_cal_peak_rate_map_t           cal_peak_rate_map;
	VL53LX_per_vcsel_period_offset_cal_data_t per_vcsel_cal_data;

} VL53LX_calibration_data_t;




typedef struct {

	VL53LX_customer_nvm_managed_t        customer;
	VL53LX_xtalkextract_config_t         xtalk_extract_cfg;
	VL53LX_xtalk_config_t                xtalk_cfg;
	VL53LX_histogram_bin_data_t          hist_data;
	VL53LX_xtalk_histogram_data_t        xtalk_shapes;
	// VL53LX_xtalk_range_results_t         xtalk_results;

} VL53LX_xtalk_debug_data_t;




typedef struct {

	VL53LX_customer_nvm_managed_t        customer;
	VL53LX_dmax_calibration_data_t       fmt_dmax_cal;
	VL53LX_dmax_calibration_data_t       cust_dmax_cal;
	VL53LX_additional_offset_cal_data_t  add_off_cal_data;
	VL53LX_offset_range_results_t        offset_results;

} VL53LX_offset_debug_data_t;




typedef struct {
	uint16_t        vl53lx_tuningparm_version;
	uint16_t        vl53lx_tuningparm_key_table_version;
	uint16_t        vl53lx_tuningparm_lld_version;
	uint8_t        vl53lx_tuningparm_hist_algo_select;
	uint8_t        vl53lx_tuningparm_hist_target_order;
	uint8_t        vl53lx_tuningparm_hist_filter_woi_0;
	uint8_t        vl53lx_tuningparm_hist_filter_woi_1;
	uint8_t        vl53lx_tuningparm_hist_amb_est_method;
	uint8_t        vl53lx_tuningparm_hist_amb_thresh_sigma_0;
	uint8_t        vl53lx_tuningparm_hist_amb_thresh_sigma_1;
	int32_t        vl53lx_tuningparm_hist_min_amb_thresh_events;
	uint16_t        vl53lx_tuningparm_hist_amb_events_scaler;
	uint16_t        vl53lx_tuningparm_hist_noise_threshold;
	int32_t        vl53lx_tuningparm_hist_signal_total_events_limit;
	uint8_t        vl53lx_tuningparm_hist_sigma_est_ref_mm;
	uint16_t        vl53lx_tuningparm_hist_sigma_thresh_mm;
	uint16_t        vl53lx_tuningparm_hist_gain_factor;
	uint8_t        vl53lx_tuningparm_consistency_hist_phase_tolerance;
	uint16_t  vl53lx_tuningparm_consistency_hist_min_max_tolerance_mm;
	uint8_t        vl53lx_tuningparm_consistency_hist_event_sigma;
	uint16_t  vl53lx_tuningparm_consistency_hist_event_sigma_min_spad_limit;
	uint8_t        vl53lx_tuningparm_initial_phase_rtn_histo_long_range;
	uint8_t        vl53lx_tuningparm_initial_phase_rtn_histo_med_range;
	uint8_t        vl53lx_tuningparm_initial_phase_rtn_histo_short_range;
	uint8_t        vl53lx_tuningparm_initial_phase_ref_histo_long_range;
	uint8_t        vl53lx_tuningparm_initial_phase_ref_histo_med_range;
	uint8_t        vl53lx_tuningparm_initial_phase_ref_histo_short_range;
	int16_t        vl53lx_tuningparm_xtalk_detect_min_valid_range_mm;
	int16_t        vl53lx_tuningparm_xtalk_detect_max_valid_range_mm;
	uint16_t        vl53lx_tuningparm_xtalk_detect_max_sigma_mm;
	uint16_t        vl53lx_tuningparm_xtalk_detect_min_max_tolerance;
	uint16_t        vl53lx_tuningparm_xtalk_detect_max_valid_rate_kcps;
	uint8_t        vl53lx_tuningparm_xtalk_detect_event_sigma;
	int16_t        vl53lx_tuningparm_hist_xtalk_margin_kcps;
	uint8_t        vl53lx_tuningparm_consistency_lite_phase_tolerance;
	uint8_t        vl53lx_tuningparm_phasecal_target;
	uint16_t        vl53lx_tuningparm_lite_cal_repeat_rate;
	uint16_t        vl53lx_tuningparm_lite_ranging_gain_factor;
	uint8_t        vl53lx_tuningparm_lite_min_clip_mm;
	uint16_t        vl53lx_tuningparm_lite_long_sigma_thresh_mm;
	uint16_t        vl53lx_tuningparm_lite_med_sigma_thresh_mm;
	uint16_t        vl53lx_tuningparm_lite_short_sigma_thresh_mm;
	uint16_t        vl53lx_tuningparm_lite_long_min_count_rate_rtn_mcps;
	uint16_t        vl53lx_tuningparm_lite_med_min_count_rate_rtn_mcps;
	uint16_t        vl53lx_tuningparm_lite_short_min_count_rate_rtn_mcps;
	uint8_t        vl53lx_tuningparm_lite_sigma_est_pulse_width;
	uint8_t        vl53lx_tuningparm_lite_sigma_est_amb_width_ns;
	uint8_t        vl53lx_tuningparm_lite_sigma_ref_mm;
	uint8_t        vl53lx_tuningparm_lite_rit_mult;
	uint8_t        vl53lx_tuningparm_lite_seed_config;
	uint8_t        vl53lx_tuningparm_lite_quantifier;
	uint8_t        vl53lx_tuningparm_lite_first_order_select;
	int16_t        vl53lx_tuningparm_lite_xtalk_margin_kcps;
	uint8_t        vl53lx_tuningparm_initial_phase_rtn_lite_long_range;
	uint8_t        vl53lx_tuningparm_initial_phase_rtn_lite_med_range;
	uint8_t        vl53lx_tuningparm_initial_phase_rtn_lite_short_range;
	uint8_t        vl53lx_tuningparm_initial_phase_ref_lite_long_range;
	uint8_t        vl53lx_tuningparm_initial_phase_ref_lite_med_range;
	uint8_t        vl53lx_tuningparm_initial_phase_ref_lite_short_range;
	uint8_t        vl53lx_tuningparm_timed_seed_config;
	uint8_t        vl53lx_tuningparm_dmax_cfg_signal_thresh_sigma;
	uint16_t        vl53lx_tuningparm_dmax_cfg_reflectance_array_0;
	uint16_t        vl53lx_tuningparm_dmax_cfg_reflectance_array_1;
	uint16_t        vl53lx_tuningparm_dmax_cfg_reflectance_array_2;
	uint16_t        vl53lx_tuningparm_dmax_cfg_reflectance_array_3;
	uint16_t        vl53lx_tuningparm_dmax_cfg_reflectance_array_4;
	uint8_t        vl53lx_tuningparm_vhv_loopbound;
	uint8_t        vl53lx_tuningparm_refspadchar_device_test_mode;
	uint8_t        vl53lx_tuningparm_refspadchar_vcsel_period;
	uint32_t        vl53lx_tuningparm_refspadchar_phasecal_timeout_us;
	uint16_t        vl53lx_tuningparm_refspadchar_target_count_rate_mcps;
	uint16_t        vl53lx_tuningparm_refspadchar_min_countrate_limit_mcps;
	uint16_t        vl53lx_tuningparm_refspadchar_max_countrate_limit_mcps;
	uint8_t        vl53lx_tuningparm_xtalk_extract_num_of_samples;
	int16_t        vl53lx_tuningparm_xtalk_extract_min_filter_thresh_mm;
	int16_t        vl53lx_tuningparm_xtalk_extract_max_filter_thresh_mm;
	uint16_t        vl53lx_tuningparm_xtalk_extract_dss_rate_mcps;
	uint32_t        vl53lx_tuningparm_xtalk_extract_phasecal_timeout_us;
	uint16_t        vl53lx_tuningparm_xtalk_extract_max_valid_rate_kcps;
	uint16_t        vl53lx_tuningparm_xtalk_extract_sigma_threshold_mm;
	uint32_t        vl53lx_tuningparm_xtalk_extract_dss_timeout_us;
	uint32_t        vl53lx_tuningparm_xtalk_extract_bin_timeout_us;
	uint16_t        vl53lx_tuningparm_offset_cal_dss_rate_mcps;
	uint32_t        vl53lx_tuningparm_offset_cal_phasecal_timeout_us;
	uint32_t        vl53lx_tuningparm_offset_cal_mm_timeout_us;
	uint32_t        vl53lx_tuningparm_offset_cal_range_timeout_us;
	uint8_t        vl53lx_tuningparm_offset_cal_pre_samples;
	uint8_t        vl53lx_tuningparm_offset_cal_mm1_samples;
	uint8_t        vl53lx_tuningparm_offset_cal_mm2_samples;
	uint16_t        vl53lx_tuningparm_zone_cal_dss_rate_mcps;
	uint32_t        vl53lx_tuningparm_zone_cal_phasecal_timeout_us;
	uint32_t        vl53lx_tuningparm_zone_cal_dss_timeout_us;
	uint16_t        vl53lx_tuningparm_zone_cal_phasecal_num_samples;
	uint32_t        vl53lx_tuningparm_zone_cal_range_timeout_us;
	uint16_t        vl53lx_tuningparm_zone_cal_zone_num_samples;
	uint8_t        vl53lx_tuningparm_spadmap_vcsel_period;
	uint8_t        vl53lx_tuningparm_spadmap_vcsel_start;
	uint16_t        vl53lx_tuningparm_spadmap_rate_limit_mcps;
	uint16_t  vl53lx_tuningparm_lite_dss_config_target_total_rate_mcps;
	uint16_t   vl53lx_tuningparm_ranging_dss_config_target_total_rate_mcps;
	uint16_t        vl53lx_tuningparm_mz_dss_config_target_total_rate_mcps;
	uint16_t     vl53lx_tuningparm_timed_dss_config_target_total_rate_mcps;
	uint32_t        vl53lx_tuningparm_lite_phasecal_config_timeout_us;
	uint32_t     vl53lx_tuningparm_ranging_long_phasecal_config_timeout_us;
	uint32_t      vl53lx_tuningparm_ranging_med_phasecal_config_timeout_us;
	uint32_t    vl53lx_tuningparm_ranging_short_phasecal_config_timeout_us;
	uint32_t        vl53lx_tuningparm_mz_long_phasecal_config_timeout_us;
	uint32_t        vl53lx_tuningparm_mz_med_phasecal_config_timeout_us;
	uint32_t        vl53lx_tuningparm_mz_short_phasecal_config_timeout_us;
	uint32_t        vl53lx_tuningparm_timed_phasecal_config_timeout_us;
	uint32_t        vl53lx_tuningparm_lite_mm_config_timeout_us;
	uint32_t        vl53lx_tuningparm_ranging_mm_config_timeout_us;
	uint32_t        vl53lx_tuningparm_mz_mm_config_timeout_us;
	uint32_t        vl53lx_tuningparm_timed_mm_config_timeout_us;
	uint32_t        vl53lx_tuningparm_lite_range_config_timeout_us;
	uint32_t        vl53lx_tuningparm_ranging_range_config_timeout_us;
	uint32_t        vl53lx_tuningparm_mz_range_config_timeout_us;
	uint32_t        vl53lx_tuningparm_timed_range_config_timeout_us;
	uint16_t        vl53lx_tuningparm_dynxtalk_smudge_margin;
	uint32_t        vl53lx_tuningparm_dynxtalk_noise_margin;
	uint32_t        vl53lx_tuningparm_dynxtalk_xtalk_offset_limit;
	uint8_t        vl53lx_tuningparm_dynxtalk_xtalk_offset_limit_hi;
	uint32_t        vl53lx_tuningparm_dynxtalk_sample_limit;
	uint32_t        vl53lx_tuningparm_dynxtalk_single_xtalk_delta;
	uint32_t        vl53lx_tuningparm_dynxtalk_averaged_xtalk_delta;
	uint32_t        vl53lx_tuningparm_dynxtalk_clip_limit;
	uint8_t        vl53lx_tuningparm_dynxtalk_scaler_calc_method;
	int16_t        vl53lx_tuningparm_dynxtalk_xgradient_scaler;
	int16_t        vl53lx_tuningparm_dynxtalk_ygradient_scaler;
	uint8_t        vl53lx_tuningparm_dynxtalk_user_scaler_set;
	uint8_t        vl53lx_tuningparm_dynxtalk_smudge_cor_single_apply;
	uint32_t        vl53lx_tuningparm_dynxtalk_xtalk_amb_threshold;
	uint32_t        vl53lx_tuningparm_dynxtalk_nodetect_amb_threshold_kcps;
	uint32_t        vl53lx_tuningparm_dynxtalk_nodetect_sample_limit;
	uint32_t        vl53lx_tuningparm_dynxtalk_nodetect_xtalk_offset_kcps;
	uint16_t        vl53lx_tuningparm_dynxtalk_nodetect_min_range_mm;
	uint8_t        vl53lx_tuningparm_lowpowerauto_vhv_loop_bound;
	uint32_t        vl53lx_tuningparm_lowpowerauto_mm_config_timeout_us;
	uint32_t        vl53lx_tuningparm_lowpowerauto_range_config_timeout_us;
	uint16_t        vl53lx_tuningparm_very_short_dss_rate_mcps;
	uint32_t        vl53lx_tuningparm_phasecal_patch_power;
} VL53LX_tuning_parameters_t;





typedef struct {

	uint16_t  target_reflectance_for_dmax[VL53LX_MAX_AMBIENT_DMAX_VALUES];

} VL53LX_dmax_reflectance_array_t;




typedef struct {

	uint8_t    spad_type;

	uint16_t   VL53LX_p_020;

	uint16_t   rate_data[VL53LX_NO_OF_SPAD_ENABLES];

	uint16_t    no_of_values;

	uint8_t    fractional_bits;

	uint8_t    error_status;


} VL53LX_spad_rate_data_t;






typedef struct {

	VL53LX_DevicePresetModes        preset_mode;

	VL53LX_DeviceZonePreset         zone_preset;

	VL53LX_DeviceMeasurementModes   measurement_mode;

	VL53LX_OffsetCalibrationMode    offset_calibration_mode;

	VL53LX_OffsetCorrectionMode     offset_correction_mode;

	VL53LX_DeviceDmaxMode           dmax_mode;


	uint32_t  phasecal_config_timeout_us;

	uint32_t  mm_config_timeout_us;

	uint32_t  range_config_timeout_us;

	uint32_t  inter_measurement_period_ms;

	uint16_t  dss_config__target_total_rate_mcps;


	VL53LX_histogram_bin_data_t    VL53LX_p_006;


} VL53LX_additional_data_t;








#define SUPPRESS_UNUSED_WARNING(x) \
	((void) (x))


#define IGNORE_STATUS(__FUNCTION_ID__, __ERROR_STATUS_CHECK__, __STATUS__) \
	do { \
		DISABLE_WARNINGS(); \
		if (__FUNCTION_ID__) { \
			if (__STATUS__ == __ERROR_STATUS_CHECK__) { \
				__STATUS__ = VL53LX_ERROR_NONE; \
				WARN_OVERRIDE_STATUS(__FUNCTION_ID__); \
			} \
		} \
		ENABLE_WARNINGS(); \
	} \
	while (0)

#define VL53LX_COPYSTRING(str, ...) \
	(strncpy(str, ##__VA_ARGS__, VL53LX_MAX_STRING_LENGTH-1))

#ifdef __cplusplus
}
#endif

#endif



