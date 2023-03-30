
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */


#ifndef _VL53LX_REGISTER_STRUCTS_H_
#define _VL53LX_REGISTER_STRUCTS_H_

#include "vl53lx_types.h"
#include "vl53lx_register_map.h"

#define VL53LX_STATIC_NVM_MANAGED_I2C_INDEX             \
	VL53LX_I2C_SLAVE__DEVICE_ADDRESS
#define VL53LX_CUSTOMER_NVM_MANAGED_I2C_INDEX           \
	VL53LX_GLOBAL_CONFIG__SPAD_ENABLES_REF_0
#define VL53LX_STATIC_CONFIG_I2C_INDEX                  \
	VL53LX_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS
#define VL53LX_GENERAL_CONFIG_I2C_INDEX                  \
	VL53LX_GPH_CONFIG__STREAM_COUNT_UPDATE_VALUE
#define VL53LX_TIMING_CONFIG_I2C_INDEX                  \
	VL53LX_MM_CONFIG__TIMEOUT_MACROP_A_HI
#define VL53LX_DYNAMIC_CONFIG_I2C_INDEX                 \
	VL53LX_SYSTEM__GROUPED_PARAMETER_HOLD_0
#define VL53LX_SYSTEM_CONTROL_I2C_INDEX                 \
	VL53LX_POWER_MANAGEMENT__GO1_POWER_FORCE
#define VL53LX_SYSTEM_RESULTS_I2C_INDEX                 \
	VL53LX_RESULT__INTERRUPT_STATUS
#define VL53LX_CORE_RESULTS_I2C_INDEX                   \
	VL53LX_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0
#define VL53LX_DEBUG_RESULTS_I2C_INDEX                  \
	VL53LX_PHASECAL_RESULT__REFERENCE_PHASE
#define VL53LX_NVM_COPY_DATA_I2C_INDEX                 \
	VL53LX_IDENTIFICATION__MODEL_ID
#define VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_INDEX    \
	VL53LX_PREV_SHADOW_RESULT__INTERRUPT_STATUS
#define VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_INDEX      \
	VL53LX_PREV_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0
#define VL53LX_PATCH_DEBUG_I2C_INDEX                   \
	VL53LX_RESULT__DEBUG_STATUS
#define VL53LX_GPH_GENERAL_CONFIG_I2C_INDEX            \
	VL53LX_GPH__SYSTEM__THRESH_RATE_HIGH
#define VL53LX_GPH_STATIC_CONFIG_I2C_INDEX             \
	VL53LX_GPH__DSS_CONFIG__ROI_MODE_CONTROL
#define VL53LX_GPH_TIMING_CONFIG_I2C_INDEX             \
	VL53LX_GPH__MM_CONFIG__TIMEOUT_MACROP_A_HI
#define VL53LX_FW_INTERNAL_I2C_INDEX                   \
	VL53LX_FIRMWARE__INTERNAL_STREAM_COUNT_DIV
#define VL53LX_PATCH_RESULTS_I2C_INDEX                 \
	VL53LX_DSS_CALC__ROI_CTRL
#define VL53LX_SHADOW_SYSTEM_RESULTS_I2C_INDEX         \
	VL53LX_SHADOW_PHASECAL_RESULT__VCSEL_START
#define VL53LX_SHADOW_CORE_RESULTS_I2C_INDEX           \
	VL53LX_SHADOW_RESULT_CORE__AMBIENT_WINDOW_EVENTS_SD0

#define VL53LX_STATIC_NVM_MANAGED_I2C_SIZE_BYTES           11
#define VL53LX_CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES         23
#define VL53LX_STATIC_CONFIG_I2C_SIZE_BYTES                32
#define VL53LX_GENERAL_CONFIG_I2C_SIZE_BYTES               22
#define VL53LX_TIMING_CONFIG_I2C_SIZE_BYTES                23
#define VL53LX_DYNAMIC_CONFIG_I2C_SIZE_BYTES               18
#define VL53LX_SYSTEM_CONTROL_I2C_SIZE_BYTES                5
#define VL53LX_SYSTEM_RESULTS_I2C_SIZE_BYTES               44
#define VL53LX_CORE_RESULTS_I2C_SIZE_BYTES                 33
#define VL53LX_DEBUG_RESULTS_I2C_SIZE_BYTES                56
#define VL53LX_NVM_COPY_DATA_I2C_SIZE_BYTES                49
#define VL53LX_PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES   44
#define VL53LX_PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES     33
#define VL53LX_PATCH_DEBUG_I2C_SIZE_BYTES                   2
#define VL53LX_GPH_GENERAL_CONFIG_I2C_SIZE_BYTES            5
#define VL53LX_GPH_STATIC_CONFIG_I2C_SIZE_BYTES             6
#define VL53LX_GPH_TIMING_CONFIG_I2C_SIZE_BYTES            16
#define VL53LX_FW_INTERNAL_I2C_SIZE_BYTES                   2
#define VL53LX_PATCH_RESULTS_I2C_SIZE_BYTES                90
#define VL53LX_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES        82
#define VL53LX_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES          33




typedef struct {
	uint8_t   i2c_slave__device_address;

	uint8_t   ana_config__vhv_ref_sel_vddpix;

	uint8_t   ana_config__vhv_ref_sel_vquench;

	uint8_t   ana_config__reg_avdd1v2_sel;

	uint8_t   ana_config__fast_osc__trim;

	uint16_t  osc_measured__fast_osc__frequency;

	uint8_t   vhv_config__timeout_macrop_loop_bound;

	uint8_t   vhv_config__count_thresh;

	uint8_t   vhv_config__offset;

	uint8_t   vhv_config__init;

} VL53LX_static_nvm_managed_t;




typedef struct {
	uint8_t   global_config__spad_enables_ref_0;

	uint8_t   global_config__spad_enables_ref_1;

	uint8_t   global_config__spad_enables_ref_2;

	uint8_t   global_config__spad_enables_ref_3;

	uint8_t   global_config__spad_enables_ref_4;

	uint8_t   global_config__spad_enables_ref_5;

	uint8_t   global_config__ref_en_start_select;

	uint8_t   ref_spad_man__num_requested_ref_spads;

	uint8_t   ref_spad_man__ref_location;

	uint16_t  algo__crosstalk_compensation_plane_offset_kcps;

	int16_t   algo__crosstalk_compensation_x_plane_gradient_kcps;

	int16_t   algo__crosstalk_compensation_y_plane_gradient_kcps;

	uint16_t  ref_spad_char__total_rate_target_mcps;

	int16_t   algo__part_to_part_range_offset_mm;

	int16_t   mm_config__inner_offset_mm;

	int16_t   mm_config__outer_offset_mm;

} VL53LX_customer_nvm_managed_t;




typedef struct {
	uint16_t  dss_config__target_total_rate_mcps;

	uint8_t   debug__ctrl;

	uint8_t   test_mode__ctrl;

	uint8_t   clk_gating__ctrl;

	uint8_t   nvm_bist__ctrl;

	uint8_t   nvm_bist__num_nvm_words;

	uint8_t   nvm_bist__start_address;

	uint8_t   host_if__status;

	uint8_t   pad_i2c_hv__config;

	uint8_t   pad_i2c_hv__extsup_config;

	uint8_t   gpio_hv_pad__ctrl;

	uint8_t   gpio_hv_mux__ctrl;

	uint8_t   gpio__tio_hv_status;

	uint8_t   gpio__fio_hv_status;

	uint8_t   ana_config__spad_sel_pswidth;

	uint8_t   ana_config__vcsel_pulse_width_offset;

	uint8_t   ana_config__fast_osc__config_ctrl;

	uint8_t   sigma_estimator__effective_pulse_width_ns;

	uint8_t   sigma_estimator__effective_ambient_width_ns;

	uint8_t   sigma_estimator__sigma_ref_mm;

	uint8_t   algo__crosstalk_compensation_valid_height_mm;

	uint8_t   spare_host_config__static_config_spare_0;

	uint8_t   spare_host_config__static_config_spare_1;

	uint16_t  algo__range_ignore_threshold_mcps;

	uint8_t   algo__range_ignore_valid_height_mm;

	uint8_t   algo__range_min_clip;

	uint8_t   algo__consistency_check__tolerance;

	uint8_t   spare_host_config__static_config_spare_2;

	uint8_t   sd_config__reset_stages_msb;

	uint8_t   sd_config__reset_stages_lsb;

} VL53LX_static_config_t;




typedef struct {
	uint8_t   gph_config__stream_count_update_value;

	uint8_t   global_config__stream_divider;

	uint8_t   system__interrupt_config_gpio;

	uint8_t   cal_config__vcsel_start;

	uint16_t  cal_config__repeat_rate;

	uint8_t   global_config__vcsel_width;

	uint8_t   phasecal_config__timeout_macrop;

	uint8_t   phasecal_config__target;

	uint8_t   phasecal_config__override;

	uint8_t   dss_config__roi_mode_control;

	uint16_t  system__thresh_rate_high;

	uint16_t  system__thresh_rate_low;

	uint16_t  dss_config__manual_effective_spads_select;

	uint8_t   dss_config__manual_block_select;

	uint8_t   dss_config__aperture_attenuation;

	uint8_t   dss_config__max_spads_limit;

	uint8_t   dss_config__min_spads_limit;

} VL53LX_general_config_t;




typedef struct {
	uint8_t   mm_config__timeout_macrop_a_hi;

	uint8_t   mm_config__timeout_macrop_a_lo;

	uint8_t   mm_config__timeout_macrop_b_hi;

	uint8_t   mm_config__timeout_macrop_b_lo;

	uint8_t   range_config__timeout_macrop_a_hi;

	uint8_t   range_config__timeout_macrop_a_lo;

	uint8_t   range_config__vcsel_period_a;

	uint8_t   range_config__timeout_macrop_b_hi;

	uint8_t   range_config__timeout_macrop_b_lo;

	uint8_t   range_config__vcsel_period_b;

	uint16_t  range_config__sigma_thresh;

	uint16_t  range_config__min_count_rate_rtn_limit_mcps;

	uint8_t   range_config__valid_phase_low;

	uint8_t   range_config__valid_phase_high;

	uint32_t  system__intermeasurement_period;

	uint8_t   system__fractional_enable;

} VL53LX_timing_config_t;




typedef struct {
	uint8_t   system__grouped_parameter_hold_0;

	uint16_t  system__thresh_high;

	uint16_t  system__thresh_low;

	uint8_t   system__enable_xtalk_per_quadrant;

	uint8_t   system__seed_config;

	uint8_t   sd_config__woi_sd0;

	uint8_t   sd_config__woi_sd1;

	uint8_t   sd_config__initial_phase_sd0;

	uint8_t   sd_config__initial_phase_sd1;

	uint8_t   system__grouped_parameter_hold_1;

	uint8_t   sd_config__first_order_select;

	uint8_t   sd_config__quantifier;

	uint8_t   roi_config__user_roi_centre_spad;

	uint8_t   roi_config__user_roi_requested_global_xy_size;

	uint8_t   system__sequence_config;

	uint8_t   system__grouped_parameter_hold;

} VL53LX_dynamic_config_t;




typedef struct {
	uint8_t   power_management__go1_power_force;

	uint8_t   system__stream_count_ctrl;

	uint8_t   firmware__enable;

	uint8_t   system__interrupt_clear;

	uint8_t   system__mode_start;

} VL53LX_system_control_t;




typedef struct {
	uint8_t   result__interrupt_status;

	uint8_t   result__range_status;

	uint8_t   result__report_status;

	uint8_t   result__stream_count;

	uint16_t  result__dss_actual_effective_spads_sd0;

	uint16_t  result__peak_signal_count_rate_mcps_sd0;

	uint16_t  result__ambient_count_rate_mcps_sd0;

	uint16_t  result__sigma_sd0;

	uint16_t  result__phase_sd0;

	uint16_t  result__final_crosstalk_corrected_range_mm_sd0;

	uint16_t  result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;

	uint16_t  result__mm_inner_actual_effective_spads_sd0;

	uint16_t  result__mm_outer_actual_effective_spads_sd0;

	uint16_t  result__avg_signal_count_rate_mcps_sd0;

	uint16_t  result__dss_actual_effective_spads_sd1;

	uint16_t  result__peak_signal_count_rate_mcps_sd1;

	uint16_t  result__ambient_count_rate_mcps_sd1;

	uint16_t  result__sigma_sd1;

	uint16_t  result__phase_sd1;

	uint16_t  result__final_crosstalk_corrected_range_mm_sd1;

	uint16_t  result__spare_0_sd1;

	uint16_t  result__spare_1_sd1;

	uint16_t  result__spare_2_sd1;

	uint8_t   result__spare_3_sd1;

	uint8_t   result__thresh_info;

} VL53LX_system_results_t;




typedef struct {
	uint32_t  result_core__ambient_window_events_sd0;

	uint32_t  result_core__ranging_total_events_sd0;

	int32_t   result_core__signal_total_events_sd0;

	uint32_t  result_core__total_periods_elapsed_sd0;

	uint32_t  result_core__ambient_window_events_sd1;

	uint32_t  result_core__ranging_total_events_sd1;

	int32_t   result_core__signal_total_events_sd1;

	uint32_t  result_core__total_periods_elapsed_sd1;

	uint8_t   result_core__spare_0;

} VL53LX_core_results_t;




typedef struct {
	uint16_t  phasecal_result__reference_phase;

	uint8_t   phasecal_result__vcsel_start;

	uint8_t   ref_spad_char_result__num_actual_ref_spads;

	uint8_t   ref_spad_char_result__ref_location;

	uint8_t   vhv_result__coldboot_status;

	uint8_t   vhv_result__search_result;

	uint8_t   vhv_result__latest_setting;

	uint16_t  result__osc_calibrate_val;

	uint8_t   ana_config__powerdown_go1;

	uint8_t   ana_config__ref_bg_ctrl;

	uint8_t   ana_config__regdvdd1v2_ctrl;

	uint8_t   ana_config__osc_slow_ctrl;

	uint8_t   test_mode__status;

	uint8_t   firmware__system_status;

	uint8_t   firmware__mode_status;

	uint8_t   firmware__secondary_mode_status;

	uint16_t  firmware__cal_repeat_rate_counter;

	uint16_t  gph__system__thresh_high;

	uint16_t  gph__system__thresh_low;

	uint8_t   gph__system__enable_xtalk_per_quadrant;

	uint8_t   gph__spare_0;

	uint8_t   gph__sd_config__woi_sd0;

	uint8_t   gph__sd_config__woi_sd1;

	uint8_t   gph__sd_config__initial_phase_sd0;

	uint8_t   gph__sd_config__initial_phase_sd1;

	uint8_t   gph__sd_config__first_order_select;

	uint8_t   gph__sd_config__quantifier;

	uint8_t   gph__roi_config__user_roi_centre_spad;

	uint8_t   gph__roi_config__user_roi_requested_global_xy_size;

	uint8_t   gph__system__sequence_config;

	uint8_t   gph__gph_id;

	uint8_t   system__interrupt_set;

	uint8_t   interrupt_manager__enables;

	uint8_t   interrupt_manager__clear;

	uint8_t   interrupt_manager__status;

	uint8_t   mcu_to_host_bank__wr_access_en;

	uint8_t   power_management__go1_reset_status;

	uint8_t   pad_startup_mode__value_ro;

	uint8_t   pad_startup_mode__value_ctrl;

	uint32_t  pll_period_us;

	uint32_t  interrupt_scheduler__data_out;

	uint8_t   nvm_bist__complete;

	uint8_t   nvm_bist__status;

} VL53LX_debug_results_t;




typedef struct {
	uint8_t   identification__model_id;

	uint8_t   identification__module_type;

	uint8_t   identification__revision_id;

	uint16_t  identification__module_id;

	uint8_t   ana_config__fast_osc__trim_max;

	uint8_t   ana_config__fast_osc__freq_set;

	uint8_t   ana_config__vcsel_trim;

	uint8_t   ana_config__vcsel_selion;

	uint8_t   ana_config__vcsel_selion_max;

	uint8_t   protected_laser_safety__lock_bit;

	uint8_t   laser_safety__key;

	uint8_t   laser_safety__key_ro;

	uint8_t   laser_safety__clip;

	uint8_t   laser_safety__mult;

	uint8_t   global_config__spad_enables_rtn_0;

	uint8_t   global_config__spad_enables_rtn_1;

	uint8_t   global_config__spad_enables_rtn_2;

	uint8_t   global_config__spad_enables_rtn_3;

	uint8_t   global_config__spad_enables_rtn_4;

	uint8_t   global_config__spad_enables_rtn_5;

	uint8_t   global_config__spad_enables_rtn_6;

	uint8_t   global_config__spad_enables_rtn_7;

	uint8_t   global_config__spad_enables_rtn_8;

	uint8_t   global_config__spad_enables_rtn_9;

	uint8_t   global_config__spad_enables_rtn_10;

	uint8_t   global_config__spad_enables_rtn_11;

	uint8_t   global_config__spad_enables_rtn_12;

	uint8_t   global_config__spad_enables_rtn_13;

	uint8_t   global_config__spad_enables_rtn_14;

	uint8_t   global_config__spad_enables_rtn_15;

	uint8_t   global_config__spad_enables_rtn_16;

	uint8_t   global_config__spad_enables_rtn_17;

	uint8_t   global_config__spad_enables_rtn_18;

	uint8_t   global_config__spad_enables_rtn_19;

	uint8_t   global_config__spad_enables_rtn_20;

	uint8_t   global_config__spad_enables_rtn_21;

	uint8_t   global_config__spad_enables_rtn_22;

	uint8_t   global_config__spad_enables_rtn_23;

	uint8_t   global_config__spad_enables_rtn_24;

	uint8_t   global_config__spad_enables_rtn_25;

	uint8_t   global_config__spad_enables_rtn_26;

	uint8_t   global_config__spad_enables_rtn_27;

	uint8_t   global_config__spad_enables_rtn_28;

	uint8_t   global_config__spad_enables_rtn_29;

	uint8_t   global_config__spad_enables_rtn_30;

	uint8_t   global_config__spad_enables_rtn_31;

	uint8_t   roi_config__mode_roi_centre_spad;

	uint8_t   roi_config__mode_roi_xy_size;

} VL53LX_nvm_copy_data_t;




typedef struct {
	uint8_t   prev_shadow_result__interrupt_status;

	uint8_t   prev_shadow_result__range_status;

	uint8_t   prev_shadow_result__report_status;

	uint8_t   prev_shadow_result__stream_count;

	uint16_t  prev_shadow_result__dss_actual_effective_spads_sd0;

	uint16_t  prev_shadow_result__peak_signal_count_rate_mcps_sd0;

	uint16_t  prev_shadow_result__ambient_count_rate_mcps_sd0;

	uint16_t  prev_shadow_result__sigma_sd0;

	uint16_t  prev_shadow_result__phase_sd0;

	uint16_t  prev_shadow_result__final_crosstalk_corrected_range_mm_sd0;

	uint16_t
	psr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;

	uint16_t  prev_shadow_result__mm_inner_actual_effective_spads_sd0;

	uint16_t  prev_shadow_result__mm_outer_actual_effective_spads_sd0;

	uint16_t  prev_shadow_result__avg_signal_count_rate_mcps_sd0;

	uint16_t  prev_shadow_result__dss_actual_effective_spads_sd1;

	uint16_t  prev_shadow_result__peak_signal_count_rate_mcps_sd1;

	uint16_t  prev_shadow_result__ambient_count_rate_mcps_sd1;

	uint16_t  prev_shadow_result__sigma_sd1;

	uint16_t  prev_shadow_result__phase_sd1;

	uint16_t  prev_shadow_result__final_crosstalk_corrected_range_mm_sd1;

	uint16_t  prev_shadow_result__spare_0_sd1;

	uint16_t  prev_shadow_result__spare_1_sd1;

	uint16_t  prev_shadow_result__spare_2_sd1;

	uint16_t  prev_shadow_result__spare_3_sd1;

} VL53LX_prev_shadow_system_results_t;




typedef struct {
	uint32_t  prev_shadow_result_core__ambient_window_events_sd0;

	uint32_t  prev_shadow_result_core__ranging_total_events_sd0;

	int32_t   prev_shadow_result_core__signal_total_events_sd0;

	uint32_t  prev_shadow_result_core__total_periods_elapsed_sd0;

	uint32_t  prev_shadow_result_core__ambient_window_events_sd1;

	uint32_t  prev_shadow_result_core__ranging_total_events_sd1;

	int32_t   prev_shadow_result_core__signal_total_events_sd1;

	uint32_t  prev_shadow_result_core__total_periods_elapsed_sd1;

	uint8_t   prev_shadow_result_core__spare_0;

} VL53LX_prev_shadow_core_results_t;




typedef struct {
	uint8_t   result__debug_status;

	uint8_t   result__debug_stage;

} VL53LX_patch_debug_t;




typedef struct {
	uint16_t  gph__system__thresh_rate_high;

	uint16_t  gph__system__thresh_rate_low;

	uint8_t   gph__system__interrupt_config_gpio;

} VL53LX_gph_general_config_t;




typedef struct {
	uint8_t   gph__dss_config__roi_mode_control;

	uint16_t  gph__dss_config__manual_effective_spads_select;

	uint8_t   gph__dss_config__manual_block_select;

	uint8_t   gph__dss_config__max_spads_limit;

	uint8_t   gph__dss_config__min_spads_limit;

} VL53LX_gph_static_config_t;




typedef struct {
	uint8_t   gph__mm_config__timeout_macrop_a_hi;

	uint8_t   gph__mm_config__timeout_macrop_a_lo;

	uint8_t   gph__mm_config__timeout_macrop_b_hi;

	uint8_t   gph__mm_config__timeout_macrop_b_lo;

	uint8_t   gph__range_config__timeout_macrop_a_hi;

	uint8_t   gph__range_config__timeout_macrop_a_lo;

	uint8_t   gph__range_config__vcsel_period_a;

	uint8_t   gph__range_config__vcsel_period_b;

	uint8_t   gph__range_config__timeout_macrop_b_hi;

	uint8_t   gph__range_config__timeout_macrop_b_lo;

	uint16_t  gph__range_config__sigma_thresh;

	uint16_t  gph__range_config__min_count_rate_rtn_limit_mcps;

	uint8_t   gph__range_config__valid_phase_low;

	uint8_t   gph__range_config__valid_phase_high;

} VL53LX_gph_timing_config_t;




typedef struct {
	uint8_t   firmware__internal_stream_count_div;

	uint8_t   firmware__internal_stream_counter_val;

} VL53LX_fw_internal_t;




typedef struct {
	uint8_t   dss_calc__roi_ctrl;

	uint8_t   dss_calc__spare_1;

	uint8_t   dss_calc__spare_2;

	uint8_t   dss_calc__spare_3;

	uint8_t   dss_calc__spare_4;

	uint8_t   dss_calc__spare_5;

	uint8_t   dss_calc__spare_6;

	uint8_t   dss_calc__spare_7;

	uint8_t   dss_calc__user_roi_spad_en_0;

	uint8_t   dss_calc__user_roi_spad_en_1;

	uint8_t   dss_calc__user_roi_spad_en_2;

	uint8_t   dss_calc__user_roi_spad_en_3;

	uint8_t   dss_calc__user_roi_spad_en_4;

	uint8_t   dss_calc__user_roi_spad_en_5;

	uint8_t   dss_calc__user_roi_spad_en_6;

	uint8_t   dss_calc__user_roi_spad_en_7;

	uint8_t   dss_calc__user_roi_spad_en_8;

	uint8_t   dss_calc__user_roi_spad_en_9;

	uint8_t   dss_calc__user_roi_spad_en_10;

	uint8_t   dss_calc__user_roi_spad_en_11;

	uint8_t   dss_calc__user_roi_spad_en_12;

	uint8_t   dss_calc__user_roi_spad_en_13;

	uint8_t   dss_calc__user_roi_spad_en_14;

	uint8_t   dss_calc__user_roi_spad_en_15;

	uint8_t   dss_calc__user_roi_spad_en_16;

	uint8_t   dss_calc__user_roi_spad_en_17;

	uint8_t   dss_calc__user_roi_spad_en_18;

	uint8_t   dss_calc__user_roi_spad_en_19;

	uint8_t   dss_calc__user_roi_spad_en_20;

	uint8_t   dss_calc__user_roi_spad_en_21;

	uint8_t   dss_calc__user_roi_spad_en_22;

	uint8_t   dss_calc__user_roi_spad_en_23;

	uint8_t   dss_calc__user_roi_spad_en_24;

	uint8_t   dss_calc__user_roi_spad_en_25;

	uint8_t   dss_calc__user_roi_spad_en_26;

	uint8_t   dss_calc__user_roi_spad_en_27;

	uint8_t   dss_calc__user_roi_spad_en_28;

	uint8_t   dss_calc__user_roi_spad_en_29;

	uint8_t   dss_calc__user_roi_spad_en_30;

	uint8_t   dss_calc__user_roi_spad_en_31;

	uint8_t   dss_calc__user_roi_0;

	uint8_t   dss_calc__user_roi_1;

	uint8_t   dss_calc__mode_roi_0;

	uint8_t   dss_calc__mode_roi_1;

	uint8_t   sigma_estimator_calc__spare_0;

	uint16_t  vhv_result__peak_signal_rate_mcps;

	uint32_t  vhv_result__signal_total_events_ref;

	uint16_t  phasecal_result__phase_output_ref;

	uint16_t  dss_result__total_rate_per_spad;

	uint8_t   dss_result__enabled_blocks;

	uint16_t  dss_result__num_requested_spads;

	uint16_t  mm_result__inner_intersection_rate;

	uint16_t  mm_result__outer_complement_rate;

	uint16_t  mm_result__total_offset;

	uint32_t  xtalk_calc__xtalk_for_enabled_spads;

	uint32_t  xtalk_result__avg_xtalk_user_roi_kcps;

	uint32_t  xtalk_result__avg_xtalk_mm_inner_roi_kcps;

	uint32_t  xtalk_result__avg_xtalk_mm_outer_roi_kcps;

	uint32_t  range_result__accum_phase;

	uint16_t  range_result__offset_corrected_range;

} VL53LX_patch_results_t;




typedef struct {
	uint8_t   shadow_phasecal_result__vcsel_start;

	uint8_t   shadow_result__interrupt_status;

	uint8_t   shadow_result__range_status;

	uint8_t   shadow_result__report_status;

	uint8_t   shadow_result__stream_count;

	uint16_t  shadow_result__dss_actual_effective_spads_sd0;

	uint16_t  shadow_result__peak_signal_count_rate_mcps_sd0;

	uint16_t  shadow_result__ambient_count_rate_mcps_sd0;

	uint16_t  shadow_result__sigma_sd0;

	uint16_t  shadow_result__phase_sd0;

	uint16_t  shadow_result__final_crosstalk_corrected_range_mm_sd0;

	uint16_t
	shr__peak_signal_count_rate_crosstalk_corrected_mcps_sd0;

	uint16_t  shadow_result__mm_inner_actual_effective_spads_sd0;

	uint16_t  shadow_result__mm_outer_actual_effective_spads_sd0;

	uint16_t  shadow_result__avg_signal_count_rate_mcps_sd0;

	uint16_t  shadow_result__dss_actual_effective_spads_sd1;

	uint16_t  shadow_result__peak_signal_count_rate_mcps_sd1;

	uint16_t  shadow_result__ambient_count_rate_mcps_sd1;

	uint16_t  shadow_result__sigma_sd1;

	uint16_t  shadow_result__phase_sd1;

	uint16_t  shadow_result__final_crosstalk_corrected_range_mm_sd1;

	uint16_t  shadow_result__spare_0_sd1;

	uint16_t  shadow_result__spare_1_sd1;

	uint16_t  shadow_result__spare_2_sd1;

	uint8_t   shadow_result__spare_3_sd1;

	uint8_t   shadow_result__thresh_info;

	uint8_t   shadow_phasecal_result__reference_phase_hi;

	uint8_t   shadow_phasecal_result__reference_phase_lo;

} VL53LX_shadow_system_results_t;




typedef struct {
	uint32_t  shadow_result_core__ambient_window_events_sd0;

	uint32_t  shadow_result_core__ranging_total_events_sd0;

	int32_t   shadow_result_core__signal_total_events_sd0;

	uint32_t  shadow_result_core__total_periods_elapsed_sd0;

	uint32_t  shadow_result_core__ambient_window_events_sd1;

	uint32_t  shadow_result_core__ranging_total_events_sd1;

	int32_t   shadow_result_core__signal_total_events_sd1;

	uint32_t  shadow_result_core__total_periods_elapsed_sd1;

	uint8_t   shadow_result_core__spare_0;

} VL53LX_shadow_core_results_t;


#endif


