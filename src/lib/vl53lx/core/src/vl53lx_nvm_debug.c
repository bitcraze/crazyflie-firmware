
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include "vl53lx_platform.h"
#include <vl53lx_platform_log.h>
#include "vl53lx_ll_def.h"
#include "vl53lx_register_map.h"
#include "vl53lx_api_debug.h"
#include "vl53lx_nvm_structs.h"
#include "vl53lx_nvm_debug.h"

#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(VL53LX_TRACE_MODULE_NVM, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(VL53LX_TRACE_MODULE_NVM, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(VL53LX_TRACE_MODULE_NVM,\
		status, fmt, ##__VA_ARGS__)

#define trace_print(level, ...) \
	_LOG_TRACE_PRINT(trace_flags, \
	level, VL53LX_TRACE_FUNCTION_NONE, ##__VA_ARGS__)

#ifdef VL53LX_LOG_ENABLE

void VL53LX_print_nvm_raw_data(
	uint8_t                       *pnvm_raw_data,
	uint32_t                       trace_flags)
{


	int i = 0;

	LOG_FUNCTION_START("");

	for (i = 0 ; i < VL53LX_NVM_SIZE_IN_BYTES ; i++) {
		if (i % 4 == 0)
			trace_print(
				VL53LX_TRACE_LEVEL_INFO,
				"\n    NVM Addr 0x%02X : 0x",
				i/4);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%02X",
			*pnvm_raw_data++);
	}

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"\n");

	LOG_FUNCTION_END(0);
}


void VL53LX_print_decoded_nvm_data(
	VL53LX_decoded_nvm_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags)
{
	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t i = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__identification_model_id",
		pdata->nvm__identification_model_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__identification_module_type",
		pdata->nvm__identification_module_type);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__identification_revision_id",
		pdata->nvm__identification_revision_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__identification_module_id",
		pdata->nvm__identification_module_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__i2c_valid",
		pdata->nvm__i2c_valid);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__i2c_device_address_ews",
		pdata->nvm__i2c_device_address_ews);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->nvm__ews__fast_osc_frequency,
		12,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__ews__fast_osc_frequency",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__fast_osc_trim_max",
		pdata->nvm__ews__fast_osc_trim_max);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__fast_osc_freq_set",
		pdata->nvm__ews__fast_osc_freq_set);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__slow_osc_calibration",
		pdata->nvm__ews__slow_osc_calibration);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->nvm__fmt__fast_osc_frequency,
		12,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__fmt__fast_osc_frequency",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__fast_osc_trim_max",
		pdata->nvm__fmt__fast_osc_trim_max);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__fast_osc_freq_set",
		pdata->nvm__fmt__fast_osc_freq_set);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__slow_osc_calibration",
		pdata->nvm__fmt__slow_osc_calibration);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_config_unlock",
		pdata->nvm__vhv_config_unlock);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ref_selvddpix",
		pdata->nvm__ref_selvddpix);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ref_selvquench",
		pdata->nvm__ref_selvquench);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__regavdd1v2_sel",
		pdata->nvm__regavdd1v2_sel);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__regdvdd1v2_sel",
		pdata->nvm__regdvdd1v2_sel);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_timeout__macrop",
		pdata->nvm__vhv_timeout__macrop);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_loop_bound",
		pdata->nvm__vhv_loop_bound);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_count_threshold",
		pdata->nvm__vhv_count_threshold);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_offset",
		pdata->nvm__vhv_offset);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_init_enable",
		pdata->nvm__vhv_init_enable);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__vhv_init_value",
		pdata->nvm__vhv_init_value);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_vcsel_trim_ll",
		pdata->nvm__laser_safety_vcsel_trim_ll);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_vcsel_selion_ll",
		pdata->nvm__laser_safety_vcsel_selion_ll);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_vcsel_selion_max_ll",
		pdata->nvm__laser_safety_vcsel_selion_max_ll);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_mult_ll",
		pdata->nvm__laser_safety_mult_ll);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_clip_ll",
		pdata->nvm__laser_safety_clip_ll);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_vcsel_trim_ld",
		pdata->nvm__laser_safety_vcsel_trim_ld);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_vcsel_selion_ld",
		pdata->nvm__laser_safety_vcsel_selion_ld);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_vcsel_selion_max_ld",
		pdata->nvm__laser_safety_vcsel_selion_max_ld);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_mult_ld",
		pdata->nvm__laser_safety_mult_ld);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_clip_ld",
		pdata->nvm__laser_safety_clip_ld);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_lock_byte",
		pdata->nvm__laser_safety_lock_byte);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__laser_safety_unlock_byte",
		pdata->nvm__laser_safety_unlock_byte);



	for (i = 0 ; i < VL53LX_RTN_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__ews__spad_enables_rtn[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__ews__spad_enables_rtn[i]);
	}

	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__ews__spad_enables_ref__loc1[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__ews__spad_enables_ref__loc1[i]);
	}

	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__ews__spad_enables_ref__loc2[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__ews__spad_enables_ref__loc2[i]);
	}

	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__ews__spad_enables_ref__loc3[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__ews__spad_enables_ref__loc3[i]);
	}




	for (i = 0 ; i < VL53LX_RTN_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__fmt__spad_enables_rtn[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__fmt__spad_enables_rtn[i]);
	}

	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__fmt__spad_enables_ref__loc1[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__fmt__spad_enables_ref__loc1[i]);
	}

	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__fmt__spad_enables_ref__loc2[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__fmt__spad_enables_ref__loc2[i]);
	}

	for (i = 0 ; i < VL53LX_REF_SPAD_BUFFER_SIZE ; i++) {
		sprintf(
			ppre_text,
			"%snvm__fmt__spad_enables_ref__loc3[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->nvm__fmt__spad_enables_ref__loc3[i]);
	}

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__roi_config__mode_roi_centre_spad",
		pdata->nvm__fmt__roi_config__mode_roi_centre_spad);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__roi_config__mode_roi_x_size",
		pdata->nvm__fmt__roi_config__mode_roi_x_size);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__roi_config__mode_roi_y_size",
		pdata->nvm__fmt__roi_config__mode_roi_y_size);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__ref_spad_apply__num_requested_ref_spad",
		pdata->nvm__fmt__ref_spad_apply__num_requested_ref_spad);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__ref_spad_man__ref_location",
		pdata->nvm__fmt__ref_spad_man__ref_location);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"nvm__fmt__mm_config__inner_offset_mm",
		pdata->nvm__fmt__mm_config__inner_offset_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"nvm__fmt__mm_config__outer_offset_mm",
		pdata->nvm__fmt__mm_config__outer_offset_mm);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->nvm__fmt__algo_part_to_part_range_offset_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__fmt__algo_part_to_part_range_offset_mm",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
	(int32_t)(
	pdata->nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps),
	9,
	VL53LX_MAX_STRING_LENGTH,
	fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__fmt__algo__crosstalk_compensation_plane_offset_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
	(int32_t)(
	pdata->nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps),
	11,
	VL53LX_MAX_STRING_LENGTH,
	fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__fmt__algo__crosstalk_compensation_x_plane_gradient_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
	(int32_t)(
	pdata->nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps),
	11,
	VL53LX_MAX_STRING_LENGTH,
	fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__fmt__algo__crosstalk_compensation_y_plane_gradient_kcps",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__spare__host_config__nvm_config_spare_0",
		pdata->nvm__fmt__spare__host_config__nvm_config_spare_0);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__spare__host_config__nvm_config_spare_1",
		pdata->nvm__fmt__spare__host_config__nvm_config_spare_1);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__customer_space_programmed",
		pdata->nvm__customer_space_programmed);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__cust__i2c_device_address",
		pdata->nvm__cust__i2c_device_address);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__cust__ref_spad_apply__num_requested_ref_spad",
		pdata->nvm__cust__ref_spad_apply__num_requested_ref_spad);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__cust__ref_spad_man__ref_location",
		pdata->nvm__cust__ref_spad_man__ref_location);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"nvm__cust__mm_config__inner_offset_mm",
		pdata->nvm__cust__mm_config__inner_offset_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"nvm__cust__mm_config__outer_offset_mm",
		pdata->nvm__cust__mm_config__outer_offset_mm);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->nvm__cust__algo_part_to_part_range_offset_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__cust__algo_part_to_part_range_offset_mm",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
	(uint32_t)(
	pdata->nvm__cust__algo__crosstalk_compensation_plane_offset_kcps),
	9,
	VL53LX_MAX_STRING_LENGTH,
	fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__cust__algo__crosstalk_compensation_plane_offset_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
	(int32_t)(
	pdata->nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps),
	11,
	VL53LX_MAX_STRING_LENGTH,
	fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__cust__algo__crosstalk_compensation_x_plane_gradient_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
	(int32_t)(
	pdata->nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps),
	11,
	VL53LX_MAX_STRING_LENGTH,
	fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"nvm__cust__algo__crosstalk_compensation_y_plane_gradient_kcps",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__cust__spare__host_config__nvm_config_spare_0",
		pdata->nvm__cust__spare__host_config__nvm_config_spare_0);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__cust__spare__host_config__nvm_config_spare_1",
		pdata->nvm__cust__spare__host_config__nvm_config_spare_1);



	sprintf(
		ppre_text,
		"%sfmt_optical_centre.", pprefix);

	VL53LX_print_optical_centre(
		&(pdata->fmt_optical_centre),
		ppre_text,
		VL53LX_TRACE_MODULE_NVM_DATA);



	sprintf(
		ppre_text,
		"%sfmt_peak_rate_map.", pprefix);

	VL53LX_print_cal_peak_rate_map(
		&(pdata->fmt_peak_rate_map),
		ppre_text,
		VL53LX_TRACE_MODULE_NVM_DATA);



	sprintf(
		ppre_text,
		"%sfmt_add_offset_data.",
		pprefix);

	VL53LX_print_additional_offset_cal_data(
		&(pdata->fmt_add_offset_data),
		ppre_text,
		VL53LX_TRACE_MODULE_NVM_DATA);



	for (i = 0 ; i < VL53LX_NVM_MAX_FMT_RANGE_DATA ; i++) {
		sprintf(
			ppre_text,
			"%sfmt_range_data[%u].",
			pprefix, i);

		VL53LX_print_decoded_nvm_fmt_range_data(
			&(pdata->fmt_range_data[i]),
			ppre_text,
			trace_flags);
	}

	sprintf(
		ppre_text,
		"%sfmt_info.",
		pprefix);

	VL53LX_print_decoded_nvm_fmt_info(
		&(pdata->fmt_info),
		ppre_text,
		trace_flags);

	sprintf(
		ppre_text,
		"%sews_info.",
		pprefix);

	VL53LX_print_decoded_nvm_ews_info(
		&(pdata->ews_info),
		ppre_text,
		trace_flags);
}


void VL53LX_print_decoded_nvm_fmt_range_data(
	VL53LX_decoded_nvm_fmt_range_data_t *pdata,
	char                                *pprefix,
	uint32_t                             trace_flags)
{
	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__actual_effective_rtn_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__actual_effective_rtn_spads",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ref_spad_array__num_requested_ref_spads",
		pdata->ref_spad_array__num_requested_ref_spads);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ref_spad_array__ref_location",
		pdata->ref_spad_array__ref_location);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__peak_signal_count_rate_rtn_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__peak_signal_count_rate_rtn_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__ambient_count_rate_rtn_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__ambient_count_rate_rtn_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__peak_signal_count_rate_ref_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"result__peak_signal_count_rate_ref_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__ambient_count_rate_ref_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__ambient_count_rate_ref_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->measured_distance_mm,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"measured_distance_mm",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(uint32_t)pdata->measured_distance_stdev_mm,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"measured_distance_stdev_mm",
		fp_text);
}


void VL53LX_print_decoded_nvm_fmt_info(
	VL53LX_decoded_nvm_fmt_info_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags)
{
	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = \"%s\"\n",
		pprefix,
		"nvm__fmt__fgc",
		pdata->nvm__fmt__fgc);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__test_program_major",
		pdata->nvm__fmt__test_program_major);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__test_program_minor",
		pdata->nvm__fmt__test_program_minor);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__map_major",
		pdata->nvm__fmt__map_major);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__map_minor",
		pdata->nvm__fmt__map_minor);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__year",
		pdata->nvm__fmt__year);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__month",
		pdata->nvm__fmt__month);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__day",
		pdata->nvm__fmt__day);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__module_date_phase",
		pdata->nvm__fmt__module_date_phase);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__time",
		pdata->nvm__fmt__time);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__tester_id",
		pdata->nvm__fmt__tester_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__fmt__site_id",
		pdata->nvm__fmt__site_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__test_program_major",
		pdata->nvm__ews__test_program_major);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__test_program_minor",
		pdata->nvm__ews__test_program_minor);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__probe_card_major",
		pdata->nvm__ews__probe_card_major);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__probe_card_minor",
		pdata->nvm__ews__probe_card_minor);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__tester_id",
		pdata->nvm__ews__tester_id);
}


void VL53LX_print_decoded_nvm_ews_info(
	VL53LX_decoded_nvm_ews_info_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags)
{
	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = \"%s\"\n",
		pprefix,
		"nvm__ews__lot",
		pdata->nvm__ews__lot);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__wafer",
		pdata->nvm__ews__wafer);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__xcoord",
		pdata->nvm__ews__xcoord);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"nvm__ews__ycoord",
		pdata->nvm__ews__ycoord);
}

#endif


