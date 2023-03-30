
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include "vl53lx_ll_def.h"
#include "vl53lx_ll_device.h"
#include "vl53lx_register_structs.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_nvm_structs.h"
#include "vl53lx_nvm.h"
#include "vl53lx_core.h"
#include "vl53lx_api_debug.h"

#ifdef VL53LX_LOG_ENABLE
#include "vl53lx_nvm_debug.h"
#endif

#define LOG_FUNCTION_START(fmt, ...)
#define LOG_FUNCTION_END(status, ...)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

#define trace_print(level, ...) 


VL53LX_Error VL53LX_decode_calibration_data_buffer(
	uint16_t                   buf_size,
	uint8_t                   *pbuffer,
	VL53LX_calibration_data_t *pdata)
{
	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (sizeof(VL53LX_calibration_data_t) > buf_size)
		return VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL;

	memcpy(pdata, pbuffer, sizeof(VL53LX_calibration_data_t));

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_nvm_debug_data(
	VL53LX_DEV                          Dev,
	VL53LX_decoded_nvm_data_t          *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status = VL53LX_read_nvm(Dev, 0, pdata);

#ifdef VL53LX_LOG_ENABLE
	if (status == VL53LX_ERROR_NONE)
		VL53LX_print_decoded_nvm_data(
			pdata,
			"get_nvm_debug_data():pnvm_info.",
			VL53LX_TRACE_MODULE_NVM_DATA);
#endif

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_histogram_debug_data(
	VL53LX_DEV                          Dev,
	VL53LX_histogram_bin_data_t        *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	memcpy(
		pdata,
		&(pdev->hist_data),
		sizeof(VL53LX_histogram_bin_data_t));

	LOG_FUNCTION_END(status);

	return status;
}




VL53LX_Error VL53LX_get_additional_data(
	VL53LX_DEV                       Dev,
	VL53LX_additional_data_t        *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");



	pdata->preset_mode             = pdev->preset_mode;
	pdata->zone_preset             = pdev->zone_preset;
	pdata->measurement_mode        = pdev->measurement_mode;
	pdata->offset_calibration_mode = pdev->offset_calibration_mode;
	pdata->offset_correction_mode  = pdev->offset_correction_mode;
	pdata->dmax_mode               = pdev->dmax_mode;

	pdata->phasecal_config_timeout_us  = pdev->phasecal_config_timeout_us;
	pdata->mm_config_timeout_us        = pdev->mm_config_timeout_us;
	pdata->range_config_timeout_us     = pdev->range_config_timeout_us;
	pdata->inter_measurement_period_ms = pdev->inter_measurement_period_ms;
	pdata->dss_config__target_total_rate_mcps =
			pdev->dss_config__target_total_rate_mcps;



	status =
		VL53LX_get_histogram_debug_data(
			Dev,
			&(pdata->VL53LX_p_006));

	LOG_FUNCTION_END(status);

	return status;
}




VL53LX_Error VL53LX_get_xtalk_debug_data(
	VL53LX_DEV                          Dev,
	VL53LX_xtalk_debug_data_t          *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	memcpy(
		&(pdata->customer),
		&(pdev->customer),
		sizeof(VL53LX_customer_nvm_managed_t));

	memcpy(
		&(pdata->xtalk_cfg),
		&(pdev->xtalk_cfg),
		sizeof(VL53LX_xtalk_config_t));

	memcpy(
		&(pdata->hist_data),
		&(pdev->hist_data),
		sizeof(VL53LX_histogram_bin_data_t));

	memcpy(
		&(pdata->xtalk_shapes),
		&(pdev->xtalk_shapes),
		sizeof(VL53LX_xtalk_histogram_data_t));

	// memcpy(
	// 	&(pdata->xtalk_results),
	// 	&(pdev->xtalk_results),
	// 	sizeof(VL53LX_xtalk_range_results_t));

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_offset_debug_data(
	VL53LX_DEV                          Dev,
	VL53LX_offset_debug_data_t         *pdata)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");


	memcpy(
		&(pdata->customer),
		&(pdev->customer),
		sizeof(VL53LX_customer_nvm_managed_t));

	memcpy(
		&(pdata->fmt_dmax_cal),
		&(pdev->fmt_dmax_cal),
		sizeof(VL53LX_dmax_calibration_data_t));

	memcpy(
		&(pdata->cust_dmax_cal),
		&(pdev->cust_dmax_cal),
		sizeof(VL53LX_dmax_calibration_data_t));

	memcpy(
		&(pdata->add_off_cal_data),
		&(pdev->add_off_cal_data),
		sizeof(VL53LX_additional_offset_cal_data_t));

	memcpy(
		&(pdata->offset_results),
		&(pdev->offset_results),
		sizeof(VL53LX_offset_range_results_t));

	LOG_FUNCTION_END(status);

	return status;
}

#ifdef VL53LX_LOG_ENABLE

void  VL53LX_signed_fixed_point_sprintf(
	int32_t    signed_fp_value,
	uint8_t    frac_bits,
	uint16_t   buf_size,
	char      *pbuffer)
{


	uint32_t  fp_value      = 0;
	uint32_t  unity_fp_value = 0;
	uint32_t  sign_bit       = 0;
	uint32_t  int_part       = 0;
	uint32_t  frac_part      = 0;
	uint32_t  dec_points     = 0;
	uint32_t  dec_scaler     = 0;
	uint32_t  dec_part       = 0;

	uint64_t  tmp_long_int   = 0;

	char  fmt[VL53LX_MAX_STRING_LENGTH];

	SUPPRESS_UNUSED_WARNING(buf_size);



	sign_bit       =  signed_fp_value >> 31;

	if (sign_bit > 0) {
		fp_value = 0x80000000 -
			(0x7FFFFFFF & (uint32_t)signed_fp_value);
	} else
		fp_value = (uint32_t)signed_fp_value;

	int_part       =  fp_value >> frac_bits;
	unity_fp_value =  0x01 << frac_bits;
	frac_part      =  fp_value & (unity_fp_value-1);


	dec_points =   2;
	dec_scaler = 100;

	while (dec_scaler < unity_fp_value) {
		dec_points++;
		dec_scaler *= 10;
	}


	if (sign_bit > 0)
		sprintf(fmt, "-%%u.%%0%uu", dec_points);
	else
		sprintf(fmt,  "%%u.%%0%uu", dec_points);


	tmp_long_int  = (uint64_t)frac_part * (uint64_t)dec_scaler;
	tmp_long_int += (uint64_t)unity_fp_value/2;

	tmp_long_int = do_division_u(tmp_long_int, (uint64_t)unity_fp_value);

	dec_part = (uint32_t)tmp_long_int;


	sprintf(
		pbuffer,
		fmt,
		int_part,
		dec_part);
}


void VL53LX_print_static_nvm_managed(
	VL53LX_static_nvm_managed_t   *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = 0x%02X\n",
		pprefix,
		"i2c_slave__device_address",
		pdata->i2c_slave__device_address);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__vhv_ref_sel_vddpix",
		pdata->ana_config__vhv_ref_sel_vddpix);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__vhv_ref_sel_vquench",
		pdata->ana_config__vhv_ref_sel_vquench);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__reg_avdd1v2_sel",
		pdata->ana_config__reg_avdd1v2_sel);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__fast_osc__trim",
		pdata->ana_config__fast_osc__trim);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->osc_measured__fast_osc__frequency,
		12,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"osc_measured__fast_osc__frequency",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"vhv_config__timeout_macrop_loop_bound",
		pdata->vhv_config__timeout_macrop_loop_bound);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"vhv_config__count_thresh",
		pdata->vhv_config__count_thresh);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"vhv_config__offset",
		pdata->vhv_config__offset);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"vhv_config__init",
		pdata->vhv_config__init);
}


void VL53LX_print_customer_nvm_managed(
	VL53LX_customer_nvm_managed_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	int16_t tmpi16;

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_ref_0",
		pdata->global_config__spad_enables_ref_0);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_ref_1",
		pdata->global_config__spad_enables_ref_1);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_ref_2",
		pdata->global_config__spad_enables_ref_2);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_ref_3",
		pdata->global_config__spad_enables_ref_3);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_ref_4",
		pdata->global_config__spad_enables_ref_4);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_ref_5",
		pdata->global_config__spad_enables_ref_5);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__ref_en_start_select",
		pdata->global_config__ref_en_start_select);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ref_spad_man__num_requested_ref_spads",
		pdata->ref_spad_man__num_requested_ref_spads);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ref_spad_man__ref_location",
		pdata->ref_spad_man__ref_location);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_compensation_plane_offset_kcps,
		9,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_plane_offset_kcps",
		fp_text);

	tmpi16 = pdata->algo__crosstalk_compensation_x_plane_gradient_kcps;
	VL53LX_signed_fixed_point_sprintf(
		(int32_t)tmpi16,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_x_plane_gradient_kcps",
		fp_text);

	tmpi16 = pdata->algo__crosstalk_compensation_y_plane_gradient_kcps;
	VL53LX_signed_fixed_point_sprintf(
		(int32_t)tmpi16,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_y_plane_gradient_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->ref_spad_char__total_rate_target_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"ref_spad_char__total_rate_target_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__part_to_part_range_offset_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__part_to_part_range_offset_mm",
		fp_text);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"mm_config__inner_offset_mm",
		pdata->mm_config__inner_offset_mm);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"mm_config__outer_offset_mm",
		pdata->mm_config__outer_offset_mm);
}


void VL53LX_print_nvm_copy_data(
	VL53LX_nvm_copy_data_t      *pdata,
	char                        *pprefix,
	uint32_t                     trace_flags)
{


	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"identification__model_id",
		pdata->identification__model_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"identification__module_type",
		pdata->identification__module_type);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"identification__revision_id",
		pdata->identification__revision_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"identification__module_id",
		pdata->identification__module_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__fast_osc__trim_max",
		pdata->ana_config__fast_osc__trim_max);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__fast_osc__freq_set",
		pdata->ana_config__fast_osc__freq_set);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__vcsel_trim",
		pdata->ana_config__vcsel_trim);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__vcsel_selion",
		pdata->ana_config__vcsel_selion);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"ana_config__vcsel_selion_max",
		pdata->ana_config__vcsel_selion_max);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"protected_laser_safety__lock_bit",
		pdata->protected_laser_safety__lock_bit);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"laser_safety__key",
		pdata->laser_safety__key);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"laser_safety__key_ro",
		pdata->laser_safety__key_ro);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"laser_safety__clip",
		pdata->laser_safety__clip);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"laser_safety__mult",
		pdata->laser_safety__mult);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_0",
		pdata->global_config__spad_enables_rtn_0);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_1",
		pdata->global_config__spad_enables_rtn_1);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_2",
		pdata->global_config__spad_enables_rtn_2);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_3",
		pdata->global_config__spad_enables_rtn_3);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_4",
		pdata->global_config__spad_enables_rtn_4);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_5",
		pdata->global_config__spad_enables_rtn_5);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_6",
		pdata->global_config__spad_enables_rtn_6);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_7",
		pdata->global_config__spad_enables_rtn_7);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_8",
		pdata->global_config__spad_enables_rtn_8);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_9",
		pdata->global_config__spad_enables_rtn_9);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_10",
		pdata->global_config__spad_enables_rtn_10);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_11",
		pdata->global_config__spad_enables_rtn_11);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_12",
		pdata->global_config__spad_enables_rtn_12);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_13",
		pdata->global_config__spad_enables_rtn_13);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_14",
		pdata->global_config__spad_enables_rtn_14);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_15",
		pdata->global_config__spad_enables_rtn_15);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_16",
		pdata->global_config__spad_enables_rtn_16);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_17",
		pdata->global_config__spad_enables_rtn_17);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_18",
		pdata->global_config__spad_enables_rtn_18);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_19",
		pdata->global_config__spad_enables_rtn_19);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_20",
		pdata->global_config__spad_enables_rtn_20);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_21",
		pdata->global_config__spad_enables_rtn_21);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_22",
		pdata->global_config__spad_enables_rtn_22);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_23",
		pdata->global_config__spad_enables_rtn_23);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_24",
		pdata->global_config__spad_enables_rtn_24);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_25",
		pdata->global_config__spad_enables_rtn_25);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_26",
		pdata->global_config__spad_enables_rtn_26);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_27",
		pdata->global_config__spad_enables_rtn_27);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_28",
		pdata->global_config__spad_enables_rtn_28);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_29",
		pdata->global_config__spad_enables_rtn_29);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_30",
		pdata->global_config__spad_enables_rtn_30);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_config__spad_enables_rtn_31",
		pdata->global_config__spad_enables_rtn_31);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"roi_config__mode_roi_centre_spad",
		pdata->roi_config__mode_roi_centre_spad);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = 0x%02X\n",
		pprefix,
		"roi_config__mode_roi_xy_size",
		pdata->roi_config__mode_roi_xy_size);
}


void VL53LX_print_histogram_bin_data(
	VL53LX_histogram_bin_data_t *pdata,
	char                        *pprefix,
	uint32_t                     trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t  i             = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cfg_device_state",
		pdata->cfg_device_state);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"rd_device_state",
		pdata->rd_device_state);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_019",
		pdata->VL53LX_p_019);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_020",
		pdata->VL53LX_p_020);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_021",
		pdata->VL53LX_p_021);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"number_of_ambient_bins",
		pdata->number_of_ambient_bins);

	for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++) {
		sprintf(ppre_text, "%sbin_seq[%u]", pprefix, i);
		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->bin_seq[i]);
	}

	for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++) {
		sprintf(ppre_text, "%sbin_rep[%u]", pprefix, i);
		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->bin_rep[i]);
	}

	for (i = 0; i < pdata->VL53LX_p_021; i++) {
		sprintf(ppre_text, "%sbin_data[%u]", pprefix, i);
		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %d\n",
			ppre_text,
			pdata->bin_data[i]);
	}

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"result__interrupt_status",
		pdata->result__interrupt_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"result__range_status",
		pdata->result__range_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"result__report_status",
		pdata->result__report_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"result__stream_count",
		pdata->result__stream_count);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__dss_actual_effective_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__dss_actual_effective_spads",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->phasecal_result__reference_phase,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"phasecal_result__reference_phase",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"phasecal_result__vcsel_start",
		pdata->phasecal_result__vcsel_start);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_config__vcsel_start",
		pdata->cal_config__vcsel_start);

	VL53LX_signed_fixed_point_sprintf(
		(uint32_t)pdata->vcsel_width,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"vcsel_width",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_005",
		pdata->VL53LX_p_005);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_015,
		12,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_015",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"total_periods_elapsed",
		pdata->total_periods_elapsed);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"peak_duration_us",
		pdata->peak_duration_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"woi_duration_us",
		pdata->woi_duration_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"min_bin_value",
		pdata->min_bin_value);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_bin_value",
		pdata->max_bin_value);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->zero_distance_phase,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"zero_distance_phase",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"number_of_ambient_samples",
		pdata->number_of_ambient_samples);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"ambient_events_sum",
		pdata->ambient_events_sum);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"VL53LX_p_028",
		pdata->VL53LX_p_028);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = 0x%02X\n",
		pprefix,
		"roi_config__user_roi_centre_spad",
		pdata->roi_config__user_roi_centre_spad);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = 0x%02X\n",
		pprefix,
		"roi_config__user_roi_requested_global_xy_size",
		pdata->roi_config__user_roi_requested_global_xy_size);
}


void VL53LX_print_xtalk_histogram_shape_data(
	VL53LX_xtalk_histogram_shape_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t  i             = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_019",
		pdata->VL53LX_p_019);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_020",
		pdata->VL53LX_p_020);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_021",
		pdata->VL53LX_p_021);

	for (i = 0; i < pdata->VL53LX_p_021; i++) {

		sprintf(ppre_text, "%sbin_data[%u]", pprefix, i);

		VL53LX_signed_fixed_point_sprintf(
				(int32_t)pdata->bin_data[i],
				10,
				VL53LX_MAX_STRING_LENGTH,
				fp_text);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %s\n",
			ppre_text,
			fp_text);
	}

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->phasecal_result__reference_phase,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"phasecal_result__reference_phase",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"phasecal_result__vcsel_start",
		pdata->phasecal_result__vcsel_start);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_config__vcsel_start",
		pdata->cal_config__vcsel_start);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->vcsel_width,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"vcsel_width",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_015,
		12,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_015",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->zero_distance_phase,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"zero_distance_phase",
		fp_text);
}


void VL53LX_print_xtalk_histogram_data(
	VL53LX_xtalk_histogram_data_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags)
{


	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);


	sprintf(ppre_text, "%sxtalk_shape.", pprefix);
	VL53LX_print_xtalk_histogram_shape_data(
		&(pdata->xtalk_shape),
		ppre_text, trace_flags);


	sprintf(ppre_text, "%sxtalk_hist_removed.", pprefix);
	VL53LX_print_histogram_bin_data(
		&(pdata->xtalk_hist_removed),
		ppre_text, trace_flags);
}


void VL53LX_print_range_data(
	VL53LX_range_data_t *pdata,
	char                *pprefix,
	uint32_t             trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"range_id",
		pdata->range_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"time_stamp",
		pdata->time_stamp);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_012",
		pdata->VL53LX_p_012);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_019",
		pdata->VL53LX_p_019);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_023",
		pdata->VL53LX_p_023);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_024",
		pdata->VL53LX_p_024);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_013",
		pdata->VL53LX_p_013);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_025",
		pdata->VL53LX_p_025);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->width,
		 4,	VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"width",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_029",
		pdata->VL53LX_p_029);



	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->fast_osc_frequency,
		12,	VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"fast_osc_frequency",
		fp_text);



	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->zero_distance_phase,
		11, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"zero_distance_phase",
		fp_text);



	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_004,
		8, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"actual_effective_spad",
		fp_text);


	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"total_periods_elapsed",
		pdata->total_periods_elapsed);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"peak_duration_us",
		pdata->peak_duration_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"woi_duration_us",
		pdata->woi_duration_us);

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
			"VL53LX_p_016",
			pdata->VL53LX_p_016);

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
			"VL53LX_p_017",
			pdata->VL53LX_p_017);

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
			"VL53LX_p_010",
			pdata->VL53LX_p_010);



	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->peak_signal_count_rate_mcps,
		7, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"peak_signal_count_rate_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->avg_signal_count_rate_mcps,
		7, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"avg_signal_count_rate_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->ambient_count_rate_mcps,
		7, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"ambient_count_rate_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->total_rate_per_spad_mcps,
		13, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"total_rate_per_spad_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_009,
		11, VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_009",
		fp_text);



	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_002,
		 2,	VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_002",
		fp_text);



	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_026,
		11,	VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_026",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_011,
		11,	VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_011",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_027,
		11,	VL53LX_MAX_STRING_LENGTH, fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_027",
		fp_text);



	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"min_range_mm",
		pdata->min_range_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"median_range_mm",
		pdata->median_range_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"max_range_mm",
		pdata->max_range_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"range_status",
		pdata->range_status);
}


void VL53LX_print_range_results(
	VL53LX_range_results_t *pdata,
	char                   *pprefix,
	uint32_t                trace_flags)
{


	char     pre_text[VL53LX_MAX_STRING_LENGTH];
	char    *ppre_text = &(pre_text[0]);

	uint8_t  i = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cfg_device_state",
		pdata->cfg_device_state);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"rd_device_state",
		pdata->rd_device_state);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"zone_id",
		pdata->zone_id);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"stream_count",
		pdata->stream_count);

	for (i = 0; i < VL53LX_MAX_AMBIENT_DMAX_VALUES; i++) {
		sprintf(
			ppre_text,
			"%sambient_dmax_mm[%u]",
			pprefix, i);

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s = %u\n",
			ppre_text,
			pdata->VL53LX_p_022[i]);
	}

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s%s = %u\n",
			pprefix,
			"device_status",
			pdata->device_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"wrap_dmax_mm",
		pdata->wrap_dmax_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_results",
		pdata->max_results);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"active_results",
		pdata->active_results);

	for (i = 0; i < pdata->active_results; i++) {
		sprintf(ppre_text, "%sdata[%u].", pprefix, i);
		VL53LX_print_range_data(
			&pdata->VL53LX_p_003[i],
			ppre_text, trace_flags);
	}

	sprintf(ppre_text, "%sxmonitor.", pprefix);
	VL53LX_print_range_data(
		&pdata->xmonitor,
		ppre_text, trace_flags);
}


void VL53LX_print_offset_range_results(
	VL53LX_offset_range_results_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t  i = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_distance_mm",
		pdata->cal_distance_mm);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->cal_reflectance_pc,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"cal_reflectance_pc",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_status",
		pdata->cal_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_report",
		pdata->cal_report);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_results",
		pdata->max_results);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"active_results",
		pdata->active_results);

	for (i = 0; i < pdata->active_results; i++) {
		sprintf(ppre_text, "%sdata[%u].", pprefix, i);
		VL53LX_print_offset_range_data(
			&(pdata->VL53LX_p_003[i]),
			ppre_text, trace_flags);
	}
}


void VL53LX_print_offset_range_data(
	VL53LX_offset_range_data_t *pdata,
	char                       *pprefix,
	uint32_t                    trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"preset_mode",
		pdata->preset_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"dss_config__roi_mode_control",
		pdata->dss_config__roi_mode_control);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->dss_config__manual_effective_spads_select,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"dss_config__manual_effective_spads_select",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"no_of_samples",
		pdata->no_of_samples);


	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->effective_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"effective_spads",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->peak_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"peak_rate_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_002,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_002",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"median_range_mm",
		pdata->median_range_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"range_mm_offset",
		pdata->range_mm_offset);
}


void VL53LX_print_cal_peak_rate_map(
	VL53LX_cal_peak_rate_map_t *pdata,
	char                       *pprefix,
	uint32_t                    trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t   i = 0;
	uint8_t   x = 0;
	uint8_t   y = 0;

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->cal_distance_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"cal_distance_mm",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->cal_reflectance_pc,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"cal_reflectance_pc",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_samples",
		pdata->max_samples);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"width",
		pdata->width);

	trace_print(
	VL53LX_TRACE_LEVEL_INFO,
	"%s%s = %u\n",
	pprefix,
	"height",
	pdata->height);

	i = 0;
	for (y = 0; y < pdata->height; y++) {
		for (x = 0; x < pdata->width; x++) {

			sprintf(ppre_text, "%speak_rate_mcps[%u]", pprefix, i);

			VL53LX_signed_fixed_point_sprintf(
				(int32_t)pdata->peak_rate_mcps[i],
				7,
				VL53LX_MAX_STRING_LENGTH,
				fp_text);

			trace_print(
				VL53LX_TRACE_LEVEL_INFO,
				"%s = %s\n",
				ppre_text,
				fp_text);

			i++;
		}
	}
}

void VL53LX_print_additional_data(
	VL53LX_additional_data_t *pdata,
	char                     *pprefix,
	uint32_t                 trace_flags)
{



	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"preset_mode",
		pdata->preset_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"zone_preset",
		pdata->zone_preset);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"measurement_mode",
		pdata->measurement_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"offset_calibration_mode",
		pdata->offset_calibration_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"offset_correction_mode",
		pdata->offset_correction_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"dmax_mode",
		pdata->dmax_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"phasecal_config_timeout_us",
		pdata->phasecal_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"mm_config_timeout_us",
		pdata->mm_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"range_config_timeout_us",
		pdata->range_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"inter_measurement_period_ms",
		pdata->inter_measurement_period_ms);


	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->dss_config__target_total_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"dss_config__target_total_rate_mcps",
		fp_text);

	sprintf(ppre_text, "%s VL53LX_p_006.", pprefix);
		VL53LX_print_histogram_bin_data(
			&pdata->VL53LX_p_006,
			ppre_text, trace_flags);


}


void VL53LX_print_additional_offset_cal_data(
	VL53LX_additional_offset_cal_data_t *pdata,
	char                                *pprefix,
	uint32_t                             trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__mm_inner_actual_effective_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__mm_inner_actual_effective_spads",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__mm_outer_actual_effective_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__mm_outer_actual_effective_spads",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__mm_inner_peak_signal_count_rtn_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__mm_inner_peak_signal_count_rtn_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->result__mm_outer_peak_signal_count_rtn_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"result__mm_outer_peak_signal_count_rtn_mcps",
		fp_text);
}


void VL53LX_print_gain_calibration_data(
	VL53LX_gain_calibration_data_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->standard_ranging_gain_factor,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"standard_ranging_gain_factor",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->histogram_ranging_gain_factor,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"histogram_ranging_gain_factor",
		fp_text);
}


void VL53LX_print_zone_calibration_data(
	VL53LX_zone_calibration_data_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"no_of_samples",
		pdata->no_of_samples);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->effective_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"effective_spads",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->peak_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"peak_rate_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_011,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_011",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->VL53LX_p_002,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"VL53LX_p_002",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->median_range_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"median_range_mm",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->range_mm_offset,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"range_mm_offset",
		fp_text);
}


void VL53LX_print_zone_calibration_results(
	VL53LX_zone_calibration_results_t *pdata,
	char                              *pprefix,
	uint32_t                           trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t  i = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"preset_mode",
		pdata->preset_mode);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"zone_preset",
		pdata->zone_preset);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_distance_mm",
		pdata->cal_distance_mm);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->cal_reflectance_pc,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"cal_reflectance_pc",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->phasecal_result__reference_phase,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"phasecal_result__reference_phase",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->zero_distance_phase,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"zero_distance_phase",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_status",
		pdata->cal_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_zones",
		pdata->max_zones);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"active_zones",
		pdata->active_zones);

	for (i = 0; i < pdata->active_zones; i++) {
		sprintf(ppre_text, "%sdata[%u].", pprefix, i);
		VL53LX_print_zone_calibration_data(
			&(pdata->VL53LX_p_003[i]),
			ppre_text, trace_flags);
	}
}

void VL53LX_print_xtalk_range_results(
	VL53LX_xtalk_range_results_t *pdata,
	char                         *pprefix,
	uint32_t                      trace_flags)
{


	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);
	uint8_t  i = 0;

	VL53LX_histogram_bin_data_t *pbin_data;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"cal_status",
		pdata->cal_status);

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s%s = %u\n",
			pprefix,
			"num_of_samples_status",
			pdata->num_of_samples_status);

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s%s = %u\n",
			pprefix,
			"zero_samples_status",
			pdata->zero_samples_status);

	trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s%s = %u\n",
			pprefix,
			"max_sigma_status",
			pdata->max_sigma_status);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_results",
		pdata->max_results);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"active_results",
		pdata->active_results);

	for (i = 0; i < pdata->active_results; i++) {
		sprintf(ppre_text, "%sdata[%u].", pprefix, i);
		VL53LX_print_xtalk_range_data(
			&(pdata->VL53LX_p_003[i]),
			ppre_text, trace_flags);
	}

	sprintf(ppre_text, "%scentral_histogram_sum.", pprefix);
	VL53LX_print_histogram_bin_data(
		&pdata->central_histogram_sum,
		ppre_text, trace_flags);

	sprintf(ppre_text, "%scentral_histogram_avg.", pprefix);
	VL53LX_print_histogram_bin_data(
		&pdata->central_histogram_avg,
		ppre_text, trace_flags);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_012",
		pdata->central_histogram__window_start);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"VL53LX_p_013",
		pdata->central_histogram__window_end);

	pbin_data = &(pdata->histogram_avg_1[0]);

	for (i = 0; i < 5; i++) {
		sprintf(ppre_text, "%shistogram_avg_1[%u].", pprefix, i);
		VL53LX_print_histogram_bin_data(
			pbin_data,
			ppre_text, trace_flags);
			pbin_data++;
	}

	pbin_data = &(pdata->histogram_avg_2[0]);

	for (i = 0; i < 5; i++) {
		sprintf(ppre_text, "%shistogram_avg_2[%u].", pprefix, i);
		VL53LX_print_histogram_bin_data(
			pbin_data,
			ppre_text, trace_flags);
			pbin_data++;
	}

	pbin_data = &(pdata->xtalk_avg[0]);

	for (i = 0; i < 5; i++) {
		sprintf(ppre_text, "%sxtalk_avg[%u].", pprefix, i);
		VL53LX_print_histogram_bin_data(
			pbin_data,
			ppre_text, trace_flags);
			pbin_data++;
	}
}


void VL53LX_print_xtalk_range_data(
	VL53LX_xtalk_range_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"no_of_samples",
		pdata->no_of_samples);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"signal_total_events_sum",
		pdata->signal_total_events_sum);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %d\n",
		pprefix,
		"signal_total_events_avg",
		pdata->signal_total_events_avg);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->rate_per_spad_kcps_sum,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"rate_per_spad_kcps_sum",
		fp_text);


	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->rate_per_spad_kcps_avg,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"rate_per_spad_kcps_avg",
		fp_text);
}


void VL53LX_print_xtalk_calibration_results(
	VL53LX_xtalk_calibration_results_t *pdata,
	char                               *pprefix,
	uint32_t                            trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	int16_t tmpi16;

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_compensation_plane_offset_kcps,
		9,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_plane_offset_kcps",
		fp_text);

	tmpi16 = pdata->algo__crosstalk_compensation_x_plane_gradient_kcps;
	VL53LX_signed_fixed_point_sprintf(
		(int32_t)tmpi16,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_x_plane_gradient_kcps",
		fp_text);

	tmpi16 = pdata->algo__crosstalk_compensation_y_plane_gradient_kcps;
	VL53LX_signed_fixed_point_sprintf(
		(int32_t)tmpi16,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_y_plane_gradient_kcps",
		fp_text);
}


void VL53LX_print_xtalk_config(
	VL53LX_xtalk_config_t *pdata,
	char                  *pprefix,
	uint32_t               trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];
	int16_t tmpi16;

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_compensation_plane_offset_kcps,
		9,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_plane_offset_kcps",
		fp_text);

	tmpi16 = pdata->algo__crosstalk_compensation_x_plane_gradient_kcps;
	VL53LX_signed_fixed_point_sprintf(
		(int32_t)tmpi16,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_x_plane_gradient_kcps",
		fp_text);

	tmpi16 = pdata->algo__crosstalk_compensation_y_plane_gradient_kcps;
	VL53LX_signed_fixed_point_sprintf(
		(int32_t)tmpi16,
		11,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_compensation_y_plane_gradient_kcps",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"global_crosstalk_compensation_enable",
		pdata->global_crosstalk_compensation_enable);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->histogram_mode_crosstalk_margin_kcps,
		9,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"histogram_mode_crosstalk_margin_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->lite_mode_crosstalk_margin_kcps,
		9,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"lite_mode_crosstalk_margin_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->crosstalk_range_ignore_threshold_mult,
		5,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"crosstalk_range_ignore_threshold_mult",
		fp_text);


	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->crosstalk_range_ignore_threshold_rate_mcps,
		13,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"crosstalk_range_ignore_threshold_rate_mcps",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"algo__crosstalk_detect_max_valid_range_mm",
		pdata->algo__crosstalk_detect_max_valid_range_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"algo__crosstalk_detect_min_valid_range_mm",
		pdata->algo__crosstalk_detect_min_valid_range_mm);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_detect_max_valid_rate_kcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_detect_max_valid_rate_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_detect_max_sigma_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_detect_max_sigma_mm",
		fp_text);

}


void VL53LX_print_xtalk_extract_config(
	VL53LX_xtalkextract_config_t *pdata,
	char                         *pprefix,
	uint32_t                      trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];


	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->dss_config__target_total_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"dss_config__target_total_rate_mcps",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"mm_config_timeout_us",
		pdata->mm_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"range_config_timeout_us",
		pdata->range_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"num_of_samples",
		pdata->num_of_samples);


	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"algo__crosstalk_extract_max_valid_range_mm",
		pdata->algo__crosstalk_extract_max_valid_range_mm);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"algo__crosstalk_extract_min_valid_range_mm",
		pdata->algo__crosstalk_extract_min_valid_range_mm);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_extract_max_valid_rate_kcps,
		9,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_extract_max_valid_rate_kcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->algo__crosstalk_extract_max_sigma_mm,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"algo__crosstalk_extract_max_sigma_mm",
		fp_text);

}


void VL53LX_print_zone_cal_config(
	VL53LX_zonecal_config_t *pdata,
	char                    *pprefix,
	uint32_t                 trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->dss_config__target_total_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"dss_config__target_total_rate_mcps",
		fp_text);


	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"mm_config_timeout_us",
		pdata->mm_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"range_config_timeout_us",
		pdata->range_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"phasecal_config_timeout_us",
		pdata->phasecal_config_timeout_us);


	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"phasecal_num_of_samples",
		pdata->phasecal_num_of_samples);


	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"zone_num_of_samples",
		pdata->zone_num_of_samples);

}

void VL53LX_print_offset_cal_config(
	VL53LX_offsetcal_config_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->dss_config__target_total_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"dss_config__target_total_rate_mcps",
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"phasecal_config_timeout_us",
		pdata->phasecal_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"range_config_timeout_us",
		pdata->range_config_timeout_us);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"pre_num_of_samples",
		pdata->pre_num_of_samples);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"mm1_num_of_samples",
		pdata->mm1_num_of_samples);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"mm2_num_of_samples",
		pdata->mm2_num_of_samples);


}


void VL53LX_print_dmax_calibration_data(
	VL53LX_dmax_calibration_data_t *pdata,
	char                           *pprefix,
	uint32_t                        trace_flags)
{


	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->ref__actual_effective_spads,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"ref__actual_effective_spads",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->ref__peak_signal_count_rate_mcps,
		7,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"ref__peak_signal_count_rate_mcps",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->ref__distance_mm,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"ref__distance_mm",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->ref_reflectance_pc,
		2,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"ref_reflectance_pc",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->coverglass_transmission,
		8,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"coverglass_transmission",
		fp_text);
}


void VL53LX_print_calibration_data(
	VL53LX_calibration_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags)
{


	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = 0x%08X\n",
		pprefix,
		"struct_version",
		pdata->struct_version);

	sprintf(ppre_text, "%scustomer.", pprefix);
	VL53LX_print_customer_nvm_managed(
		&(pdata->customer),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sfmt_dmax_cal.", pprefix);
	VL53LX_print_dmax_calibration_data(
		&(pdata->fmt_dmax_cal),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%scust_dmax_cal.", pprefix);
	VL53LX_print_dmax_calibration_data(
		&(pdata->cust_dmax_cal),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sadd_off_cal_data.", pprefix);
	VL53LX_print_additional_offset_cal_data(
		&(pdata->add_off_cal_data),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%soptical_centre.", pprefix);
	VL53LX_print_optical_centre(
		&(pdata->optical_centre),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sxtalkhisto.", pprefix);
	VL53LX_print_xtalk_histogram_data(
		&(pdata->xtalkhisto),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sgain_cal.", pprefix);
	VL53LX_print_gain_calibration_data(
		&(pdata->gain_cal),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%scal_peak_rate_map.", pprefix);
	VL53LX_print_cal_peak_rate_map(
		&(pdata->cal_peak_rate_map),
		ppre_text, trace_flags);
}


void VL53LX_print_xtalk_debug_data(
	VL53LX_xtalk_debug_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags)
{


	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	sprintf(ppre_text, "%scustomer.", pprefix);
	VL53LX_print_customer_nvm_managed(
		&(pdata->customer),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sxtalk_cfg.", pprefix);
	VL53LX_print_xtalk_config(
		&(pdata->xtalk_cfg),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sxtalk_extract_cfg.", pprefix);
	VL53LX_print_xtalk_extract_config(
		&(pdata->xtalk_extract_cfg),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%shist_data.", pprefix);
	VL53LX_print_histogram_bin_data(
		&(pdata->hist_data),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sxtalk_shapes.", pprefix);
	VL53LX_print_xtalk_histogram_data(
		&(pdata->xtalk_shapes),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sgain_cal.", pprefix);
	VL53LX_print_xtalk_range_results(
		&(pdata->xtalk_results),
		ppre_text, trace_flags);
}


void VL53LX_print_offset_debug_data(
	VL53LX_offset_debug_data_t *pdata,
	char                       *pprefix,
	uint32_t                    trace_flags)
{


	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	sprintf(ppre_text, "%scustomer.", pprefix);
	VL53LX_print_customer_nvm_managed(
		&(pdata->customer),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sfmt_dmax_cal.", pprefix);
	VL53LX_print_dmax_calibration_data(
		&(pdata->fmt_dmax_cal),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%scust_dmax_cal.", pprefix);
	VL53LX_print_dmax_calibration_data(
		&(pdata->cust_dmax_cal),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%sadd_off_cal_data.", pprefix);
	VL53LX_print_additional_offset_cal_data(
		&(pdata->add_off_cal_data),
		ppre_text, trace_flags);

	sprintf(ppre_text, "%soffset_results.", pprefix);
	VL53LX_print_offset_range_results(
		&(pdata->offset_results),
		ppre_text, trace_flags);
}


void VL53LX_print_zone_config(
	VL53LX_zone_config_t *pdata,
	char                 *pprefix,
	uint32_t              trace_flags)
{



	char  pre_text[VL53LX_MAX_STRING_LENGTH];
	char *ppre_text = &(pre_text[0]);

	uint8_t  i = 0;

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"max_zones",
		pdata->max_zones);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"active_zones",
		pdata->active_zones);

	for (i = 0; i < pdata->active_zones; i++) {
		sprintf(ppre_text, "%suser_zones[%u].", pprefix, i);
		VL53LX_print_user_zone(
			&pdata->user_zones[i],
			ppre_text,
			trace_flags);
	}
}


void VL53LX_print_optical_centre(
	VL53LX_optical_centre_t  *pdata,
	char                     *pprefix,
	uint32_t                  trace_flags)
{



	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->x_centre,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"x_centre",
		fp_text);

	VL53LX_signed_fixed_point_sprintf(
		(int32_t)pdata->y_centre,
		4,
		VL53LX_MAX_STRING_LENGTH,
		fp_text);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %s\n",
		pprefix,
		"y_centre",
		fp_text);
}


void VL53LX_print_user_zone(
	VL53LX_user_zone_t   *pdata,
	char                 *pprefix,
	uint32_t              trace_flags)
{



	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"x_centre",
		pdata->x_centre);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"y_centre",
		pdata->y_centre);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"width",
		pdata->width);

	trace_print(VL53LX_TRACE_LEVEL_INFO,
		"%s%s = %u\n",
		pprefix,
		"height",
		pdata->height);
}


void VL53LX_print_spad_rate_data(
	VL53LX_spad_rate_data_t  *pspad_rates,
	char                     *pprefix,
	uint32_t                  trace_flags)
{



	uint16_t spad_no = 0;
	uint8_t  row     = 0;
	uint8_t  col     = 0;

	char  fp_text[VL53LX_MAX_STRING_LENGTH];

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%8s,%4s,%4s, %s\n",
		pprefix,
		"spad_no",
		"row",
		"col",
		"peak_rate_mcps");

	for (spad_no = 0; spad_no < pspad_rates->no_of_values; spad_no++) {


		VL53LX_decode_row_col(
			(uint8_t)spad_no,
			&row,
			&col);



		VL53LX_signed_fixed_point_sprintf(
			(int32_t)pspad_rates->rate_data[spad_no],
			pspad_rates->fractional_bits,
			VL53LX_MAX_STRING_LENGTH,
			fp_text);



		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s%8u,%4u,%4u, %s\n",
			pprefix,
			spad_no,
			row,
			col,
			fp_text);
	}
}


void VL53LX_print_spad_rate_map(
	VL53LX_spad_rate_data_t  *pspad_rates,
	char                     *pprefix,
	uint32_t                  trace_flags)
{



	uint8_t  spad_no = 0;
	uint8_t  row     = 0;
	uint8_t  col     = 0;

	char  fp_text[VL53LX_MAX_STRING_LENGTH];


	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"%s%4s",
		pprefix,
		" ");

	for (col = 0;  col < VL53LX_SPAD_ARRAY_WIDTH; col++)
		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			",%8u",
			col);

	trace_print(
		VL53LX_TRACE_LEVEL_INFO,
		"\n");



	for (row = 0;  row < VL53LX_SPAD_ARRAY_HEIGHT; row++) {

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"%s%4u",
			pprefix,
			row);

		for (col = 0;  col < VL53LX_SPAD_ARRAY_HEIGHT; col++) {



			VL53LX_encode_row_col(
				row,
				col,
				&spad_no);



			VL53LX_signed_fixed_point_sprintf(
				(int32_t)pspad_rates->rate_data[spad_no],
				pspad_rates->fractional_bits,
				VL53LX_MAX_STRING_LENGTH,
				fp_text);



			trace_print(
				VL53LX_TRACE_LEVEL_INFO,
				",%8s",
				fp_text);
		}

		trace_print(
			VL53LX_TRACE_LEVEL_INFO,
			"\n");
	}
}


#endif


