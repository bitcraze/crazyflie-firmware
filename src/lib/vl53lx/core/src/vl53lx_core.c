
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#include "vl53lx_platform.h"
#include "vl53lx_ll_def.h"
#include "vl53lx_ll_device.h"
#include "vl53lx_register_map.h"
#include "vl53lx_register_funcs.h"
#include "vl53lx_register_settings.h"
#include "vl53lx_hist_structs.h"
#include "vl53lx_api_preset_modes.h"
#include "vl53lx_core.h"
#include "vl53lx_tuning_parm_defaults.h"



#define LOG_FUNCTION_START(fmt, ...)
#define LOG_FUNCTION_END(status, ...)
#define LOG_FUNCTION_END_FMT(status, fmt, ...)

#define trace_print(level, ...)


void  VL53LX_init_version(
	VL53LX_DEV        Dev)
{


	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	pdev->version.ll_major    = VL53LX_LL_API_IMPLEMENTATION_VER_MAJOR;
	pdev->version.ll_minor    = VL53LX_LL_API_IMPLEMENTATION_VER_MINOR;
	pdev->version.ll_build    = VL53LX_LL_API_IMPLEMENTATION_VER_SUB;
	pdev->version.ll_revision = VL53LX_LL_API_IMPLEMENTATION_VER_REVISION;
}


void  VL53LX_init_ll_driver_state(
	VL53LX_DEV         Dev,
	VL53LX_DeviceState device_state)
{


	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);

	pstate->cfg_device_state  = device_state;
	pstate->cfg_stream_count  = 0;
	pstate->cfg_gph_id        = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
	pstate->cfg_timing_status = 0;
	pstate->cfg_zone_id       = 0;

	pstate->rd_device_state   = device_state;
	pstate->rd_stream_count   = 0;
	pstate->rd_gph_id         = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
	pstate->rd_timing_status  = 0;
	pstate->rd_zone_id        = 0;

}


VL53LX_Error  VL53LX_update_ll_driver_rd_state(
	VL53LX_DEV         Dev)
{


	VL53LX_Error        status  = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);



	LOG_FUNCTION_START("");



	if ((pdev->sys_ctrl.system__mode_start &
		VL53LX_DEVICEMEASUREMENTMODE_MODE_MASK) == 0x00) {

		pstate->rd_device_state  = VL53LX_DEVICESTATE_SW_STANDBY;
		pstate->rd_stream_count  = 0;
		pstate->rd_internal_stream_count = 0;
		pstate->rd_internal_stream_count_val = 0;
		pstate->rd_gph_id = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
		pstate->rd_timing_status = 0;
		pstate->rd_zone_id       = 0;

	} else {



		if (pstate->rd_stream_count == 0xFF)
			pstate->rd_stream_count = 0x80;
		else
			pstate->rd_stream_count++;


		status = VL53LX_update_internal_stream_counters(Dev,
			pstate->rd_stream_count,
			&(pstate->rd_internal_stream_count),
			&(pstate->rd_internal_stream_count_val));



		pstate->rd_gph_id ^= VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;



		switch (pstate->rd_device_state) {

		case VL53LX_DEVICESTATE_SW_STANDBY:

			if ((pdev->dyn_cfg.system__grouped_parameter_hold &
				VL53LX_GROUPEDPARAMETERHOLD_ID_MASK) > 0) {
				pstate->rd_device_state =
				VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC;
			} else {
				if (pstate->rd_zone_id >=
					pdev->zone_cfg.active_zones)
					pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
				else
					pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_GATHER_DATA;
			}

			pstate->rd_stream_count  = 0;
			pstate->rd_internal_stream_count = 0;
			pstate->rd_internal_stream_count_val = 0;
			pstate->rd_timing_status = 0;
			pstate->rd_zone_id       = 0;

			break;

		case VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC:
			pstate->rd_stream_count = 0;
			pstate->rd_internal_stream_count = 0;
			pstate->rd_internal_stream_count_val = 0;
			pstate->rd_zone_id      = 0;
			if (pstate->rd_zone_id >=
				pdev->zone_cfg.active_zones)
				pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
			else
				pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_GATHER_DATA;

			break;

		case VL53LX_DEVICESTATE_RANGING_GATHER_DATA:
			pstate->rd_zone_id++;
			if (pstate->rd_zone_id >=
				pdev->zone_cfg.active_zones)
				pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
			else
				pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_GATHER_DATA;

			break;

		case VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA:
			pstate->rd_zone_id        = 0;
			pstate->rd_timing_status ^= 0x01;

			if (pstate->rd_zone_id >=
				pdev->zone_cfg.active_zones)
				pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA;
			else
				pstate->rd_device_state =
					VL53LX_DEVICESTATE_RANGING_GATHER_DATA;
			break;

		default:
			pstate->rd_device_state  =
				VL53LX_DEVICESTATE_SW_STANDBY;
			pstate->rd_stream_count  = 0;
			pstate->rd_internal_stream_count = 0;
			pstate->rd_internal_stream_count_val = 0;
			pstate->rd_gph_id = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
			pstate->rd_timing_status = 0;
			pstate->rd_zone_id       = 0;
			break;
		}
	}



	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_check_ll_driver_rd_state(
	VL53LX_DEV         Dev)
{


	VL53LX_Error         status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t  *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t  *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	VL53LX_ll_driver_state_t  *pstate       = &(pdev->ll_state);
	VL53LX_system_results_t   *psys_results = &(pdev->sys_results);
	VL53LX_histogram_bin_data_t *phist_data = &(pdev->VL53LX_LLDriverCommonData->hist_data);
	VL53LX_zone_private_dyn_cfgs_t *pZ = &(pres->zone_dyn_cfgs);

	uint8_t   device_range_status   = 0;
	uint8_t   device_stream_count   = 0;
	uint8_t   device_gph_id         = 0;
	uint8_t   histogram_mode        = 0;
	uint8_t   expected_stream_count = 0;
	uint8_t   expected_gph_id       = 0;

	LOG_FUNCTION_START("");



	device_range_status =
			psys_results->result__range_status &
			VL53LX_RANGE_STATUS__RANGE_STATUS_MASK;

	device_stream_count = psys_results->result__stream_count;



	histogram_mode =
		(pdev->sys_ctrl.system__mode_start &
		VL53LX_DEVICESCHEDULERMODE_HISTOGRAM) ==
		VL53LX_DEVICESCHEDULERMODE_HISTOGRAM;


	device_gph_id = (psys_results->result__interrupt_status &
		VL53LX_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK) >> 4;

	if (histogram_mode)
		device_gph_id = (phist_data->result__interrupt_status &
			VL53LX_INTERRUPT_STATUS__GPH_ID_INT_STATUS_MASK) >> 4;



	if (!((pdev->sys_ctrl.system__mode_start &
		VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK) ==
		VL53LX_DEVICEMEASUREMENTMODE_BACKTOBACK))
		goto ENDFUNC;



	if (pstate->rd_device_state ==
		VL53LX_DEVICESTATE_RANGING_WAIT_GPH_SYNC) {

		if (histogram_mode == 0) {
			if (device_range_status !=
			VL53LX_DEVICEERROR_GPHSTREAMCOUNT0READY)
				status =
				VL53LX_ERROR_GPH_SYNC_CHECK_FAIL;

		}
	} else {
		if (pstate->rd_stream_count != device_stream_count)
			status = VL53LX_ERROR_STREAM_COUNT_CHECK_FAIL;


		if (pstate->rd_gph_id != device_gph_id)
			status = VL53LX_ERROR_GPH_ID_CHECK_FAIL;




		expected_stream_count =
		pZ->VL53LX_p_003[pstate->rd_zone_id].expected_stream_count;
		expected_gph_id =
		pZ->VL53LX_p_003[pstate->rd_zone_id].expected_gph_id;



		if (expected_stream_count != device_stream_count) {


			if (!((pdev->zone_cfg.active_zones == 0) &&
				(device_stream_count == 255)))
				status =
				VL53LX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL;


		}



		if (expected_gph_id != device_gph_id)
			status = VL53LX_ERROR_ZONE_GPH_ID_CHECK_FAIL;

	}



ENDFUNC:
	LOG_FUNCTION_END(status);
	return status;
}


VL53LX_Error  VL53LX_update_ll_driver_cfg_state(
	VL53LX_DEV         Dev)
{


	VL53LX_Error         status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t  *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t  *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);
	VL53LX_zone_private_dyn_cfgs_t *pZ = &(pres->zone_dyn_cfgs);

	uint8_t prev_cfg_zone_id;
	uint8_t prev_cfg_gph_id;
	uint8_t prev_cfg_stream_count;

	LOG_FUNCTION_START("");





	if ((pdev->sys_ctrl.system__mode_start &
		VL53LX_DEVICEMEASUREMENTMODE_MODE_MASK) == 0x00) {

		pstate->cfg_device_state  = VL53LX_DEVICESTATE_SW_STANDBY;
		pstate->cfg_stream_count  = 0;
		pstate->cfg_internal_stream_count = 0;
		pstate->cfg_internal_stream_count_val = 0;
		pstate->cfg_gph_id = VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
		pstate->cfg_timing_status = 0;
		pstate->cfg_zone_id       = 0;
		prev_cfg_zone_id          = 0;
		prev_cfg_gph_id           = 0;
		prev_cfg_stream_count     = 0;

	} else {

		prev_cfg_gph_id           = pstate->cfg_gph_id;
		prev_cfg_zone_id          = pstate->cfg_zone_id;
		prev_cfg_stream_count     = pstate->cfg_stream_count;



		if (pstate->cfg_stream_count == 0xFF)
			pstate->cfg_stream_count = 0x80;
		else
			pstate->cfg_stream_count++;


		status = VL53LX_update_internal_stream_counters(
			Dev,
			pstate->cfg_stream_count,
			&(pstate->cfg_internal_stream_count),
			&(pstate->cfg_internal_stream_count_val));



		pstate->cfg_gph_id ^= VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;



		switch (pstate->cfg_device_state) {

		case VL53LX_DEVICESTATE_SW_STANDBY:
			pstate->cfg_zone_id = 1;
			if (pstate->cfg_zone_id >
				pdev->zone_cfg.active_zones) {
				pstate->cfg_zone_id = 0;
				pstate->cfg_timing_status ^= 0x01;
			}
			pstate->cfg_stream_count = 1;

			if (pdev->gen_cfg.global_config__stream_divider == 0) {
				pstate->cfg_internal_stream_count = 1;
				pstate->cfg_internal_stream_count_val = 0;
			} else {
				pstate->cfg_internal_stream_count = 0;
				pstate->cfg_internal_stream_count_val = 1;
			}
			pstate->cfg_device_state =
					VL53LX_DEVICESTATE_RANGING_DSS_AUTO;
			break;

		case VL53LX_DEVICESTATE_RANGING_DSS_AUTO:
			pstate->cfg_zone_id++;
			if (pstate->cfg_zone_id >
				pdev->zone_cfg.active_zones) {

				pstate->cfg_zone_id = 0;
				pstate->cfg_timing_status ^= 0x01;




				if (pdev->zone_cfg.active_zones > 0) {
					pstate->cfg_device_state =
					VL53LX_DEVICESTATE_RANGING_DSS_MANUAL;
				}
			}
			break;

		case VL53LX_DEVICESTATE_RANGING_DSS_MANUAL:
			pstate->cfg_zone_id++;
			if (pstate->cfg_zone_id >
				pdev->zone_cfg.active_zones) {
				pstate->cfg_zone_id = 0;
				pstate->cfg_timing_status ^= 0x01;
			}
			break;

		default:
			pstate->cfg_device_state =
					VL53LX_DEVICESTATE_SW_STANDBY;
			pstate->cfg_stream_count = 0;
			pstate->cfg_internal_stream_count = 0;
			pstate->cfg_internal_stream_count_val = 0;
			pstate->cfg_gph_id =
					VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
			pstate->cfg_timing_status = 0;
			pstate->cfg_zone_id       = 0;
			break;
		}
	}


	if (pdev->zone_cfg.active_zones == 0) {

		pZ->VL53LX_p_003[prev_cfg_zone_id].expected_stream_count
			= prev_cfg_stream_count - 1;

		pZ->VL53LX_p_003[pstate->rd_zone_id].expected_gph_id =
			prev_cfg_gph_id ^ VL53LX_GROUPEDPARAMETERHOLD_ID_MASK;
	} else {
		pZ->VL53LX_p_003[prev_cfg_zone_id].expected_stream_count
			= prev_cfg_stream_count;
		pZ->VL53LX_p_003[prev_cfg_zone_id].expected_gph_id =
			prev_cfg_gph_id;
	}



	LOG_FUNCTION_END(status);

	return status;
}


void VL53LX_copy_rtn_good_spads_to_buffer(
	VL53LX_nvm_copy_data_t  *pdata,
	uint8_t                 *pbuffer)
{


	*(pbuffer +  0) = pdata->global_config__spad_enables_rtn_0;
	*(pbuffer +  1) = pdata->global_config__spad_enables_rtn_1;
	*(pbuffer +  2) = pdata->global_config__spad_enables_rtn_2;
	*(pbuffer +  3) = pdata->global_config__spad_enables_rtn_3;
	*(pbuffer +  4) = pdata->global_config__spad_enables_rtn_4;
	*(pbuffer +  5) = pdata->global_config__spad_enables_rtn_5;
	*(pbuffer +  6) = pdata->global_config__spad_enables_rtn_6;
	*(pbuffer +  7) = pdata->global_config__spad_enables_rtn_7;
	*(pbuffer +  8) = pdata->global_config__spad_enables_rtn_8;
	*(pbuffer +  9) = pdata->global_config__spad_enables_rtn_9;
	*(pbuffer + 10) = pdata->global_config__spad_enables_rtn_10;
	*(pbuffer + 11) = pdata->global_config__spad_enables_rtn_11;
	*(pbuffer + 12) = pdata->global_config__spad_enables_rtn_12;
	*(pbuffer + 13) = pdata->global_config__spad_enables_rtn_13;
	*(pbuffer + 14) = pdata->global_config__spad_enables_rtn_14;
	*(pbuffer + 15) = pdata->global_config__spad_enables_rtn_15;
	*(pbuffer + 16) = pdata->global_config__spad_enables_rtn_16;
	*(pbuffer + 17) = pdata->global_config__spad_enables_rtn_17;
	*(pbuffer + 18) = pdata->global_config__spad_enables_rtn_18;
	*(pbuffer + 19) = pdata->global_config__spad_enables_rtn_19;
	*(pbuffer + 20) = pdata->global_config__spad_enables_rtn_20;
	*(pbuffer + 21) = pdata->global_config__spad_enables_rtn_21;
	*(pbuffer + 22) = pdata->global_config__spad_enables_rtn_22;
	*(pbuffer + 23) = pdata->global_config__spad_enables_rtn_23;
	*(pbuffer + 24) = pdata->global_config__spad_enables_rtn_24;
	*(pbuffer + 25) = pdata->global_config__spad_enables_rtn_25;
	*(pbuffer + 26) = pdata->global_config__spad_enables_rtn_26;
	*(pbuffer + 27) = pdata->global_config__spad_enables_rtn_27;
	*(pbuffer + 28) = pdata->global_config__spad_enables_rtn_28;
	*(pbuffer + 29) = pdata->global_config__spad_enables_rtn_29;
	*(pbuffer + 30) = pdata->global_config__spad_enables_rtn_30;
	*(pbuffer + 31) = pdata->global_config__spad_enables_rtn_31;
}


void VL53LX_init_system_results(
		VL53LX_system_results_t  *pdata)
{


	pdata->result__interrupt_status                       = 0xFF;
	pdata->result__range_status                           = 0xFF;
	pdata->result__report_status                          = 0xFF;
	pdata->result__stream_count                           = 0xFF;

	pdata->result__dss_actual_effective_spads_sd0         = 0xFFFF;
	pdata->result__peak_signal_count_rate_mcps_sd0        = 0xFFFF;
	pdata->result__ambient_count_rate_mcps_sd0            = 0xFFFF;
	pdata->result__sigma_sd0                              = 0xFFFF;
	pdata->result__phase_sd0                              = 0xFFFF;
	pdata->result__final_crosstalk_corrected_range_mm_sd0 = 0xFFFF;
	pdata->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
			0xFFFF;
	pdata->result__mm_inner_actual_effective_spads_sd0    = 0xFFFF;
	pdata->result__mm_outer_actual_effective_spads_sd0    = 0xFFFF;
	pdata->result__avg_signal_count_rate_mcps_sd0         = 0xFFFF;

	pdata->result__dss_actual_effective_spads_sd1         = 0xFFFF;
	pdata->result__peak_signal_count_rate_mcps_sd1        = 0xFFFF;
	pdata->result__ambient_count_rate_mcps_sd1            = 0xFFFF;
	pdata->result__sigma_sd1                              = 0xFFFF;
	pdata->result__phase_sd1                              = 0xFFFF;
	pdata->result__final_crosstalk_corrected_range_mm_sd1 = 0xFFFF;
	pdata->result__spare_0_sd1                            = 0xFFFF;
	pdata->result__spare_1_sd1                            = 0xFFFF;
	pdata->result__spare_2_sd1                            = 0xFFFF;
	pdata->result__spare_3_sd1                            = 0xFF;

}


void V53L1_init_zone_results_structure(
	uint8_t                 active_zones,
	VL53LX_zone_results_t  *pdata)
{



	uint8_t  z = 0;
	VL53LX_zone_objects_t *pobjects;

	pdata->max_zones    = VL53LX_MAX_USER_ZONES;
	pdata->active_zones = active_zones;

	for (z = 0; z < pdata->max_zones; z++) {
		pobjects = &(pdata->VL53LX_p_003[z]);
		pobjects->cfg_device_state = VL53LX_DEVICESTATE_SW_STANDBY;
		pobjects->rd_device_state  = VL53LX_DEVICESTATE_SW_STANDBY;
		pobjects->max_objects      = VL53LX_MAX_RANGE_RESULTS;
		pobjects->active_objects   = 0;
	}
}

void V53L1_init_zone_dss_configs(
	VL53LX_DEV              Dev)
{



	VL53LX_LLDriverResults_t  *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);
	uint8_t  z = 0;
	uint8_t max_zones    = VL53LX_MAX_USER_ZONES;
	VL53LX_zone_private_dyn_cfgs_t *pdata = &(pres->zone_dyn_cfgs);

	for (z = 0; z < max_zones; z++) {
		pdata->VL53LX_p_003[z].dss_mode =
			VL53LX_DSS_CONTROL__MODE_TARGET_RATE;
		pdata->VL53LX_p_003[z].dss_requested_effective_spad_count = 0;
	}
}


void VL53LX_init_histogram_config_structure(
	uint8_t   even_bin0,
	uint8_t   even_bin1,
	uint8_t   even_bin2,
	uint8_t   even_bin3,
	uint8_t   even_bin4,
	uint8_t   even_bin5,
	uint8_t   odd_bin0,
	uint8_t   odd_bin1,
	uint8_t   odd_bin2,
	uint8_t   odd_bin3,
	uint8_t   odd_bin4,
	uint8_t   odd_bin5,
	VL53LX_histogram_config_t  *pdata)
{


	pdata->histogram_config__low_amb_even_bin_0_1  =
			(even_bin1 << 4) + even_bin0;
	pdata->histogram_config__low_amb_even_bin_2_3  =
			(even_bin3 << 4) + even_bin2;
	pdata->histogram_config__low_amb_even_bin_4_5  =
			(even_bin5 << 4) + even_bin4;

	pdata->histogram_config__low_amb_odd_bin_0_1   =
			(odd_bin1 << 4) + odd_bin0;
	pdata->histogram_config__low_amb_odd_bin_2_3   =
			(odd_bin3 << 4) + odd_bin2;
	pdata->histogram_config__low_amb_odd_bin_4_5   =
			(odd_bin5 << 4) + odd_bin4;

	pdata->histogram_config__mid_amb_even_bin_0_1  =
			pdata->histogram_config__low_amb_even_bin_0_1;
	pdata->histogram_config__mid_amb_even_bin_2_3  =
			pdata->histogram_config__low_amb_even_bin_2_3;
	pdata->histogram_config__mid_amb_even_bin_4_5  =
			pdata->histogram_config__low_amb_even_bin_4_5;

	pdata->histogram_config__mid_amb_odd_bin_0_1   =
			pdata->histogram_config__low_amb_odd_bin_0_1;
	pdata->histogram_config__mid_amb_odd_bin_2     = odd_bin2;
	pdata->histogram_config__mid_amb_odd_bin_3_4   =
			(odd_bin4 << 4) + odd_bin3;
	pdata->histogram_config__mid_amb_odd_bin_5     = odd_bin5;

	pdata->histogram_config__user_bin_offset       = 0x00;

	pdata->histogram_config__high_amb_even_bin_0_1 =
			pdata->histogram_config__low_amb_even_bin_0_1;
	pdata->histogram_config__high_amb_even_bin_2_3 =
			pdata->histogram_config__low_amb_even_bin_2_3;
	pdata->histogram_config__high_amb_even_bin_4_5 =
			pdata->histogram_config__low_amb_even_bin_4_5;

	pdata->histogram_config__high_amb_odd_bin_0_1  =
			pdata->histogram_config__low_amb_odd_bin_0_1;
	pdata->histogram_config__high_amb_odd_bin_2_3  =
			pdata->histogram_config__low_amb_odd_bin_2_3;
	pdata->histogram_config__high_amb_odd_bin_4_5  =
			pdata->histogram_config__low_amb_odd_bin_4_5;



	pdata->histogram_config__amb_thresh_low        = 0xFFFF;
	pdata->histogram_config__amb_thresh_high       = 0xFFFF;



	pdata->histogram_config__spad_array_selection  = 0x00;

}

void VL53LX_init_histogram_multizone_config_structure(
	uint8_t   even_bin0,
	uint8_t   even_bin1,
	uint8_t   even_bin2,
	uint8_t   even_bin3,
	uint8_t   even_bin4,
	uint8_t   even_bin5,
	uint8_t   odd_bin0,
	uint8_t   odd_bin1,
	uint8_t   odd_bin2,
	uint8_t   odd_bin3,
	uint8_t   odd_bin4,
	uint8_t   odd_bin5,
	VL53LX_histogram_config_t  *pdata)
{


	pdata->histogram_config__low_amb_even_bin_0_1  =
			(even_bin1 << 4) + even_bin0;
	pdata->histogram_config__low_amb_even_bin_2_3  =
			(even_bin3 << 4) + even_bin2;
	pdata->histogram_config__low_amb_even_bin_4_5  =
			(even_bin5 << 4) + even_bin4;

	pdata->histogram_config__low_amb_odd_bin_0_1   =
			pdata->histogram_config__low_amb_even_bin_0_1;
	pdata->histogram_config__low_amb_odd_bin_2_3
		= pdata->histogram_config__low_amb_even_bin_2_3;
	pdata->histogram_config__low_amb_odd_bin_4_5
		= pdata->histogram_config__low_amb_even_bin_4_5;

	pdata->histogram_config__mid_amb_even_bin_0_1  =
		pdata->histogram_config__low_amb_even_bin_0_1;
	pdata->histogram_config__mid_amb_even_bin_2_3
		= pdata->histogram_config__low_amb_even_bin_2_3;
	pdata->histogram_config__mid_amb_even_bin_4_5
		= pdata->histogram_config__low_amb_even_bin_4_5;

	pdata->histogram_config__mid_amb_odd_bin_0_1
		= pdata->histogram_config__low_amb_odd_bin_0_1;
	pdata->histogram_config__mid_amb_odd_bin_2     = odd_bin2;
	pdata->histogram_config__mid_amb_odd_bin_3_4   =
			(odd_bin4 << 4) + odd_bin3;
	pdata->histogram_config__mid_amb_odd_bin_5     = odd_bin5;

	pdata->histogram_config__user_bin_offset       = 0x00;

	pdata->histogram_config__high_amb_even_bin_0_1 =
			(odd_bin1 << 4) + odd_bin0;
	pdata->histogram_config__high_amb_even_bin_2_3 =
			(odd_bin3 << 4) + odd_bin2;
	pdata->histogram_config__high_amb_even_bin_4_5 =
			(odd_bin5 << 4) + odd_bin4;

	pdata->histogram_config__high_amb_odd_bin_0_1
		= pdata->histogram_config__high_amb_even_bin_0_1;
	pdata->histogram_config__high_amb_odd_bin_2_3
		= pdata->histogram_config__high_amb_even_bin_2_3;
	pdata->histogram_config__high_amb_odd_bin_4_5
		= pdata->histogram_config__high_amb_even_bin_4_5;



	pdata->histogram_config__amb_thresh_low        = 0xFFFF;
	pdata->histogram_config__amb_thresh_high       = 0xFFFF;



	pdata->histogram_config__spad_array_selection  = 0x00;
}


void VL53LX_init_xtalk_bin_data_struct(
	uint32_t                        bin_value,
	uint16_t                        VL53LX_p_021,
	VL53LX_xtalk_histogram_shape_t *pdata)
{



	uint16_t          i = 0;

	pdata->zone_id                   = 0;
	pdata->time_stamp                = 0;

	pdata->VL53LX_p_019                 = 0;
	pdata->VL53LX_p_020               = VL53LX_XTALK_HISTO_BINS;
	pdata->VL53LX_p_021            = (uint8_t)VL53LX_p_021;

	pdata->phasecal_result__reference_phase   = 0;
	pdata->phasecal_result__vcsel_start       = 0;
	pdata->cal_config__vcsel_start            = 0;

	pdata->vcsel_width                        = 0;
	pdata->VL53LX_p_015                = 0;

	pdata->zero_distance_phase                = 0;

	for (i = 0; i < VL53LX_XTALK_HISTO_BINS; i++) {
		if (i < VL53LX_p_021)
			pdata->bin_data[i] = bin_value;
		else
			pdata->bin_data[i] = 0;
	}
}


void VL53LX_i2c_encode_uint16_t(
	uint16_t    ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{


	uint16_t   i    = 0;
	uint16_t   VL53LX_p_003 = 0;

	VL53LX_p_003 =  ip_value;

	for (i = 0; i < count; i++) {
		pbuffer[count-i-1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
		VL53LX_p_003 = VL53LX_p_003 >> 8;
	}
}

uint16_t VL53LX_i2c_decode_uint16_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{


	uint16_t   value = 0x00;

	while (count-- > 0)
		value = (value << 8) | (uint16_t)*pbuffer++;

	return value;
}


void VL53LX_i2c_encode_int16_t(
	int16_t     ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{


	uint16_t   i    = 0;
	int16_t    VL53LX_p_003 = 0;

	VL53LX_p_003 =  ip_value;

	for (i = 0; i < count; i++) {
		pbuffer[count-i-1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
		VL53LX_p_003 = VL53LX_p_003 >> 8;
	}
}

int16_t VL53LX_i2c_decode_int16_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{


	int16_t    value = 0x00;


	if (*pbuffer >= 0x80)
		value = 0xFFFF;

	while (count-- > 0)
		value = (value << 8) | (int16_t)*pbuffer++;

	return value;
}

void VL53LX_i2c_encode_uint32_t(
	uint32_t    ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{


	uint16_t   i    = 0;
	uint32_t   VL53LX_p_003 = 0;

	VL53LX_p_003 =  ip_value;

	for (i = 0; i < count; i++) {
		pbuffer[count-i-1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
		VL53LX_p_003 = VL53LX_p_003 >> 8;
	}
}

uint32_t VL53LX_i2c_decode_uint32_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{


	uint32_t   value = 0x00;

	while (count-- > 0)
		value = (value << 8) | (uint32_t)*pbuffer++;

	return value;
}


uint32_t VL53LX_i2c_decode_with_mask(
	uint16_t    count,
	uint8_t    *pbuffer,
	uint32_t    bit_mask,
	uint32_t    down_shift,
	uint32_t    offset)
{


	uint32_t   value = 0x00;


	while (count-- > 0)
		value = (value << 8) | (uint32_t)*pbuffer++;


	value =  value & bit_mask;
	if (down_shift > 0)
		value = value >> down_shift;


	value = value + offset;

	return value;
}


void VL53LX_i2c_encode_int32_t(
	int32_t     ip_value,
	uint16_t    count,
	uint8_t    *pbuffer)
{


	uint16_t   i    = 0;
	int32_t    VL53LX_p_003 = 0;

	VL53LX_p_003 =  ip_value;

	for (i = 0; i < count; i++) {
		pbuffer[count-i-1] = (uint8_t)(VL53LX_p_003 & 0x00FF);
		VL53LX_p_003 = VL53LX_p_003 >> 8;
	}
}

int32_t VL53LX_i2c_decode_int32_t(
	uint16_t    count,
	uint8_t    *pbuffer)
{


	int32_t    value = 0x00;


	if (*pbuffer >= 0x80)
		value = 0xFFFFFFFF;

	while (count-- > 0)
		value = (value << 8) | (int32_t)*pbuffer++;

	return value;
}


VL53LX_Error VL53LX_start_test(
	VL53LX_DEV    Dev,
	uint8_t       test_mode__ctrl)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	if (status == VL53LX_ERROR_NONE) {
		status = VL53LX_WrByte(
					Dev,
					VL53LX_TEST_MODE__CTRL,
					test_mode__ctrl);
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_firmware_enable_register(
	VL53LX_DEV    Dev,
	uint8_t       value)
{


	VL53LX_Error status         = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	pdev->sys_ctrl.firmware__enable = value;

	status = VL53LX_WrByte(
				Dev,
				VL53LX_FIRMWARE__ENABLE,
				pdev->sys_ctrl.firmware__enable);

	return status;
}

VL53LX_Error VL53LX_enable_firmware(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status = VL53LX_set_firmware_enable_register(Dev, 0x01);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_disable_firmware(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status = VL53LX_set_firmware_enable_register(Dev, 0x00);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_powerforce_register(
	VL53LX_DEV    Dev,
	uint8_t       value)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	pdev->sys_ctrl.power_management__go1_power_force = value;

	status = VL53LX_WrByte(
			Dev,
			VL53LX_POWER_MANAGEMENT__GO1_POWER_FORCE,
			pdev->sys_ctrl.power_management__go1_power_force);

	return status;
}


VL53LX_Error VL53LX_enable_powerforce(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status = VL53LX_set_powerforce_register(Dev, 0x01);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_disable_powerforce(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status = VL53LX_set_powerforce_register(Dev, 0x00);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_clear_interrupt(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->sys_ctrl.system__interrupt_clear = VL53LX_CLEAR_RANGE_INT;

	status = VL53LX_WrByte(
				Dev,
				VL53LX_SYSTEM__INTERRUPT_CLEAR,
				pdev->sys_ctrl.system__interrupt_clear);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_force_shadow_stream_count_to_zero(
	VL53LX_DEV    Dev)
{


	VL53LX_Error status       = VL53LX_ERROR_NONE;

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_disable_firmware(Dev);

	if (status == VL53LX_ERROR_NONE) {
		status = VL53LX_WrByte(
				Dev,
				VL53LX_SHADOW_RESULT__STREAM_COUNT,
				0x00);
	}

	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_enable_firmware(Dev);

	return status;
}


uint32_t VL53LX_calc_macro_period_us(
	uint16_t  fast_osc_frequency,
	uint8_t   VL53LX_p_005)
{


	uint32_t  pll_period_us        = 0;
	uint8_t   VL53LX_p_030   = 0;
	uint32_t  macro_period_us      = 0;

	LOG_FUNCTION_START("");



	pll_period_us = VL53LX_calc_pll_period_us(fast_osc_frequency);



	VL53LX_p_030 = VL53LX_decode_vcsel_period(VL53LX_p_005);



	macro_period_us =
			(uint32_t)VL53LX_MACRO_PERIOD_VCSEL_PERIODS *
			pll_period_us;
	macro_period_us = macro_period_us >> 6;

	macro_period_us = macro_period_us * (uint32_t)VL53LX_p_030;
	macro_period_us = macro_period_us >> 6;



	LOG_FUNCTION_END(0);

	return macro_period_us;
}


uint16_t VL53LX_calc_range_ignore_threshold(
	uint32_t central_rate,
	int16_t  x_gradient,
	int16_t  y_gradient,
	uint8_t  rate_mult)
{


	int32_t    range_ignore_thresh_int  = 0;
	uint16_t   range_ignore_thresh_kcps = 0;
	int32_t    central_rate_int         = 0;
	int16_t    x_gradient_int           = 0;
	int16_t    y_gradient_int           = 0;

	LOG_FUNCTION_START("");



	central_rate_int = ((int32_t)central_rate * (1 << 4)) / (1000);

	if (x_gradient < 0)
		x_gradient_int = x_gradient * -1;

	if (y_gradient < 0)
		y_gradient_int = y_gradient * -1;





	range_ignore_thresh_int = (8 * x_gradient_int * 4) +
			(8 * y_gradient_int * 4);



	range_ignore_thresh_int = range_ignore_thresh_int / 1000;



	range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;



	range_ignore_thresh_int = (int32_t)rate_mult * range_ignore_thresh_int;

	range_ignore_thresh_int = (range_ignore_thresh_int + (1<<4)) / (1<<5);



	if (range_ignore_thresh_int > 0xFFFF)
		range_ignore_thresh_kcps = 0xFFFF;
	else
		range_ignore_thresh_kcps = (uint16_t)range_ignore_thresh_int;



	LOG_FUNCTION_END(0);

	return range_ignore_thresh_kcps;
}


uint32_t VL53LX_calc_timeout_mclks(
	uint32_t timeout_us,
	uint32_t macro_period_us)
{


	uint32_t timeout_mclks   = 0;

	LOG_FUNCTION_START("");

	if (macro_period_us == 0)
		timeout_mclks = 0;
	else
		timeout_mclks   =
			((timeout_us << 12) + (macro_period_us>>1)) /
			macro_period_us;

	LOG_FUNCTION_END(0);

	return timeout_mclks;
}


uint16_t VL53LX_calc_encoded_timeout(
	uint32_t timeout_us,
	uint32_t macro_period_us)
{


	uint32_t timeout_mclks   = 0;
	uint16_t timeout_encoded = 0;

	LOG_FUNCTION_START("");

	timeout_mclks   =
		VL53LX_calc_timeout_mclks(timeout_us, macro_period_us);

	timeout_encoded =
		VL53LX_encode_timeout(timeout_mclks);



	LOG_FUNCTION_END(0);

	return timeout_encoded;
}


uint32_t VL53LX_calc_timeout_us(
	uint32_t timeout_mclks,
	uint32_t macro_period_us)
{


	uint32_t timeout_us     = 0;
	uint64_t tmp            = 0;

	LOG_FUNCTION_START("");

	tmp  = (uint64_t)timeout_mclks * (uint64_t)macro_period_us;
	tmp += 0x00800;
	tmp  = tmp >> 12;

	timeout_us = (uint32_t)tmp;



	LOG_FUNCTION_END(0);

	return timeout_us;
}

uint32_t VL53LX_calc_crosstalk_plane_offset_with_margin(
		uint32_t     plane_offset_kcps,
		int16_t      margin_offset_kcps)
{
	uint32_t plane_offset_with_margin = 0;
	int32_t  plane_offset_kcps_temp   = 0;

	LOG_FUNCTION_START("");

	plane_offset_kcps_temp =
		(int32_t)plane_offset_kcps +
		(int32_t)margin_offset_kcps;

	if (plane_offset_kcps_temp < 0)
		plane_offset_kcps_temp = 0;
	else
		if (plane_offset_kcps_temp > 0x3FFFF)
			plane_offset_kcps_temp = 0x3FFFF;

	plane_offset_with_margin = (uint32_t) plane_offset_kcps_temp;

	LOG_FUNCTION_END(0);

	return plane_offset_with_margin;

}

uint32_t VL53LX_calc_decoded_timeout_us(
	uint16_t timeout_encoded,
	uint32_t macro_period_us)
{


	uint32_t timeout_mclks  = 0;
	uint32_t timeout_us     = 0;

	LOG_FUNCTION_START("");

	timeout_mclks =
		VL53LX_decode_timeout(timeout_encoded);

	timeout_us    =
		VL53LX_calc_timeout_us(timeout_mclks, macro_period_us);

	LOG_FUNCTION_END(0);

	return timeout_us;
}


uint16_t VL53LX_encode_timeout(uint32_t timeout_mclks)
{


	uint16_t encoded_timeout = 0;
	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;

	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte = ls_byte >> 1;
			ms_byte++;
		}

		encoded_timeout = (ms_byte << 8)
				+ (uint16_t) (ls_byte & 0x000000FF);
	}

	return encoded_timeout;
}


uint32_t VL53LX_decode_timeout(uint16_t encoded_timeout)
{


	uint32_t timeout_macro_clks = 0;

	timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
			<< (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

	return timeout_macro_clks;
}


VL53LX_Error VL53LX_calc_timeout_register_values(
	uint32_t                 phasecal_config_timeout_us,
	uint32_t                 mm_config_timeout_us,
	uint32_t                 range_config_timeout_us,
	uint16_t                 fast_osc_frequency,
	VL53LX_general_config_t *pgeneral,
	VL53LX_timing_config_t  *ptiming)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint32_t macro_period_us    = 0;
	uint32_t timeout_mclks      = 0;
	uint16_t timeout_encoded    = 0;

	LOG_FUNCTION_START("");

	if (fast_osc_frequency == 0) {
		status = VL53LX_ERROR_DIVISION_BY_ZERO;
	} else {

		macro_period_us =
				VL53LX_calc_macro_period_us(
					fast_osc_frequency,
					ptiming->range_config__vcsel_period_a);


		timeout_mclks =
			VL53LX_calc_timeout_mclks(
				phasecal_config_timeout_us,
				macro_period_us);


		if (timeout_mclks > 0xFF)
			timeout_mclks = 0xFF;

		pgeneral->phasecal_config__timeout_macrop =
				(uint8_t)timeout_mclks;


		timeout_encoded =
			VL53LX_calc_encoded_timeout(
				mm_config_timeout_us,
				macro_period_us);

		ptiming->mm_config__timeout_macrop_a_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->mm_config__timeout_macrop_a_lo =
				(uint8_t) (timeout_encoded & 0x00FF);


		timeout_encoded =
			VL53LX_calc_encoded_timeout(
				range_config_timeout_us,
				macro_period_us);

		ptiming->range_config__timeout_macrop_a_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->range_config__timeout_macrop_a_lo =
				(uint8_t) (timeout_encoded & 0x00FF);


		macro_period_us =
				VL53LX_calc_macro_period_us(
					fast_osc_frequency,
					ptiming->range_config__vcsel_period_b);


		timeout_encoded =
				VL53LX_calc_encoded_timeout(
					mm_config_timeout_us,
					macro_period_us);

		ptiming->mm_config__timeout_macrop_b_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->mm_config__timeout_macrop_b_lo =
				(uint8_t) (timeout_encoded & 0x00FF);


		timeout_encoded = VL53LX_calc_encoded_timeout(
							range_config_timeout_us,
							macro_period_us);

		ptiming->range_config__timeout_macrop_b_hi =
				(uint8_t)((timeout_encoded & 0xFF00) >> 8);
		ptiming->range_config__timeout_macrop_b_lo =
				(uint8_t) (timeout_encoded & 0x00FF);
	}

	LOG_FUNCTION_END(0);

	return status;

}


uint8_t VL53LX_encode_vcsel_period(uint8_t VL53LX_p_030)
{


	uint8_t vcsel_period_reg = 0;

	vcsel_period_reg = (VL53LX_p_030 >> 1) - 1;

	return vcsel_period_reg;
}


uint32_t VL53LX_decode_unsigned_integer(
	uint8_t  *pbuffer,
	uint8_t   no_of_bytes)
{


	uint8_t   i = 0;
	uint32_t  decoded_value = 0;

	for (i = 0; i < no_of_bytes; i++)
		decoded_value = (decoded_value << 8) + (uint32_t)pbuffer[i];

	return decoded_value;
}


void VL53LX_encode_unsigned_integer(
	uint32_t  ip_value,
	uint8_t   no_of_bytes,
	uint8_t  *pbuffer)
{


	uint8_t   i    = 0;
	uint32_t  VL53LX_p_003 = 0;

	VL53LX_p_003 = ip_value;
	for (i = 0; i < no_of_bytes; i++) {
		pbuffer[no_of_bytes-i-1] = VL53LX_p_003 & 0x00FF;
		VL53LX_p_003 = VL53LX_p_003 >> 8;
	}
}


VL53LX_Error  VL53LX_hist_copy_and_scale_ambient_info(
	VL53LX_zone_hist_info_t       *pidata,
	VL53LX_histogram_bin_data_t   *podata)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	int64_t  evts              = 0;
	int64_t  tmpi              = 0;
	int64_t  tmpo              = 0;

	LOG_FUNCTION_START("");


	if (pidata->result__dss_actual_effective_spads == 0) {
		status = VL53LX_ERROR_DIVISION_BY_ZERO;
	} else {
		if (pidata->number_of_ambient_bins >  0 &&
			podata->number_of_ambient_bins == 0) {



			tmpo    = 1 + (int64_t)podata->total_periods_elapsed;
			tmpo   *=
			(int64_t)podata->result__dss_actual_effective_spads;

			tmpi    = 1 + (int64_t)pidata->total_periods_elapsed;
			tmpi   *=
			(int64_t)pidata->result__dss_actual_effective_spads;

			evts  = tmpo *
				(int64_t)pidata->ambient_events_sum;
			evts += (tmpi/2);


			if (tmpi != 0)
				evts = do_division_s(evts, tmpi);

			podata->ambient_events_sum = (int32_t)evts;



			podata->VL53LX_p_028 =
				podata->ambient_events_sum;
			podata->VL53LX_p_028 +=
				((int32_t)pidata->number_of_ambient_bins / 2);
			podata->VL53LX_p_028 /=
				(int32_t)pidata->number_of_ambient_bins;
		}
	}

	LOG_FUNCTION_END(0);

	return status;
}


void  VL53LX_hist_get_bin_sequence_config(
	VL53LX_DEV                     Dev,
	VL53LX_histogram_bin_data_t   *pdata)
{


	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	int32_t amb_thresh_low   = 0;
	int32_t amb_thresh_high  = 0;

	uint8_t i = 0;

	LOG_FUNCTION_START("");



	amb_thresh_low  = 1024 *
		(int32_t)pdev->hist_cfg.histogram_config__amb_thresh_low;
	amb_thresh_high = 1024 *
		(int32_t)pdev->hist_cfg.histogram_config__amb_thresh_high;



	if ((pdev->ll_state.rd_stream_count & 0x01) == 0) {

		pdata->bin_seq[5] =
		pdev->hist_cfg.histogram_config__mid_amb_even_bin_4_5 >> 4;
		pdata->bin_seq[4] =
		pdev->hist_cfg.histogram_config__mid_amb_even_bin_4_5 & 0x0F;
		pdata->bin_seq[3] =
		pdev->hist_cfg.histogram_config__mid_amb_even_bin_2_3 >> 4;
		pdata->bin_seq[2] =
		pdev->hist_cfg.histogram_config__mid_amb_even_bin_2_3 & 0x0F;
		pdata->bin_seq[1] =
		pdev->hist_cfg.histogram_config__mid_amb_even_bin_0_1 >> 4;
		pdata->bin_seq[0] =
		pdev->hist_cfg.histogram_config__mid_amb_even_bin_0_1 & 0x0F;

		if (pdata->ambient_events_sum > amb_thresh_high) {
			pdata->bin_seq[5] =
			pdev->hist_cfg.histogram_config__high_amb_even_bin_4_5
			>> 4;
			pdata->bin_seq[4] =
			pdev->hist_cfg.histogram_config__high_amb_even_bin_4_5
			& 0x0F;
			pdata->bin_seq[3] =
			pdev->hist_cfg.histogram_config__high_amb_even_bin_2_3
			>> 4;
			pdata->bin_seq[2] =
			pdev->hist_cfg.histogram_config__high_amb_even_bin_2_3
			& 0x0F;
			pdata->bin_seq[1] =
			pdev->hist_cfg.histogram_config__high_amb_even_bin_0_1
			>> 4;
			pdata->bin_seq[0] =
			pdev->hist_cfg.histogram_config__high_amb_even_bin_0_1
			& 0x0F;
		}

		if (pdata->ambient_events_sum < amb_thresh_low) {
			pdata->bin_seq[5] =
			pdev->hist_cfg.histogram_config__low_amb_even_bin_4_5
			>> 4;
			pdata->bin_seq[4] =
			pdev->hist_cfg.histogram_config__low_amb_even_bin_4_5
			& 0x0F;
			pdata->bin_seq[3] =
			pdev->hist_cfg.histogram_config__low_amb_even_bin_2_3
			>> 4;
			pdata->bin_seq[2] =
			pdev->hist_cfg.histogram_config__low_amb_even_bin_2_3
			& 0x0F;
			pdata->bin_seq[1] =
			pdev->hist_cfg.histogram_config__low_amb_even_bin_0_1
			>> 4;
			pdata->bin_seq[0] =
			pdev->hist_cfg.histogram_config__low_amb_even_bin_0_1
			& 0x0F;
		}

	} else {
		pdata->bin_seq[5] =
			pdev->hist_cfg.histogram_config__mid_amb_odd_bin_5
			& 0x0F;
		pdata->bin_seq[4] =
			pdev->hist_cfg.histogram_config__mid_amb_odd_bin_3_4
			& 0x0F;
		pdata->bin_seq[3] =
			pdev->hist_cfg.histogram_config__mid_amb_odd_bin_3_4
			>> 4;
		pdata->bin_seq[2] =
			pdev->hist_cfg.histogram_config__mid_amb_odd_bin_2 &
			0x0F;
		pdata->bin_seq[1] =
			pdev->hist_cfg.histogram_config__mid_amb_odd_bin_0_1
			>> 4;
		pdata->bin_seq[0] =
			pdev->hist_cfg.histogram_config__mid_amb_odd_bin_0_1
			& 0x0F;

		if (pdata->ambient_events_sum > amb_thresh_high) {
			pdata->bin_seq[5] =
			pdev->hist_cfg.histogram_config__high_amb_odd_bin_4_5
			>> 4;
			pdata->bin_seq[4] =
			pdev->hist_cfg.histogram_config__high_amb_odd_bin_4_5
			& 0x0F;
			pdata->bin_seq[3] =
			pdev->hist_cfg.histogram_config__high_amb_odd_bin_2_3
			>> 4;
			pdata->bin_seq[2] =
			pdev->hist_cfg.histogram_config__high_amb_odd_bin_2_3
			& 0x0F;
			pdata->bin_seq[1] =
			pdev->hist_cfg.histogram_config__high_amb_odd_bin_0_1
			>> 4;
			pdata->bin_seq[0] =
			pdev->hist_cfg.histogram_config__high_amb_odd_bin_0_1
			& 0x0F;
		}

		if (pdata->ambient_events_sum < amb_thresh_low) {
			pdata->bin_seq[5] =
			pdev->hist_cfg.histogram_config__low_amb_odd_bin_4_5
			>> 4;
			pdata->bin_seq[4] =
			pdev->hist_cfg.histogram_config__low_amb_odd_bin_4_5
			& 0x0F;
			pdata->bin_seq[3] =
			pdev->hist_cfg.histogram_config__low_amb_odd_bin_2_3
			>> 4;
			pdata->bin_seq[2] =
			pdev->hist_cfg.histogram_config__low_amb_odd_bin_2_3
			& 0x0F;
			pdata->bin_seq[1] =
			pdev->hist_cfg.histogram_config__low_amb_odd_bin_0_1
			>> 4;
			pdata->bin_seq[0] =
			pdev->hist_cfg.histogram_config__low_amb_odd_bin_0_1
			& 0x0F;
		}
	}



	for (i = 0; i < VL53LX_MAX_BIN_SEQUENCE_LENGTH; i++)
		pdata->bin_rep[i] = 1;

	LOG_FUNCTION_END(0);

}


VL53LX_Error  VL53LX_hist_phase_consistency_check(
	VL53LX_DEV                   Dev,
	VL53LX_zone_hist_info_t     *phist_prev,
	VL53LX_zone_objects_t       *prange_prev,
	VL53LX_range_results_t      *prange_curr)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
		VL53LXDevStructGetLLDriverHandle(Dev);

	uint8_t   lc = 0;
	uint8_t   p = 0;

	uint16_t  phase_delta      = 0;
	uint16_t  phase_tolerance  = 0;

	int32_t   events_delta     = 0;
	int32_t   events_tolerance = 0;


	uint8_t event_sigma;
	uint16_t event_min_spad_count;
	uint16_t min_max_tolerance;
	uint8_t pht;

	VL53LX_DeviceError  range_status = 0;

	LOG_FUNCTION_START("");

	event_sigma =
		pdev->histpostprocess.algo__consistency_check__event_sigma;
	event_min_spad_count =
	pdev->histpostprocess.algo__consistency_check__event_min_spad_count;
	min_max_tolerance =
	pdev->histpostprocess.algo__consistency_check__min_max_tolerance;


	pht = pdev->histpostprocess.algo__consistency_check__phase_tolerance;
	phase_tolerance = (uint16_t)pht;
	phase_tolerance = phase_tolerance << 8;



	if (prange_prev->rd_device_state !=
			VL53LX_DEVICESTATE_RANGING_GATHER_DATA &&
		prange_prev->rd_device_state !=
				VL53LX_DEVICESTATE_RANGING_OUTPUT_DATA)
		return status;



	if (phase_tolerance == 0)
		return status;

	for (lc = 0; lc < prange_curr->active_results; lc++) {

		if (!((prange_curr->VL53LX_p_003[lc].range_status ==
			VL53LX_DEVICEERROR_RANGECOMPLETE) ||
			(prange_curr->VL53LX_p_003[lc].range_status ==
			VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK)))
			continue;






		if (prange_prev->active_objects == 0)
			prange_curr->VL53LX_p_003[lc].range_status =
			VL53LX_DEVICEERROR_PREV_RANGE_NO_TARGETS;
		else
			prange_curr->VL53LX_p_003[lc].range_status =
				VL53LX_DEVICEERROR_PHASECONSISTENCY;





		for (p = 0; p < prange_prev->active_objects; p++) {

			if (prange_curr->VL53LX_p_003[lc].VL53LX_p_011 >
				prange_prev->VL53LX_p_003[p].VL53LX_p_011) {
				phase_delta =
				prange_curr->VL53LX_p_003[lc].VL53LX_p_011 -
				prange_prev->VL53LX_p_003[p].VL53LX_p_011;
			} else {
				phase_delta =
				prange_prev->VL53LX_p_003[p].VL53LX_p_011 -
				prange_curr->VL53LX_p_003[lc].VL53LX_p_011;
			}

			if (phase_delta < phase_tolerance) {





				if (status == VL53LX_ERROR_NONE)
					status =
					VL53LX_hist_events_consistency_check(
					event_sigma,
					event_min_spad_count,
					phist_prev,
					&(prange_prev->VL53LX_p_003[p]),
					&(prange_curr->VL53LX_p_003[lc]),
					&events_tolerance,
					&events_delta,
					&range_status);




				if (status == VL53LX_ERROR_NONE &&
					range_status ==
					VL53LX_DEVICEERROR_RANGECOMPLETE)
					status =
					VL53LX_hist_merged_pulse_check(
					min_max_tolerance,
					&(prange_curr->VL53LX_p_003[lc]),
					&range_status);

				prange_curr->VL53LX_p_003[lc].range_status =
						range_status;
			}
		}

	}

	LOG_FUNCTION_END(status);

	return status;
}



VL53LX_Error  VL53LX_hist_events_consistency_check(
	uint8_t                      event_sigma,
	uint16_t                     min_effective_spad_count,
	VL53LX_zone_hist_info_t     *phist_prev,
	VL53LX_object_data_t        *prange_prev,
	VL53LX_range_data_t         *prange_curr,
	int32_t                     *pevents_tolerance,
	int32_t                     *pevents_delta,
	VL53LX_DeviceError          *prange_status)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	int64_t   tmpp                   = 0;
	int64_t   tmpc                   = 0;
	int64_t   events_scaler          = 0;
	int64_t   events_scaler_sq       = 0;
	int64_t   c_signal_events        = 0;
	int64_t   c_sig_noise_sq         = 0;
	int64_t   c_amb_noise_sq         = 0;
	int64_t   p_amb_noise_sq         = 0;

	int32_t   p_signal_events        = 0;
	uint32_t  noise_sq_sum           = 0;



	if (event_sigma == 0) {
		*prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE;
		return status;
	}



	tmpp  = 1 + (int64_t)phist_prev->total_periods_elapsed;
	tmpp *= (int64_t)phist_prev->result__dss_actual_effective_spads;



	tmpc  = 1 + (int64_t)prange_curr->total_periods_elapsed;
	tmpc *= (int64_t)prange_curr->VL53LX_p_004;



	events_scaler  = tmpp * 4096;
	if (tmpc != 0) {
		events_scaler += (tmpc/2);
		events_scaler  = do_division_s(events_scaler, tmpc);
	}

	events_scaler_sq  = events_scaler * events_scaler;
	events_scaler_sq += 2048;
	events_scaler_sq /= 4096;



	c_signal_events  = (int64_t)prange_curr->VL53LX_p_017;
	c_signal_events -= (int64_t)prange_curr->VL53LX_p_016;
	c_signal_events *= (int64_t)events_scaler;
	c_signal_events += 2048;
	c_signal_events /= 4096;

	c_sig_noise_sq  = (int64_t)events_scaler_sq;
	c_sig_noise_sq *= (int64_t)prange_curr->VL53LX_p_017;
	c_sig_noise_sq += 2048;
	c_sig_noise_sq /= 4096;

	c_amb_noise_sq  = (int64_t)events_scaler_sq;
	c_amb_noise_sq *= (int64_t)prange_curr->VL53LX_p_016;
	c_amb_noise_sq += 2048;
	c_amb_noise_sq /= 4096;


	c_amb_noise_sq += 2;
	c_amb_noise_sq /= 4;



	p_amb_noise_sq  =
		(int64_t)prange_prev->VL53LX_p_016;


	p_amb_noise_sq += 2;
	p_amb_noise_sq /= 4;

	noise_sq_sum =
		(uint32_t)prange_prev->VL53LX_p_017 +
		(uint32_t)c_sig_noise_sq +
		(uint32_t)p_amb_noise_sq +
		(uint32_t)c_amb_noise_sq;

	*pevents_tolerance =
		(int32_t)VL53LX_isqrt(noise_sq_sum * 16);

	*pevents_tolerance *= (int32_t)event_sigma;
	*pevents_tolerance += 32;
	*pevents_tolerance /= 64;

	p_signal_events  = (int32_t)prange_prev->VL53LX_p_017;
	p_signal_events -= (int32_t)prange_prev->VL53LX_p_016;

	if ((int32_t)c_signal_events > p_signal_events)
		*pevents_delta =
			(int32_t)c_signal_events - p_signal_events;
	else
		*pevents_delta =
			p_signal_events - (int32_t)c_signal_events;

	if (*pevents_delta > *pevents_tolerance &&
		prange_curr->VL53LX_p_004 > min_effective_spad_count)
		*prange_status = VL53LX_DEVICEERROR_EVENTCONSISTENCY;
	else
		*prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE;





	return status;
}




VL53LX_Error  VL53LX_hist_merged_pulse_check(
	int16_t                      min_max_tolerance_mm,
	VL53LX_range_data_t         *pdata,
	VL53LX_DeviceError          *prange_status)
{


	VL53LX_Error  status   = VL53LX_ERROR_NONE;
	int16_t       delta_mm = 0;

	if (pdata->max_range_mm > pdata->min_range_mm)
		delta_mm =
			pdata->max_range_mm - pdata->min_range_mm;
	else
		delta_mm =
			pdata->min_range_mm - pdata->max_range_mm;

	if (min_max_tolerance_mm > 0 &&
		delta_mm > min_max_tolerance_mm)
		*prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE;
	else
		*prange_status = VL53LX_DEVICEERROR_RANGECOMPLETE;

	return status;
}




VL53LX_Error  VL53LX_hist_xmonitor_consistency_check(
	VL53LX_DEV                   Dev,
	VL53LX_zone_hist_info_t     *phist_prev,
	VL53LX_zone_objects_t       *prange_prev,
	VL53LX_range_data_t         *prange_curr)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev =
		VL53LXDevStructGetLLDriverHandle(Dev);

	int32_t   events_delta     = 0;
	int32_t   events_tolerance = 0;
	uint8_t event_sigma;
	uint16_t min_spad_count;

	event_sigma = pdev->histpostprocess.algo__crosstalk_detect_event_sigma;
	min_spad_count =
	pdev->histpostprocess.algo__consistency_check__event_min_spad_count;

	if (prange_curr->range_status == VL53LX_DEVICEERROR_RANGECOMPLETE ||
		prange_curr->range_status ==
			VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK ||
		prange_curr->range_status ==
				VL53LX_DEVICEERROR_EVENTCONSISTENCY) {

		if (prange_prev->xmonitor.range_status ==
				VL53LX_DEVICEERROR_RANGECOMPLETE ||
			prange_prev->xmonitor.range_status ==
			VL53LX_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK ||
			prange_prev->xmonitor.range_status ==
				VL53LX_DEVICEERROR_EVENTCONSISTENCY) {

			prange_curr->range_status =
					VL53LX_DEVICEERROR_RANGECOMPLETE;

			status =
				VL53LX_hist_events_consistency_check(
					event_sigma,
					min_spad_count,
					phist_prev,
					&(prange_prev->xmonitor),
					prange_curr,
					&events_tolerance,
					&events_delta,
					&(prange_curr->range_status));

		}
	}

	return status;
}




VL53LX_Error  VL53LX_hist_wrap_dmax(
	VL53LX_hist_post_process_config_t  *phistpostprocess,
	VL53LX_histogram_bin_data_t        *pcurrent,
	int16_t                            *pwrap_dmax_mm)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	uint32_t  pll_period_mm        = 0;
	uint32_t  wrap_dmax_phase      = 0;
	uint32_t  range_mm             = 0;

	LOG_FUNCTION_START("");

	*pwrap_dmax_mm = 0;


	if (pcurrent->VL53LX_p_015 != 0) {



		pll_period_mm =
			VL53LX_calc_pll_period_mm(
				pcurrent->VL53LX_p_015);



		wrap_dmax_phase =
			(uint32_t)phistpostprocess->valid_phase_high << 8;



		range_mm = wrap_dmax_phase * pll_period_mm;
		range_mm = (range_mm + (1<<14)) >> 15;

		*pwrap_dmax_mm = (int16_t)range_mm;
	}

	LOG_FUNCTION_END(status);

	return status;
}


void VL53LX_hist_combine_mm1_mm2_offsets(
	int16_t                               mm1_offset_mm,
	int16_t                               mm2_offset_mm,
	uint8_t                               encoded_mm_roi_centre,
	uint8_t                               encoded_mm_roi_size,
	uint8_t                               encoded_zone_centre,
	uint8_t                               encoded_zone_size,
	VL53LX_additional_offset_cal_data_t  *pcal_data,
	uint8_t                              *pgood_spads,
	uint16_t                              aperture_attenuation,
	int16_t                               *prange_offset_mm)
{



	uint16_t max_mm_inner_effective_spads = 0;
	uint16_t max_mm_outer_effective_spads = 0;
	uint16_t mm_inner_effective_spads     = 0;
	uint16_t mm_outer_effective_spads     = 0;

	uint32_t scaled_mm1_peak_rate_mcps    = 0;
	uint32_t scaled_mm2_peak_rate_mcps    = 0;

	int32_t tmp0 = 0;
	int32_t tmp1 = 0;



	VL53LX_calc_mm_effective_spads(
		encoded_mm_roi_centre,
		encoded_mm_roi_size,
		0xC7,
		0xFF,
		pgood_spads,
		aperture_attenuation,
		&max_mm_inner_effective_spads,
		&max_mm_outer_effective_spads);

	if ((max_mm_inner_effective_spads == 0) ||
		(max_mm_outer_effective_spads == 0))
		goto FAIL;


	VL53LX_calc_mm_effective_spads(
		encoded_mm_roi_centre,
		encoded_mm_roi_size,
		encoded_zone_centre,
		encoded_zone_size,
		pgood_spads,
		aperture_attenuation,
		&mm_inner_effective_spads,
		&mm_outer_effective_spads);



	scaled_mm1_peak_rate_mcps  =
	(uint32_t)pcal_data->result__mm_inner_peak_signal_count_rtn_mcps;
	scaled_mm1_peak_rate_mcps *= (uint32_t)mm_inner_effective_spads;
	scaled_mm1_peak_rate_mcps /= (uint32_t)max_mm_inner_effective_spads;

	scaled_mm2_peak_rate_mcps  =
	(uint32_t)pcal_data->result__mm_outer_peak_signal_count_rtn_mcps;
	scaled_mm2_peak_rate_mcps *= (uint32_t)mm_outer_effective_spads;
	scaled_mm2_peak_rate_mcps /= (uint32_t)max_mm_outer_effective_spads;



	tmp0  = ((int32_t)mm1_offset_mm * (int32_t)scaled_mm1_peak_rate_mcps);
	tmp0 += ((int32_t)mm2_offset_mm * (int32_t)scaled_mm2_peak_rate_mcps);

	tmp1 =  (int32_t)scaled_mm1_peak_rate_mcps +
			(int32_t)scaled_mm2_peak_rate_mcps;



	if (tmp1 != 0)
		tmp0 = (tmp0 * 4) / tmp1;
FAIL:
	*prange_offset_mm = (int16_t)tmp0;

}


VL53LX_Error VL53LX_hist_xtalk_extract_calc_window(
	int16_t                             target_distance_mm,
	uint16_t                            target_width_oversize,
	VL53LX_histogram_bin_data_t        *phist_bins,
	VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");


	pxtalk_data->pll_period_mm =
		VL53LX_calc_pll_period_mm(phist_bins->VL53LX_p_015);
	if (pxtalk_data->pll_period_mm == 0)
		pxtalk_data->pll_period_mm = 1;


	pxtalk_data->xtalk_width_phase =
		(int32_t)phist_bins->vcsel_width * 128;
	pxtalk_data->target_width_phase =
		pxtalk_data->xtalk_width_phase +
		(int32_t)target_width_oversize * 128;



	pxtalk_data->xtalk_start_phase =
		(int32_t)phist_bins->zero_distance_phase -
		(pxtalk_data->xtalk_width_phase / 2);
	pxtalk_data->xtalk_end_phase  =
		(int32_t)pxtalk_data->xtalk_start_phase +
		pxtalk_data->xtalk_width_phase;

	if (pxtalk_data->xtalk_start_phase < 0)
		pxtalk_data->xtalk_start_phase = 0;




	pxtalk_data->VL53LX_p_012 =
		(uint8_t)(pxtalk_data->xtalk_start_phase / 2048);


	pxtalk_data->VL53LX_p_013 =
		(uint8_t)((pxtalk_data->xtalk_end_phase + 2047) / 2048);



	pxtalk_data->target_start_phase  =
			(int32_t)target_distance_mm * 2048 * 16;
	pxtalk_data->target_start_phase +=
			((int32_t)pxtalk_data->pll_period_mm / 2);
	pxtalk_data->target_start_phase /= (int32_t)pxtalk_data->pll_period_mm;
	pxtalk_data->target_start_phase +=
			(int32_t)phist_bins->zero_distance_phase;



	pxtalk_data->target_start_phase -=
			(pxtalk_data->target_width_phase / 2);
	pxtalk_data->target_end_phase  =
		(int32_t)pxtalk_data->target_start_phase +
		pxtalk_data->target_width_phase;

	if (pxtalk_data->target_start_phase < 0)
		pxtalk_data->target_start_phase = 0;


	pxtalk_data->target_start =
		(uint8_t)(pxtalk_data->target_start_phase / 2048);


	if (pxtalk_data->VL53LX_p_013 > (pxtalk_data->target_start-1))
		pxtalk_data->VL53LX_p_013 = pxtalk_data->target_start-1;


	pxtalk_data->effective_width =
			(2048 * ((int32_t)pxtalk_data->VL53LX_p_013+1));
	pxtalk_data->effective_width -= pxtalk_data->xtalk_start_phase;


	if (pxtalk_data->effective_width > pxtalk_data->xtalk_width_phase)
		pxtalk_data->effective_width = pxtalk_data->xtalk_width_phase;

	if (pxtalk_data->effective_width < 1)
		pxtalk_data->effective_width = 1;


	pxtalk_data->event_scaler  =  pxtalk_data->xtalk_width_phase * 1000;
	pxtalk_data->event_scaler +=  (pxtalk_data->effective_width / 2);
	pxtalk_data->event_scaler /=  pxtalk_data->effective_width;


	if (pxtalk_data->event_scaler < 1000)
		pxtalk_data->event_scaler = 1000;

	if (pxtalk_data->event_scaler > 4000)
		pxtalk_data->event_scaler = 4000;


	pxtalk_data->event_scaler_sum += pxtalk_data->event_scaler;


	pxtalk_data->peak_duration_us_sum +=
		(uint32_t)phist_bins->peak_duration_us;


	pxtalk_data->effective_spad_count_sum +=
		(uint32_t)phist_bins->result__dss_actual_effective_spads;


	pxtalk_data->zero_distance_phase_sum +=
		(uint32_t)phist_bins->zero_distance_phase;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_hist_xtalk_extract_calc_event_sums(
	VL53LX_histogram_bin_data_t        *phist_bins,
	VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	uint8_t   lb = 0;
	uint8_t   i = 0;

	LOG_FUNCTION_START("");



	for (lb  = pxtalk_data->VL53LX_p_012;
		 lb <= pxtalk_data->VL53LX_p_013;
		 lb++) {


		i = (lb + phist_bins->number_of_ambient_bins +
			phist_bins->VL53LX_p_021) %
					phist_bins->VL53LX_p_021;


		pxtalk_data->signal_events_sum += phist_bins->bin_data[i];
		pxtalk_data->signal_events_sum -=
				phist_bins->VL53LX_p_028;
	}



	for (lb  = 0; lb < VL53LX_XTALK_HISTO_BINS  &&
			lb < phist_bins->VL53LX_p_021; lb++) {


		i = (lb + phist_bins->number_of_ambient_bins +
			phist_bins->VL53LX_p_021) %
					phist_bins->VL53LX_p_021;


		pxtalk_data->bin_data_sums[lb] += phist_bins->bin_data[i];
		pxtalk_data->bin_data_sums[lb] -=
				phist_bins->VL53LX_p_028;
	}

	pxtalk_data->sample_count += 1;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_hist_xtalk_extract_calc_rate_per_spad(
	VL53LX_hist_xtalk_extract_data_t   *pxtalk_data)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	uint64_t tmp64_0        = 0;
	uint64_t tmp64_1        = 0;
	uint64_t xtalk_per_spad = 0;

	LOG_FUNCTION_START("");








	tmp64_1 =
		(uint64_t)pxtalk_data->effective_spad_count_sum *
		(uint64_t)pxtalk_data->peak_duration_us_sum;

	if (pxtalk_data->signal_events_sum < 0) {
		pxtalk_data->signal_events_sum = 0;

		tmp64_0 =
		((uint64_t)pxtalk_data->sample_count *
		 (uint64_t)pxtalk_data->event_scaler_avg * 256U) << 9U;
		if (tmp64_0 > 0) {
			pxtalk_data->signal_events_sum = (int32_t)
			do_division_u((50U * tmp64_1), tmp64_0);
		}
	}
	tmp64_0 =
		((uint64_t)pxtalk_data->signal_events_sum *
		 (uint64_t)pxtalk_data->sample_count *
		 (uint64_t)pxtalk_data->event_scaler_avg * 256U) << 9U;



	if (tmp64_1 > 0U) {

		tmp64_0 = tmp64_0 + (tmp64_1 >> 1U);
		xtalk_per_spad = do_division_u(tmp64_0, tmp64_1);
	} else {
		xtalk_per_spad = (uint64_t)tmp64_0;
	}

	pxtalk_data->xtalk_rate_kcps_per_spad = (uint32_t)xtalk_per_spad;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_hist_xtalk_extract_calc_shape(
	VL53LX_hist_xtalk_extract_data_t  *pxtalk_data,
	VL53LX_xtalk_histogram_shape_t    *pxtalk_shape)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	int32_t  lb = 0;
	uint64_t total_events    = 0U;
	uint64_t tmp64_0         = 0U;
	int32_t  remaining_area  = 1024;

	LOG_FUNCTION_START("");



	pxtalk_shape->VL53LX_p_019      = 0;
	pxtalk_shape->VL53LX_p_020    = VL53LX_XTALK_HISTO_BINS;
	pxtalk_shape->VL53LX_p_021 = VL53LX_XTALK_HISTO_BINS;

	pxtalk_shape->zero_distance_phase =
		(uint16_t)pxtalk_data->zero_distance_phase_avg;
	pxtalk_shape->phasecal_result__reference_phase =
		(uint16_t)pxtalk_data->zero_distance_phase_avg + (3*2048);



	if (pxtalk_data->signal_events_sum > 0)
		total_events =
			(uint64_t)pxtalk_data->signal_events_sum *
			(uint64_t)pxtalk_data->event_scaler_avg;
	else
		total_events = 1;
	if (total_events == 0)
		total_events = 1;


	remaining_area  = 1024;
	pxtalk_data->max_shape_value = 0;

	for (lb = 0; lb < VL53LX_XTALK_HISTO_BINS; lb++) {

		if ((lb < (int32_t)pxtalk_data->VL53LX_p_012 ||
			 lb > (int32_t)pxtalk_data->VL53LX_p_013)  ||
				pxtalk_data->bin_data_sums[lb] < 0) {


			if (remaining_area > 0 && remaining_area < 1024) {
				if (remaining_area >
					pxtalk_data->max_shape_value) {
					pxtalk_shape->bin_data[lb] =
					(uint32_t)pxtalk_data->max_shape_value;
					remaining_area -=
						pxtalk_data->max_shape_value;
				} else {
					pxtalk_shape->bin_data[lb] =
						(uint32_t)remaining_area;
					remaining_area = 0;
				}
			} else {
				pxtalk_shape->bin_data[lb] = 0;
			}

		} else {

			tmp64_0 =
				(uint64_t)pxtalk_data->bin_data_sums[lb]
							* 1024U * 1000U;
			tmp64_0 += (total_events >> 1);
			tmp64_0 = do_division_u(tmp64_0, total_events);
			if (tmp64_0 > 0xFFFFU)
				tmp64_0 = 0xFFFFU;

			pxtalk_shape->bin_data[lb] = (uint32_t)tmp64_0;


			if ((int32_t)pxtalk_shape->bin_data[lb] >
				pxtalk_data->max_shape_value)
				pxtalk_data->max_shape_value =
					(int32_t)pxtalk_shape->bin_data[lb];

			remaining_area -= (int32_t)pxtalk_shape->bin_data[lb];
		}
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_hist_xtalk_shape_model(
	uint16_t                         events_per_bin,
	uint16_t                         pulse_centre,
	uint16_t                         pulse_width,
	VL53LX_xtalk_histogram_shape_t  *pxtalk_shape)
{


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	uint32_t phase_start  = 0;
	uint32_t phase_stop   = 0;
	uint32_t phase_bin    = 0;

	uint32_t bin_start    = 0;
	uint32_t bin_stop     = 0;

	uint32_t  lb           = 0;
	uint16_t  VL53LX_p_018      = 0;

	LOG_FUNCTION_START("");



	pxtalk_shape->VL53LX_p_019      = 0;
	pxtalk_shape->VL53LX_p_020    = VL53LX_XTALK_HISTO_BINS;
	pxtalk_shape->VL53LX_p_021 = VL53LX_XTALK_HISTO_BINS;

	pxtalk_shape->zero_distance_phase              = pulse_centre;
	pxtalk_shape->phasecal_result__reference_phase =
			pulse_centre + (3*2048);


	if (pulse_centre > (pulse_width >> 1))
		phase_start = (uint32_t)pulse_centre -
			((uint32_t)pulse_width >> 1);
	else
		phase_start = 0;

	phase_stop = (uint32_t)pulse_centre  +
			((uint32_t)pulse_width >> 1);


	bin_start = (phase_start / 2048);
	bin_stop  = (phase_stop  / 2048);

	for (lb = 0; lb < VL53LX_XTALK_HISTO_BINS; lb++) {
		VL53LX_p_018 = 0;


		if (lb == bin_start && lb == bin_stop) {
			VL53LX_p_018 =
			VL53LX_hist_xtalk_shape_model_interp(
				events_per_bin,
				phase_stop - phase_start);

		} else if (lb > bin_start && lb < bin_stop) {


			VL53LX_p_018 = events_per_bin;

		} else if (lb == bin_start) {


			phase_bin = (lb + 1) * 2048;
			VL53LX_p_018 =
			VL53LX_hist_xtalk_shape_model_interp(
				events_per_bin,
				(phase_bin - phase_start));

		} else if (lb == bin_stop) {


			phase_bin = lb * 2048;
			VL53LX_p_018 =
			VL53LX_hist_xtalk_shape_model_interp(
				events_per_bin,
				(phase_stop - phase_bin));
		}

		pxtalk_shape->bin_data[lb] = VL53LX_p_018;
	}

	LOG_FUNCTION_END(status);

	return status;
}


uint16_t VL53LX_hist_xtalk_shape_model_interp(
	uint16_t      events_per_bin,
	uint32_t      phase_delta)
{


	uint32_t  VL53LX_p_018  = 0;

	LOG_FUNCTION_START("");


	VL53LX_p_018  = (uint32_t)events_per_bin * phase_delta;
	VL53LX_p_018 +=  1024;
	VL53LX_p_018 /=  2048;


	if (VL53LX_p_018 > 0xFFFFU)
		VL53LX_p_018 = 0xFFFFU;

	LOG_FUNCTION_END(0);

	return (uint16_t)VL53LX_p_018;
}


void VL53LX_spad_number_to_byte_bit_index(
	uint8_t  spad_number,
	uint8_t *pbyte_index,
	uint8_t *pbit_index,
	uint8_t *pbit_mask)
{



	*pbyte_index  = spad_number >> 3;
	*pbit_index   = spad_number & 0x07;
	*pbit_mask    = 0x01 << *pbit_index;

}


void VL53LX_encode_row_col(
	uint8_t  row,
	uint8_t  col,
	uint8_t *pspad_number)
{


	if (row > 7)
		*pspad_number = 128 + (col << 3) + (15-row);
	else
		*pspad_number = ((15-col) << 3) + row;

}


void VL53LX_decode_zone_size(
	uint8_t  encoded_xy_size,
	uint8_t  *pwidth,
	uint8_t  *pheight)
{



	*pheight = encoded_xy_size >> 4;
	*pwidth  = encoded_xy_size & 0x0F;

}


void VL53LX_encode_zone_size(
	uint8_t  width,
	uint8_t  height,
	uint8_t *pencoded_xy_size)
{


	*pencoded_xy_size = (height << 4) + width;

}


void VL53LX_decode_zone_limits(
	uint8_t   encoded_xy_centre,
	uint8_t   encoded_xy_size,
	int16_t  *px_ll,
	int16_t  *py_ll,
	int16_t  *px_ur,
	int16_t  *py_ur)
{



	uint8_t x_centre = 0;
	uint8_t y_centre = 0;
	uint8_t width    = 0;
	uint8_t height   = 0;



	VL53LX_decode_row_col(
		encoded_xy_centre,
		&y_centre,
		&x_centre);

	VL53LX_decode_zone_size(
		encoded_xy_size,
		&width,
		&height);



	*px_ll = (int16_t)x_centre - ((int16_t)width + 1) / 2;
	if (*px_ll < 0)
		*px_ll = 0;

	*px_ur = *px_ll + (int16_t)width;
	if (*px_ur > (VL53LX_SPAD_ARRAY_WIDTH-1))
		*px_ur = VL53LX_SPAD_ARRAY_WIDTH-1;

	*py_ll = (int16_t)y_centre - ((int16_t)height + 1) / 2;
	if (*py_ll < 0)
		*py_ll = 0;

	*py_ur = *py_ll + (int16_t)height;
	if (*py_ur > (VL53LX_SPAD_ARRAY_HEIGHT-1))
		*py_ur = VL53LX_SPAD_ARRAY_HEIGHT-1;
}


uint8_t VL53LX_is_aperture_location(
	uint8_t row,
	uint8_t col)
{


	uint8_t is_aperture = 0;
	uint8_t mod_row     = row % 4;
	uint8_t mod_col     = col % 4;

	if (mod_row == 0 && mod_col == 2)
		is_aperture = 1;

	if (mod_row == 2 && mod_col == 0)
		is_aperture = 1;

	return is_aperture;
}


void VL53LX_calc_max_effective_spads(
	uint8_t     encoded_zone_centre,
	uint8_t     encoded_zone_size,
	uint8_t    *pgood_spads,
	uint16_t    aperture_attenuation,
	uint16_t   *pmax_effective_spads)
{



	int16_t   x         = 0;
	int16_t   y         = 0;

	int16_t   zone_x_ll = 0;
	int16_t   zone_y_ll = 0;
	int16_t   zone_x_ur = 0;
	int16_t   zone_y_ur = 0;

	uint8_t   spad_number = 0;
	uint8_t   byte_index  = 0;
	uint8_t   bit_index   = 0;
	uint8_t   bit_mask    = 0;

	uint8_t   is_aperture = 0;



	VL53LX_decode_zone_limits(
		encoded_zone_centre,
		encoded_zone_size,
		&zone_x_ll,
		&zone_y_ll,
		&zone_x_ur,
		&zone_y_ur);



	*pmax_effective_spads = 0;

	for (y = zone_y_ll; y <= zone_y_ur; y++) {
		for (x = zone_x_ll; x <= zone_x_ur; x++) {



			VL53LX_encode_row_col(
				(uint8_t)y,
				(uint8_t)x,
				&spad_number);



			VL53LX_spad_number_to_byte_bit_index(
				spad_number,
				&byte_index,
				&bit_index,
				&bit_mask);



			if ((pgood_spads[byte_index] & bit_mask) > 0) {


				is_aperture = VL53LX_is_aperture_location(
					(uint8_t)y,
					(uint8_t)x);

				if (is_aperture > 0)
					*pmax_effective_spads +=
							aperture_attenuation;
				else
					*pmax_effective_spads += 0x0100;

			}
		}
	}
}


void VL53LX_calc_mm_effective_spads(
	uint8_t     encoded_mm_roi_centre,
	uint8_t     encoded_mm_roi_size,
	uint8_t     encoded_zone_centre,
	uint8_t     encoded_zone_size,
	uint8_t    *pgood_spads,
	uint16_t    aperture_attenuation,
	uint16_t   *pmm_inner_effective_spads,
	uint16_t   *pmm_outer_effective_spads)
{



	int16_t   x         = 0;
	int16_t   y         = 0;

	int16_t   mm_x_ll   = 0;
	int16_t   mm_y_ll   = 0;
	int16_t   mm_x_ur   = 0;
	int16_t   mm_y_ur   = 0;

	int16_t   zone_x_ll = 0;
	int16_t   zone_y_ll = 0;
	int16_t   zone_x_ur = 0;
	int16_t   zone_y_ur = 0;

	uint8_t   spad_number = 0;
	uint8_t   byte_index  = 0;
	uint8_t   bit_index   = 0;
	uint8_t   bit_mask    = 0;

	uint8_t   is_aperture = 0;
	uint16_t  spad_attenuation = 0;



	VL53LX_decode_zone_limits(
		encoded_mm_roi_centre,
		encoded_mm_roi_size,
		&mm_x_ll,
		&mm_y_ll,
		&mm_x_ur,
		&mm_y_ur);

	VL53LX_decode_zone_limits(
		encoded_zone_centre,
		encoded_zone_size,
		&zone_x_ll,
		&zone_y_ll,
		&zone_x_ur,
		&zone_y_ur);



	*pmm_inner_effective_spads = 0;
	*pmm_outer_effective_spads = 0;

	for (y = zone_y_ll; y <= zone_y_ur; y++) {
		for (x = zone_x_ll; x <= zone_x_ur; x++) {



			VL53LX_encode_row_col(
				(uint8_t)y,
				(uint8_t)x,
				&spad_number);



			VL53LX_spad_number_to_byte_bit_index(
				spad_number,
				&byte_index,
				&bit_index,
				&bit_mask);



			if ((pgood_spads[byte_index] & bit_mask) > 0) {


				is_aperture = VL53LX_is_aperture_location(
					(uint8_t)y,
					(uint8_t)x);

				if (is_aperture > 0)
					spad_attenuation = aperture_attenuation;
				else
					spad_attenuation = 0x0100;



				if (x >= mm_x_ll && x <= mm_x_ur &&
					y >= mm_y_ll && y <= mm_y_ur)
					*pmm_inner_effective_spads +=
						spad_attenuation;
				else
					*pmm_outer_effective_spads +=
						spad_attenuation;
			}
		}
	}
}


void VL53LX_hist_copy_results_to_sys_and_core(
	VL53LX_histogram_bin_data_t      *pbins,
	VL53LX_range_results_t           *phist,
	VL53LX_system_results_t          *psys,
	VL53LX_core_results_t            *pcore)
{


	uint8_t  i = 0;

	VL53LX_range_data_t  *pdata;

	LOG_FUNCTION_START("");



	VL53LX_init_system_results(psys);



	psys->result__interrupt_status = pbins->result__interrupt_status;
	psys->result__range_status     = phist->active_results;
	psys->result__report_status    = pbins->result__report_status;
	psys->result__stream_count     = pbins->result__stream_count;

	pdata = &(phist->VL53LX_p_003[0]);

	for (i = 0; i < phist->active_results; i++) {

		switch (i) {
		case 0:
			psys->result__dss_actual_effective_spads_sd0 =
					pdata->VL53LX_p_004;
			psys->result__peak_signal_count_rate_mcps_sd0 =
					pdata->peak_signal_count_rate_mcps;
			psys->result__avg_signal_count_rate_mcps_sd0 =
					pdata->avg_signal_count_rate_mcps;
			psys->result__ambient_count_rate_mcps_sd0 =
					pdata->ambient_count_rate_mcps;

			psys->result__sigma_sd0 = pdata->VL53LX_p_002;
			psys->result__phase_sd0 = pdata->VL53LX_p_011;

			psys->result__final_crosstalk_corrected_range_mm_sd0 =
					(uint16_t)pdata->median_range_mm;

			psys->result__phase_sd1  = pdata->zero_distance_phase;

			pcore->result_core__ranging_total_events_sd0 =
					pdata->VL53LX_p_017;
			pcore->result_core__signal_total_events_sd0 =
					pdata->VL53LX_p_010;
			pcore->result_core__total_periods_elapsed_sd0 =
					pdata->total_periods_elapsed;
			pcore->result_core__ambient_window_events_sd0 =
					pdata->VL53LX_p_016;

			break;
		case 1:
			psys->result__dss_actual_effective_spads_sd1 =
				pdata->VL53LX_p_004;
			psys->result__peak_signal_count_rate_mcps_sd1 =
				pdata->peak_signal_count_rate_mcps;
			psys->result__ambient_count_rate_mcps_sd1 =
				pdata->ambient_count_rate_mcps;

			psys->result__sigma_sd1 = pdata->VL53LX_p_002;
			psys->result__phase_sd1 = pdata->VL53LX_p_011;

			psys->result__final_crosstalk_corrected_range_mm_sd1 =
				(uint16_t)pdata->median_range_mm;

			pcore->result_core__ranging_total_events_sd1 =
				pdata->VL53LX_p_017;
			pcore->result_core__signal_total_events_sd1 =
				pdata->VL53LX_p_010;
			pcore->result_core__total_periods_elapsed_sd1 =
				pdata->total_periods_elapsed;
			pcore->result_core__ambient_window_events_sd1 =
				pdata->VL53LX_p_016;
			break;
		}

		pdata++;
	}

	LOG_FUNCTION_END(0);

}


VL53LX_Error VL53LX_sum_histogram_data(
		VL53LX_histogram_bin_data_t *phist_input,
		VL53LX_histogram_bin_data_t *phist_output)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t i = 0;
	uint8_t smallest_bin_num = 0;

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE) {
		if (phist_output->VL53LX_p_021 >=
				phist_input->VL53LX_p_021)
			smallest_bin_num = phist_input->VL53LX_p_021;
		else
			smallest_bin_num = phist_output->VL53LX_p_021;
	}





	if (status == VL53LX_ERROR_NONE)
		for (i = 0; i < smallest_bin_num; i++)

			phist_output->bin_data[i] += phist_input->bin_data[i];

	if (status == VL53LX_ERROR_NONE)
		phist_output->VL53LX_p_028 +=
			phist_input->VL53LX_p_028;


	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_avg_histogram_data(
			uint8_t no_of_samples,
			VL53LX_histogram_bin_data_t *phist_sum,
			VL53LX_histogram_bin_data_t *phist_avg)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	uint8_t i = 0;

	LOG_FUNCTION_START("");





	if (status == VL53LX_ERROR_NONE) {
		for (i = 0; i < phist_sum->VL53LX_p_021; i++) {



			if (no_of_samples > 0)
				phist_avg->bin_data[i] =
					phist_sum->bin_data[i] /
					(int32_t)no_of_samples;
			else
				phist_avg->bin_data[i] = phist_sum->bin_data[i];
		}
	}

	if (status == VL53LX_ERROR_NONE) {
		if (no_of_samples > 0)
			phist_avg->VL53LX_p_028 =
				phist_sum->VL53LX_p_028 /
					(int32_t)no_of_samples;
		else
			phist_avg->VL53LX_p_028 =
					phist_sum->VL53LX_p_028;
	}

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_save_cfg_data(
	VL53LX_DEV  Dev)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t  *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t  *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);

	VL53LX_zone_private_dyn_cfg_t *pzone_dyn_cfg;
	VL53LX_dynamic_config_t       *pdynamic = &(pdev->dyn_cfg);

	LOG_FUNCTION_START("");

	pzone_dyn_cfg =
		&(pres->zone_dyn_cfgs.VL53LX_p_003[pdev->ll_state.cfg_zone_id]);

	pzone_dyn_cfg->expected_stream_count =
			pdev->ll_state.cfg_stream_count;

	pzone_dyn_cfg->expected_gph_id =
			pdev->ll_state.cfg_gph_id;

	pzone_dyn_cfg->roi_config__user_roi_centre_spad =
		pdynamic->roi_config__user_roi_centre_spad;

	pzone_dyn_cfg->roi_config__user_roi_requested_global_xy_size =
		pdynamic->roi_config__user_roi_requested_global_xy_size;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_dynamic_zone_update(
	VL53LX_DEV  Dev,
	VL53LX_range_results_t *presults)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t  *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t  *pres =
			VL53LXDevStructGetLLResultsHandle(Dev);
	VL53LX_zone_private_dyn_cfgs_t *pZ = &(pres->zone_dyn_cfgs);

	uint8_t   zone_id = pdev->ll_state.rd_zone_id;
	uint8_t   i;
	uint16_t  max_total_rate_per_spads;
	uint16_t  target_rate =
		pdev->stat_cfg.dss_config__target_total_rate_mcps;
	uint32_t  temp = 0xFFFF;
#ifdef VL53LX_LOG_ENABLE
	uint16_t eff_spad_cnt =
		pZ->VL53LX_p_003[zone_id].dss_requested_effective_spad_count;
#endif

	LOG_FUNCTION_START("");

	pZ->VL53LX_p_003[zone_id].dss_requested_effective_spad_count = 0;

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"    DYNZONEUPDATE: peak signal count rate mcps:");

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"%u actual effective spads: %u\n",
		presults->VL53LX_p_003[0].peak_signal_count_rate_mcps,
		presults->VL53LX_p_003[0].VL53LX_p_004);

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"    DYNZONEUPDATE: active results: %u\n",
		presults->active_results);

	max_total_rate_per_spads =
		presults->VL53LX_p_003[0].total_rate_per_spad_mcps;

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"    DYNZONEUPDATE: max total rate per spad at start: %u\n",
		max_total_rate_per_spads);

	for (i = 1; i < presults->active_results; i++) {
		trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"    DYNZONEUPDATE: zone total rate per spad: zone_id: %u,",
		i);

		trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"total rate per spad: %u\n",
		presults->VL53LX_p_003[i].total_rate_per_spad_mcps);

		if (presults->VL53LX_p_003[i].total_rate_per_spad_mcps >
			max_total_rate_per_spads)
			max_total_rate_per_spads =
			presults->VL53LX_p_003[i].total_rate_per_spad_mcps;

	}

	if (max_total_rate_per_spads == 0) {

		temp = 0xFFFF;
	} else {

		temp = target_rate << 14;
		trace_print(VL53LX_TRACE_LEVEL_DEBUG,
			"    DYNZONEUPDATE: 1: temp: %u\n",
			temp);


		temp = temp / max_total_rate_per_spads;

		trace_print(VL53LX_TRACE_LEVEL_DEBUG,
			"    DYNZONEUPDATE: 2: temp: %u\n",
			temp);


		if (temp > 0xFFFF)
			temp = 0xFFFF;

		trace_print(VL53LX_TRACE_LEVEL_DEBUG,
			"    DYNZONEUPDATE: 3: temp: %u\n",
			temp);
	}

	pZ->VL53LX_p_003[zone_id].dss_requested_effective_spad_count =
			(uint16_t)temp;

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"    DYNZONEUPDATE: zone_id: %u, target_rate: %u,",
		zone_id,
		target_rate);

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"max_total_rate_per_spads: %u, requested_spads: %u\n",
		max_total_rate_per_spads,
		eff_spad_cnt);

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_multizone_hist_bins_update(
	VL53LX_DEV  Dev)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_ll_driver_state_t *pstate = &(pdev->ll_state);
	VL53LX_zone_config_t *pzone_cfg = &(pdev->zone_cfg);
	VL53LX_histogram_config_t *phist_cfg = &(pdev->hist_cfg);
	VL53LX_histogram_config_t *pmulti_hist =
			&(pzone_cfg->multizone_hist_cfg);

	uint8_t   next_range_is_odd_timing = (pstate->cfg_stream_count) % 2;

	LOG_FUNCTION_START("");


	if (pzone_cfg->bin_config[pdev->ll_state.cfg_zone_id] ==
		VL53LX_ZONECONFIG_BINCONFIG__LOWAMB) {
		if (!next_range_is_odd_timing) {
			trace_print (VL53LX_TRACE_LEVEL_DEBUG,
			"   HISTBINCONFIGUPDATE: Setting LOWAMB EVEN timing\n");
			phist_cfg->histogram_config__low_amb_even_bin_0_1  =
			pmulti_hist->histogram_config__low_amb_even_bin_0_1;
			phist_cfg->histogram_config__low_amb_even_bin_2_3  =
			pmulti_hist->histogram_config__low_amb_even_bin_2_3;
			phist_cfg->histogram_config__low_amb_even_bin_4_5  =
			pmulti_hist->histogram_config__low_amb_even_bin_4_5;
		}

		if (next_range_is_odd_timing) {
			trace_print (VL53LX_TRACE_LEVEL_DEBUG,
			"    HISTBINCONFIGUPDATE: Setting LOWAMB ODD timing\n");
			phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
			pmulti_hist->histogram_config__low_amb_even_bin_0_1;
			phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
			pmulti_hist->histogram_config__low_amb_even_bin_2_3;
			phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
			pmulti_hist->histogram_config__low_amb_even_bin_4_5;
		}
	} else if (pzone_cfg->bin_config[pdev->ll_state.cfg_zone_id] ==
		VL53LX_ZONECONFIG_BINCONFIG__MIDAMB) {
		trace_print (VL53LX_TRACE_LEVEL_DEBUG,
			"    HISTBINCONFIGUPDATE: Setting MIDAMB timing\n");
		if (!next_range_is_odd_timing) {
			trace_print(VL53LX_TRACE_LEVEL_DEBUG,
			"   HISTBINCONFIGUPDATE: Setting MIDAMB EVEN timing\n");
			phist_cfg->histogram_config__low_amb_even_bin_0_1  =
			pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
			phist_cfg->histogram_config__low_amb_even_bin_2_3  =
			pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
			phist_cfg->histogram_config__low_amb_even_bin_4_5  =
			pmulti_hist->histogram_config__mid_amb_even_bin_4_5;
		}

		if (next_range_is_odd_timing) {
			trace_print (VL53LX_TRACE_LEVEL_DEBUG,
			"    HISTBINCONFIGUPDATE: Setting MIDAMB ODD timing\n");
			phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
			pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
			phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
			pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
			phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
			pmulti_hist->histogram_config__mid_amb_even_bin_4_5;
		}
	} else if (pzone_cfg->bin_config[pdev->ll_state.cfg_zone_id] ==
			VL53LX_ZONECONFIG_BINCONFIG__HIGHAMB) {
		if (!next_range_is_odd_timing) {
			trace_print (VL53LX_TRACE_LEVEL_DEBUG,
			"    HISTBINCONFIGUPDATE: Setting HIGHAMB EVEN timing\n"
					);
			phist_cfg->histogram_config__low_amb_even_bin_0_1  =
			pmulti_hist->histogram_config__high_amb_even_bin_0_1;
			phist_cfg->histogram_config__low_amb_even_bin_2_3  =
			pmulti_hist->histogram_config__high_amb_even_bin_2_3;
			phist_cfg->histogram_config__low_amb_even_bin_4_5  =
			pmulti_hist->histogram_config__high_amb_even_bin_4_5;
		}

		if (next_range_is_odd_timing) {
			trace_print (VL53LX_TRACE_LEVEL_DEBUG,
			"   HISTBINCONFIGUPDATE: Setting HIGHAMB ODD timing\n");
			phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
			pmulti_hist->histogram_config__high_amb_even_bin_0_1;
			phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
			pmulti_hist->histogram_config__high_amb_even_bin_2_3;
			phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
			pmulti_hist->histogram_config__high_amb_even_bin_4_5;
		}
	}



	if (status == VL53LX_ERROR_NONE) {
		VL53LX_copy_hist_bins_to_static_cfg(
			phist_cfg,
			&(pdev->stat_cfg),
			&(pdev->tim_cfg));
	}

	LOG_FUNCTION_END(status);

	return status;
}



VL53LX_Error VL53LX_update_internal_stream_counters(
	VL53LX_DEV  Dev,
	uint8_t     external_stream_count,
	uint8_t    *pinternal_stream_count,
	uint8_t    *pinternal_stream_count_val)
{

	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t stream_divider;

	VL53LX_LLDriverData_t  *pdev =
			VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	stream_divider = pdev->gen_cfg.global_config__stream_divider;

	if (stream_divider == 0) {


		*pinternal_stream_count = external_stream_count;

	} else if (*pinternal_stream_count_val == (stream_divider-1)) {


		if (*pinternal_stream_count == 0xFF)
			*pinternal_stream_count = 0x80;
		else
			*pinternal_stream_count = *pinternal_stream_count + 1;


		*pinternal_stream_count_val = 0;

	} else {


		*pinternal_stream_count_val = *pinternal_stream_count_val + 1;
	}

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"UPDINTSTREAMCOUNT   internal_steam_count:  %d,",
		*pinternal_stream_count);

	trace_print(VL53LX_TRACE_LEVEL_DEBUG,
		"internal_stream_count_val: %d, divider: %d\n",
		*pinternal_stream_count_val,
		stream_divider);

	LOG_FUNCTION_END(status);

	return status;
}



VL53LX_Error VL53LX_set_histogram_multizone_initial_bin_config(
	VL53LX_zone_config_t		*pzone_cfg,
	VL53LX_histogram_config_t	*phist_cfg,
	VL53LX_histogram_config_t	*pmulti_hist)
{
	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");


	if (pzone_cfg->bin_config[0] ==
			VL53LX_ZONECONFIG_BINCONFIG__LOWAMB) {
		phist_cfg->histogram_config__low_amb_even_bin_0_1  =
			pmulti_hist->histogram_config__low_amb_even_bin_0_1;
		phist_cfg->histogram_config__low_amb_even_bin_2_3  =
			pmulti_hist->histogram_config__low_amb_even_bin_2_3;
		phist_cfg->histogram_config__low_amb_even_bin_4_5  =
			pmulti_hist->histogram_config__low_amb_even_bin_4_5;

		phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
			pmulti_hist->histogram_config__low_amb_even_bin_0_1;
		phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
			pmulti_hist->histogram_config__low_amb_even_bin_2_3;
		phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
			pmulti_hist->histogram_config__low_amb_even_bin_4_5;
	} else if (pzone_cfg->bin_config[0] ==
			VL53LX_ZONECONFIG_BINCONFIG__MIDAMB) {
		phist_cfg->histogram_config__low_amb_even_bin_0_1  =
			pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
		phist_cfg->histogram_config__low_amb_even_bin_2_3  =
			pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
		phist_cfg->histogram_config__low_amb_even_bin_4_5  =
			pmulti_hist->histogram_config__mid_amb_even_bin_4_5;

		phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
			pmulti_hist->histogram_config__mid_amb_even_bin_0_1;
		phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
			pmulti_hist->histogram_config__mid_amb_even_bin_2_3;
		phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
			pmulti_hist->histogram_config__mid_amb_even_bin_4_5;
	} else if (pzone_cfg->bin_config[0] ==
			VL53LX_ZONECONFIG_BINCONFIG__HIGHAMB) {
		phist_cfg->histogram_config__low_amb_even_bin_0_1  =
			pmulti_hist->histogram_config__high_amb_even_bin_0_1;
		phist_cfg->histogram_config__low_amb_even_bin_2_3  =
			pmulti_hist->histogram_config__high_amb_even_bin_2_3;
		phist_cfg->histogram_config__low_amb_even_bin_4_5  =
			pmulti_hist->histogram_config__high_amb_even_bin_4_5;
		phist_cfg->histogram_config__low_amb_odd_bin_0_1  =
			pmulti_hist->histogram_config__high_amb_even_bin_0_1;
		phist_cfg->histogram_config__low_amb_odd_bin_2_3  =
			pmulti_hist->histogram_config__high_amb_even_bin_2_3;
		phist_cfg->histogram_config__low_amb_odd_bin_4_5  =
			pmulti_hist->histogram_config__high_amb_even_bin_4_5;
	}

	LOG_FUNCTION_END(status);
	return status;
}



uint8_t	VL53LX_encode_GPIO_interrupt_config(
	VL53LX_GPIO_interrupt_config_t	*pintconf)
{
	uint8_t system__interrupt_config;

	system__interrupt_config = pintconf->intr_mode_distance;
	system__interrupt_config |= ((pintconf->intr_mode_rate) << 2);
	system__interrupt_config |= ((pintconf->intr_new_measure_ready) << 5);
	system__interrupt_config |= ((pintconf->intr_no_target) << 6);
	system__interrupt_config |= ((pintconf->intr_combined_mode) << 7);

	return system__interrupt_config;
}



VL53LX_GPIO_interrupt_config_t VL53LX_decode_GPIO_interrupt_config(
	uint8_t		system__interrupt_config)
{
	VL53LX_GPIO_interrupt_config_t	intconf;

	intconf.intr_mode_distance = system__interrupt_config & 0x03;
	intconf.intr_mode_rate = (system__interrupt_config >> 2) & 0x03;
	intconf.intr_new_measure_ready = (system__interrupt_config >> 5) & 0x01;
	intconf.intr_no_target = (system__interrupt_config >> 6) & 0x01;
	intconf.intr_combined_mode = (system__interrupt_config >> 7) & 0x01;


	intconf.threshold_rate_low = 0;
	intconf.threshold_rate_high = 0;
	intconf.threshold_distance_low = 0;
	intconf.threshold_distance_high = 0;

	return intconf;
}



VL53LX_Error VL53LX_set_GPIO_distance_threshold(
	VL53LX_DEV                      Dev,
	uint16_t			threshold_high,
	uint16_t			threshold_low)
{
	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->dyn_cfg.system__thresh_high = threshold_high;
	pdev->dyn_cfg.system__thresh_low = threshold_low;

	LOG_FUNCTION_END(status);
	return status;
}



VL53LX_Error VL53LX_set_GPIO_rate_threshold(
	VL53LX_DEV                      Dev,
	uint16_t			threshold_high,
	uint16_t			threshold_low)
{
	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->gen_cfg.system__thresh_rate_high = threshold_high;
	pdev->gen_cfg.system__thresh_rate_low = threshold_low;

	LOG_FUNCTION_END(status);
	return status;
}



VL53LX_Error VL53LX_set_GPIO_thresholds_from_struct(
	VL53LX_DEV                      Dev,
	VL53LX_GPIO_interrupt_config_t *pintconf)
{
	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");

	status = VL53LX_set_GPIO_distance_threshold(
			Dev,
			pintconf->threshold_distance_high,
			pintconf->threshold_distance_low);

	if (status == VL53LX_ERROR_NONE) {
		status =
			VL53LX_set_GPIO_rate_threshold(
				Dev,
				pintconf->threshold_rate_high,
				pintconf->threshold_rate_low);
	}

	LOG_FUNCTION_END(status);
	return status;
}


VL53LX_Error VL53LX_set_ref_spad_char_config(
	VL53LX_DEV    Dev,
	uint8_t       vcsel_period_a,
	uint32_t      phasecal_timeout_us,
	uint16_t      total_rate_target_mcps,
	uint16_t      max_count_rate_rtn_limit_mcps,
	uint16_t      min_count_rate_rtn_limit_mcps,
	uint16_t      fast_osc_frequency)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	uint8_t buffer[2];

	uint32_t macro_period_us = 0;
	uint32_t timeout_mclks   = 0;

	LOG_FUNCTION_START("");


	macro_period_us =
		VL53LX_calc_macro_period_us(
			fast_osc_frequency,
			vcsel_period_a);
	if (macro_period_us == 0)
		macro_period_us = 1;


	timeout_mclks = phasecal_timeout_us << 12;
	timeout_mclks = timeout_mclks + (macro_period_us>>1);
	timeout_mclks = timeout_mclks / macro_period_us;

	if (timeout_mclks > 0xFF)
		pdev->gen_cfg.phasecal_config__timeout_macrop = 0xFF;
	else
		pdev->gen_cfg.phasecal_config__timeout_macrop =
				(uint8_t)timeout_mclks;

	pdev->tim_cfg.range_config__vcsel_period_a = vcsel_period_a;



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrByte(
				Dev,
				VL53LX_PHASECAL_CONFIG__TIMEOUT_MACROP,
				pdev->gen_cfg.phasecal_config__timeout_macrop);

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrByte(
				Dev,
				VL53LX_RANGE_CONFIG__VCSEL_PERIOD_A,
				pdev->tim_cfg.range_config__vcsel_period_a);



	buffer[0] = pdev->tim_cfg.range_config__vcsel_period_a;
	buffer[1] = pdev->tim_cfg.range_config__vcsel_period_a;

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WriteMulti(
				Dev,
				VL53LX_SD_CONFIG__WOI_SD0,
				buffer,
				2);



	pdev->customer.ref_spad_char__total_rate_target_mcps =
			total_rate_target_mcps;

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrWord(
				Dev,
				VL53LX_REF_SPAD_CHAR__TOTAL_RATE_TARGET_MCPS,
				total_rate_target_mcps);

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrWord(
				Dev,
				VL53LX_RANGE_CONFIG__SIGMA_THRESH,
				max_count_rate_rtn_limit_mcps);

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrWord(
			Dev,
			VL53LX_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS,
			min_count_rate_rtn_limit_mcps);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_set_ssc_config(
	VL53LX_DEV            Dev,
	VL53LX_ssc_config_t  *pssc_cfg,
	uint16_t              fast_osc_frequency)
{


	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t buffer[5];

	uint32_t macro_period_us = 0;
	uint16_t timeout_encoded = 0;

	LOG_FUNCTION_START("");


	macro_period_us =
		VL53LX_calc_macro_period_us(
			fast_osc_frequency,
			pssc_cfg->VL53LX_p_005);


	timeout_encoded =
		VL53LX_calc_encoded_timeout(
			pssc_cfg->timeout_us,
			macro_period_us);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrByte(
				Dev,
				VL53LX_CAL_CONFIG__VCSEL_START,
				pssc_cfg->vcsel_start);

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrByte(
				Dev,
				VL53LX_GLOBAL_CONFIG__VCSEL_WIDTH,
				pssc_cfg->vcsel_width);



	buffer[0] = (uint8_t)((timeout_encoded &  0x0000FF00) >> 8);
	buffer[1] = (uint8_t) (timeout_encoded &  0x000000FF);
	buffer[2] = pssc_cfg->VL53LX_p_005;
	buffer[3] = (uint8_t)((pssc_cfg->rate_limit_mcps &  0x0000FF00) >> 8);
	buffer[4] = (uint8_t) (pssc_cfg->rate_limit_mcps &  0x000000FF);

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WriteMulti(
				Dev,
				VL53LX_RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
				buffer,
				5);



	buffer[0] = pssc_cfg->VL53LX_p_005;
	buffer[1] = pssc_cfg->VL53LX_p_005;

	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WriteMulti(
				Dev,
				VL53LX_SD_CONFIG__WOI_SD0,
				buffer,
				2);


	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_WrByte(
				Dev,
				VL53LX_NVM_BIST__CTRL,
				pssc_cfg->array_select);

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_get_spad_rate_data(
	VL53LX_DEV                Dev,
	VL53LX_spad_rate_data_t  *pspad_rates)
{



	VL53LX_Error status = VL53LX_ERROR_NONE;
	int               i = 0;

	uint8_t  VL53LX_p_003[512];
	uint8_t *pdata = &VL53LX_p_003[0];

	LOG_FUNCTION_START("");



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_disable_firmware(Dev);



	if (status == VL53LX_ERROR_NONE)
		status =
			VL53LX_ReadMulti(
				Dev,
				VL53LX_PRIVATE__PATCH_BASE_ADDR_RSLV,
				pdata,
				512);


	pdata = &VL53LX_p_003[0];
	for (i = 0; i < VL53LX_NO_OF_SPAD_ENABLES; i++) {
		pspad_rates->rate_data[i] =
			(uint16_t)VL53LX_decode_unsigned_integer(pdata, 2);
		pdata += 2;
	}



	pspad_rates->VL53LX_p_020     = VL53LX_NO_OF_SPAD_ENABLES;
	pspad_rates->no_of_values    = VL53LX_NO_OF_SPAD_ENABLES;
	pspad_rates->fractional_bits = 15;



	if (status == VL53LX_ERROR_NONE)
		status = VL53LX_enable_firmware(Dev);

	LOG_FUNCTION_END(status);

	return status;
}



VL53LX_Error VL53LX_dynamic_xtalk_correction_calc_required_samples(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);
	VL53LX_smudge_corrector_config_t *pconfig =
				&(pdev->VL53LX_LLDriverCommonData->smudge_correct_config);
	VL53LX_smudge_corrector_internals_t *pint =
				&(pdev->smudge_corrector_internals);

	VL53LX_range_results_t *presults = &(pres->range_results);
	VL53LX_range_data_t *pxmonitor = &(presults->xmonitor);

	uint32_t peak_duration_us = pxmonitor->peak_duration_us;

	uint64_t temp64a;
	uint64_t temp64z;

	LOG_FUNCTION_START("");

	temp64a = pxmonitor->VL53LX_p_017 +
		pxmonitor->VL53LX_p_016;
	if (peak_duration_us == 0)
		peak_duration_us = 1000;
	temp64a = do_division_u((temp64a * 1000), peak_duration_us);
	temp64a = do_division_u((temp64a * 1000), peak_duration_us);

	temp64z = pconfig->noise_margin * pxmonitor->VL53LX_p_004;
	if (temp64z == 0)
		temp64z = 1;
	temp64a = temp64a * 1000 * 256;
	temp64a = do_division_u(temp64a, temp64z);
	temp64a = temp64a * 1000 * 256;
	temp64a = do_division_u(temp64a, temp64z);

	pint->required_samples = (uint32_t)temp64a;


	if (pint->required_samples < 2)
		pint->required_samples = 2;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
	VL53LX_DEV				Dev,
	uint32_t				xtalk_offset_out,
	VL53LX_smudge_corrector_config_t	*pconfig,
	VL53LX_smudge_corrector_data_t		*pout,
	uint8_t					add_smudge,
	uint8_t					soft_update
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	int16_t  x_gradient_scaler;
	int16_t  y_gradient_scaler;
	uint32_t orig_xtalk_offset;
	int16_t  orig_x_gradient;
	int16_t  orig_y_gradient;
	uint8_t  histo_merge_nb;
	uint8_t  i;
	int32_t  itemp32;
	uint32_t SmudgeFactor;
	VL53LX_xtalk_config_t  *pX = &(pdev->VL53LX_LLDriverCommonData->xtalk_cfg);
	VL53LX_xtalk_calibration_results_t  *pC = &(pdev->xtalk_cal);
	uint32_t *pcpo;
	uint32_t max, nXtalk, cXtalk;
	uint32_t incXtalk, cval;

	LOG_FUNCTION_START("");


	if (add_smudge == 1) {
		pout->algo__crosstalk_compensation_plane_offset_kcps =
			(uint32_t)xtalk_offset_out +
			(uint32_t)pconfig->smudge_margin;
	} else {
		pout->algo__crosstalk_compensation_plane_offset_kcps =
			(uint32_t)xtalk_offset_out;
	}


	orig_xtalk_offset =
	pX->nvm_default__crosstalk_compensation_plane_offset_kcps;

	orig_x_gradient =
		pX->nvm_default__crosstalk_compensation_x_plane_gradient_kcps;

	orig_y_gradient =
		pX->nvm_default__crosstalk_compensation_y_plane_gradient_kcps;

	if (((pconfig->user_scaler_set == 0) ||
		(pconfig->scaler_calc_method == 1)) &&
		(pC->algo__crosstalk_compensation_plane_offset_kcps != 0)) {

		VL53LX_compute_histo_merge_nb(Dev, &histo_merge_nb);

		if (histo_merge_nb == 0)
			histo_merge_nb = 1;
		if (pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge != 1)
			orig_xtalk_offset =
			pC->algo__crosstalk_compensation_plane_offset_kcps;
		else
			orig_xtalk_offset =
			pC->algo__xtalk_cpo_HistoMerge_kcps[histo_merge_nb-1];

		orig_x_gradient =
			pC->algo__crosstalk_compensation_x_plane_gradient_kcps;

		orig_y_gradient =
			pC->algo__crosstalk_compensation_y_plane_gradient_kcps;
	}


	if ((pconfig->user_scaler_set == 0) && (orig_x_gradient == 0))
		pout->gradient_zero_flag |= 0x01;

	if ((pconfig->user_scaler_set == 0) && (orig_y_gradient == 0))
		pout->gradient_zero_flag |= 0x02;



	if (orig_xtalk_offset == 0)
		orig_xtalk_offset = 1;



	if (pconfig->user_scaler_set == 1) {
		x_gradient_scaler = pconfig->x_gradient_scaler;
		y_gradient_scaler = pconfig->y_gradient_scaler;
	} else {

		x_gradient_scaler = (int16_t)do_division_s(
				(((int32_t)orig_x_gradient) << 6),
				orig_xtalk_offset);
		pconfig->x_gradient_scaler = x_gradient_scaler;
		y_gradient_scaler = (int16_t)do_division_s(
				(((int32_t)orig_y_gradient) << 6),
				orig_xtalk_offset);
		pconfig->y_gradient_scaler = y_gradient_scaler;
	}



	if (pconfig->scaler_calc_method == 0) {


		itemp32 = (int32_t)(
			pout->algo__crosstalk_compensation_plane_offset_kcps *
				x_gradient_scaler);
		itemp32 = itemp32 >> 6;
		if (itemp32 > 0xFFFF)
			itemp32 = 0xFFFF;

		pout->algo__crosstalk_compensation_x_plane_gradient_kcps =
			(int16_t)itemp32;

		itemp32 = (int32_t)(
			pout->algo__crosstalk_compensation_plane_offset_kcps *
				y_gradient_scaler);
		itemp32 = itemp32 >> 6;
		if (itemp32 > 0xFFFF)
			itemp32 = 0xFFFF;

		pout->algo__crosstalk_compensation_y_plane_gradient_kcps =
			(int16_t)itemp32;
	} else if (pconfig->scaler_calc_method == 1) {


		itemp32 = (int32_t)(orig_xtalk_offset -
			pout->algo__crosstalk_compensation_plane_offset_kcps);
		itemp32 = (int32_t)(do_division_s(itemp32, 16));
		itemp32 = itemp32 << 2;
		itemp32 = itemp32 + (int32_t)(orig_x_gradient);
		if (itemp32 > 0xFFFF)
			itemp32 = 0xFFFF;

		pout->algo__crosstalk_compensation_x_plane_gradient_kcps =
			(int16_t)itemp32;

		itemp32 = (int32_t)(orig_xtalk_offset -
			pout->algo__crosstalk_compensation_plane_offset_kcps);
		itemp32 = (int32_t)(do_division_s(itemp32, 80));
		itemp32 = itemp32 << 2;
		itemp32 = itemp32 + (int32_t)(orig_y_gradient);
		if (itemp32 > 0xFFFF)
			itemp32 = 0xFFFF;

		pout->algo__crosstalk_compensation_y_plane_gradient_kcps =
			(int16_t)itemp32;
	}


	if ((pconfig->smudge_corr_apply_enabled == 1) &&
		(soft_update != 1)) {

		pout->new_xtalk_applied_flag = 1;
		nXtalk = pout->algo__crosstalk_compensation_plane_offset_kcps;

		VL53LX_compute_histo_merge_nb(Dev, &histo_merge_nb);
		max = pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge_max_size;
		pcpo = &(pC->algo__xtalk_cpo_HistoMerge_kcps[0]);
		if ((histo_merge_nb > 0) &&
			(pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge == 1) &&
			(nXtalk != 0)) {
			cXtalk =
			pX->algo__crosstalk_compensation_plane_offset_kcps;
			SmudgeFactor = cXtalk * 1000 / nXtalk;
			if ((max ==  0)||
				(SmudgeFactor >= pconfig->max_smudge_factor))
				pout->new_xtalk_applied_flag = 0;
			else {
				incXtalk = nXtalk / max;
				cval = 0;
				for (i = 0; i < max-1; i++) {
					cval += incXtalk;
					*pcpo = cval;
					pcpo++;
				}
				*pcpo = nXtalk;
			}
		}
		if (pout->new_xtalk_applied_flag) {

		pX->algo__crosstalk_compensation_plane_offset_kcps =
		pout->algo__crosstalk_compensation_plane_offset_kcps;
		pX->algo__crosstalk_compensation_x_plane_gradient_kcps =
		pout->algo__crosstalk_compensation_x_plane_gradient_kcps;
		pX->algo__crosstalk_compensation_y_plane_gradient_kcps =
		pout->algo__crosstalk_compensation_y_plane_gradient_kcps;

		if (pconfig->smudge_corr_single_apply == 1) {

			pconfig->smudge_corr_apply_enabled = 0;
			pconfig->smudge_corr_single_apply = 0;
		}
		}
	}


	if (soft_update != 1)
		pout->smudge_corr_valid = 1;

	LOG_FUNCTION_END(status);

	return status;
}

#define CONT_CONTINUE	0
#define CONT_NEXT_LOOP	1
#define CONT_RESET	2
VL53LX_Error VL53LX_dynamic_xtalk_correction_corrector(
	VL53LX_DEV                          Dev
	)
{



	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);
	VL53LX_smudge_corrector_config_t *pconfig =
				&(pdev->VL53LX_LLDriverCommonData->smudge_correct_config);
	VL53LX_smudge_corrector_internals_t *pint =
				&(pdev->smudge_corrector_internals);
	VL53LX_smudge_corrector_data_t *pout =
			&(pres->range_results.smudge_corrector_data);
	VL53LX_range_results_t  *pR = &(pres->range_results);
	VL53LX_xtalk_config_t  *pX = &(pdev->VL53LX_LLDriverCommonData->xtalk_cfg);

	uint8_t	run_smudge_detection = 0;
	uint8_t merging_complete = 0;
	uint8_t	run_nodetect = 0;
	uint8_t ambient_check = 0;
	int32_t itemp32 = 0;
	uint64_t utemp64 = 0;
	uint8_t continue_processing = CONT_CONTINUE;
	uint32_t xtalk_offset_out = 0;
	uint32_t xtalk_offset_in = 0;
	uint32_t current_xtalk = 0;
	uint32_t smudge_margin_adjusted = 0;
	uint8_t i = 0;
	uint8_t nodetect_index = 0;
	uint16_t    amr;
	uint32_t    cco;
	uint8_t histo_merge_nb;


	LOG_FUNCTION_START("");

	VL53LX_compute_histo_merge_nb(Dev, &histo_merge_nb);
	if ((histo_merge_nb == 0) ||
		(pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge != 1))
		histo_merge_nb = 1;


	VL53LX_dynamic_xtalk_correction_output_init(pres);


	ambient_check = (pconfig->smudge_corr_ambient_threshold == 0) ||
		((pconfig->smudge_corr_ambient_threshold * histo_merge_nb)  >
		((uint32_t)pR->xmonitor.ambient_count_rate_mcps));


	merging_complete =
		((pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge != 1) ||
		(histo_merge_nb == pdev->VL53LX_LLDriverCommonData->tuning_parms.tp_hist_merge_max_size));
	run_smudge_detection =
		(pconfig->smudge_corr_enabled == 1) &&
		ambient_check &&
		(pR->xmonitor.range_status
			== VL53LX_DEVICEERROR_RANGECOMPLETE) &&
		merging_complete;


	if ((pR->xmonitor.range_status
		!= VL53LX_DEVICEERROR_RANGECOMPLETE) &&
			(pconfig->smudge_corr_enabled == 1)) {

		run_nodetect = 2;
		for (i = 0; i < pR->active_results; i++) {
			if (pR->VL53LX_p_003[i].range_status ==
				VL53LX_DEVICEERROR_RANGECOMPLETE) {
				if (pR->VL53LX_p_003[i].median_range_mm
						<=
					pconfig->nodetect_min_range_mm) {
					run_nodetect = 0;
				} else {
					if (run_nodetect == 2) {
						run_nodetect = 1;
						nodetect_index = i;
					}
				}
			}
		}

		if (run_nodetect == 2)

			run_nodetect = 0;

		amr =
		pR->VL53LX_p_003[nodetect_index].ambient_count_rate_mcps;

		if (run_nodetect == 1) {




			utemp64 = 1000 * ((uint64_t)amr);


			utemp64 = utemp64 << 9;


			if (utemp64 < pconfig->nodetect_ambient_threshold)
				run_nodetect = 1;
			else
				run_nodetect = 0;

		}
	}


	if (run_smudge_detection) {

		pint->nodetect_counter = 0;


		VL53LX_dynamic_xtalk_correction_calc_required_samples(Dev);


		xtalk_offset_in =
			pR->xmonitor.VL53LX_p_009;


		cco = pX->algo__crosstalk_compensation_plane_offset_kcps;
		current_xtalk = ((uint32_t)cco) << 2;


		smudge_margin_adjusted =
				((uint32_t)(pconfig->smudge_margin)) << 2;


		itemp32 = xtalk_offset_in - current_xtalk +
			smudge_margin_adjusted;

		if (itemp32 < 0)
			itemp32 = itemp32 * (-1);


		if (itemp32 > ((int32_t)pconfig->single_xtalk_delta)) {
			if ((int32_t)xtalk_offset_in >
				((int32_t)current_xtalk -
					(int32_t)smudge_margin_adjusted)) {
				pout->single_xtalk_delta_flag = 1;
			} else {
				pout->single_xtalk_delta_flag = 2;
			}
		}


		pint->current_samples = pint->current_samples + 1;


		if (pint->current_samples > pconfig->sample_limit) {
			pout->sample_limit_exceeded_flag = 1;
			continue_processing = CONT_RESET;
		} else {
			pint->accumulator = pint->accumulator +
				xtalk_offset_in;
		}

		if (pint->current_samples < pint->required_samples)
			continue_processing = CONT_NEXT_LOOP;


		xtalk_offset_out =
		(uint32_t)(do_division_u(pint->accumulator,
			pint->current_samples));


		itemp32 = xtalk_offset_out - current_xtalk +
			smudge_margin_adjusted;

		if (itemp32 < 0)
			itemp32 = itemp32 * (-1);

		if (continue_processing == CONT_CONTINUE &&
			(itemp32 >= ((int32_t)(pconfig->averaged_xtalk_delta)))
			) {
			if ((int32_t)xtalk_offset_out >
				((int32_t)current_xtalk -
					(int32_t)smudge_margin_adjusted))
				pout->averaged_xtalk_delta_flag = 1;
			else
				pout->averaged_xtalk_delta_flag = 2;
		}

		if (continue_processing == CONT_CONTINUE &&
			(itemp32 < ((int32_t)(pconfig->averaged_xtalk_delta)))
			)

			continue_processing = CONT_RESET;



		pout->smudge_corr_clipped = 0;
		if ((continue_processing == CONT_CONTINUE) &&
			(pconfig->smudge_corr_clip_limit != 0)) {
			if (xtalk_offset_out >
			(pconfig->smudge_corr_clip_limit * histo_merge_nb)) {
				pout->smudge_corr_clipped = 1;
				continue_processing = CONT_RESET;
			}
		}



		if (pconfig->user_xtalk_offset_limit_hi &&
			(xtalk_offset_out >
				pconfig->user_xtalk_offset_limit))
			xtalk_offset_out =
				pconfig->user_xtalk_offset_limit;



		if ((pconfig->user_xtalk_offset_limit_hi == 0) &&
			(xtalk_offset_out <
				pconfig->user_xtalk_offset_limit))
			xtalk_offset_out =
				pconfig->user_xtalk_offset_limit;



		xtalk_offset_out = xtalk_offset_out >> 2;
		if (xtalk_offset_out > 0x3FFFF)
			xtalk_offset_out = 0x3FFFF;


		if (continue_processing == CONT_CONTINUE) {

			VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
				Dev,
				xtalk_offset_out,
				pconfig,
				pout,
				1,
				0
				);


			continue_processing = CONT_RESET;
		} else {

			VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
				Dev,
				xtalk_offset_out,
				pconfig,
				pout,
				1,
				1
				);
		}


		if (continue_processing == CONT_RESET) {
			pint->accumulator = 0;
			pint->current_samples = 0;
			pint->nodetect_counter = 0;
		}

	}

	continue_processing = CONT_CONTINUE;
	if (run_nodetect == 1) {

		pint->nodetect_counter += 1;


		if (pint->nodetect_counter < pconfig->nodetect_sample_limit)
			continue_processing = CONT_NEXT_LOOP;


		xtalk_offset_out = (uint32_t)(pconfig->nodetect_xtalk_offset);

		if (continue_processing == CONT_CONTINUE) {

			VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
				Dev,
				xtalk_offset_out,
				pconfig,
				pout,
				0,
				0
				);


			pout->smudge_corr_valid = 2;


			continue_processing = CONT_RESET;
		} else {

			VL53LX_dynamic_xtalk_correction_calc_new_xtalk(
				Dev,
				xtalk_offset_out,
				pconfig,
				pout,
				0,
				1
				);
		}


		if (continue_processing == CONT_RESET) {
			pint->accumulator = 0;
			pint->current_samples = 0;
			pint->nodetect_counter = 0;
		}
	}

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_data_init(
	VL53LX_DEV                          Dev
	)
{




	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_LLDriverResults_t *pres = VL53LXDevStructGetLLResultsHandle(Dev);

	LOG_FUNCTION_START("");



	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_enabled       = 1;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_apply_enabled = 1;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_single_apply  =
		VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_COR_SINGLE_APPLY_DEFAULT;

	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_margin =
		VL53LX_TUNINGPARM_DYNXTALK_SMUDGE_MARGIN_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.noise_margin =
		VL53LX_TUNINGPARM_DYNXTALK_NOISE_MARGIN_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_xtalk_offset_limit =
		VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_xtalk_offset_limit_hi =
		VL53LX_TUNINGPARM_DYNXTALK_XTALK_OFFSET_LIMIT_HI_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.sample_limit =
		VL53LX_TUNINGPARM_DYNXTALK_SAMPLE_LIMIT_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.single_xtalk_delta =
		VL53LX_TUNINGPARM_DYNXTALK_SINGLE_XTALK_DELTA_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.averaged_xtalk_delta =
		VL53LX_TUNINGPARM_DYNXTALK_AVERAGED_XTALK_DELTA_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_clip_limit =
		VL53LX_TUNINGPARM_DYNXTALK_CLIP_LIMIT_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.smudge_corr_ambient_threshold =
		VL53LX_TUNINGPARM_DYNXTALK_XTALK_AMB_THRESHOLD_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.scaler_calc_method =
		0;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.x_gradient_scaler =
		VL53LX_TUNINGPARM_DYNXTALK_XGRADIENT_SCALER_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.y_gradient_scaler =
		VL53LX_TUNINGPARM_DYNXTALK_YGRADIENT_SCALER_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.user_scaler_set =
		VL53LX_TUNINGPARM_DYNXTALK_USER_SCALER_SET_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_ambient_threshold =
		VL53LX_TUNINGPARM_DYNXTALK_NODETECT_AMB_THRESHOLD_KCPS_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_sample_limit =
		VL53LX_TUNINGPARM_DYNXTALK_NODETECT_SAMPLE_LIMIT_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_xtalk_offset =
		VL53LX_TUNINGPARM_DYNXTALK_NODETECT_XTALK_OFFSET_KCPS_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.nodetect_min_range_mm =
		VL53LX_TUNINGPARM_DYNXTALK_NODETECT_MIN_RANGE_MM_DEFAULT;
	pdev->VL53LX_LLDriverCommonData->smudge_correct_config.max_smudge_factor =
		VL53LX_TUNINGPARM_DYNXTALK_MAX_SMUDGE_FACTOR_DEFAULT;


	pdev->smudge_corrector_internals.current_samples = 0;
	pdev->smudge_corrector_internals.required_samples = 0;
	pdev->smudge_corrector_internals.accumulator = 0;
	pdev->smudge_corrector_internals.nodetect_counter = 0;


	VL53LX_dynamic_xtalk_correction_output_init(pres);

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_dynamic_xtalk_correction_output_init(
	VL53LX_LLDriverResults_t *pres
	)
{




	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_smudge_corrector_data_t *pdata;

	LOG_FUNCTION_START("");


	pdata = &(pres->range_results.smudge_corrector_data);

	pdata->smudge_corr_valid = 0;
	pdata->smudge_corr_clipped = 0;
	pdata->single_xtalk_delta_flag = 0;
	pdata->averaged_xtalk_delta_flag = 0;
	pdata->sample_limit_exceeded_flag = 0;
	pdata->gradient_zero_flag = 0;
	pdata->new_xtalk_applied_flag = 0;

	pdata->algo__crosstalk_compensation_plane_offset_kcps = 0;
	pdata->algo__crosstalk_compensation_x_plane_gradient_kcps = 0;
	pdata->algo__crosstalk_compensation_y_plane_gradient_kcps = 0;

	LOG_FUNCTION_END(status);

	return status;
}


VL53LX_Error VL53LX_xtalk_cal_data_init(
	VL53LX_DEV                          Dev
	)
{




	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");



	pdev->xtalk_cal.algo__crosstalk_compensation_plane_offset_kcps = 0;
	pdev->xtalk_cal.algo__crosstalk_compensation_x_plane_gradient_kcps = 0;
	pdev->xtalk_cal.algo__crosstalk_compensation_y_plane_gradient_kcps = 0;
	memset(&pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps[0], 0,
		sizeof(pdev->xtalk_cal.algo__xtalk_cpo_HistoMerge_kcps));

	LOG_FUNCTION_END(status);

	return status;
}






VL53LX_Error VL53LX_low_power_auto_data_init(
	VL53LX_DEV                          Dev
	)
{




	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");

	pdev->low_power_auto_data.vhv_loop_bound =
		VL53LX_TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT;
	pdev->low_power_auto_data.is_low_power_auto_mode = 0;
	pdev->low_power_auto_data.low_power_auto_range_count = 0;
	pdev->low_power_auto_data.saved_interrupt_config = 0;
	pdev->low_power_auto_data.saved_vhv_init = 0;
	pdev->low_power_auto_data.saved_vhv_timeout = 0;
	pdev->low_power_auto_data.first_run_phasecal_result = 0;
	pdev->low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
	pdev->low_power_auto_data.dss__required_spads = 0;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_low_power_auto_data_stop_range(
	VL53LX_DEV                          Dev
	)
{




	VL53LX_Error  status = VL53LX_ERROR_NONE;

	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	LOG_FUNCTION_START("");



	pdev->low_power_auto_data.low_power_auto_range_count = 0xFF;

	pdev->low_power_auto_data.first_run_phasecal_result = 0;
	pdev->low_power_auto_data.dss__total_rate_per_spad_mcps = 0;
	pdev->low_power_auto_data.dss__required_spads = 0;


	if (pdev->low_power_auto_data.saved_vhv_init != 0)
		pdev->stat_nvm.vhv_config__init =
			pdev->low_power_auto_data.saved_vhv_init;
	if (pdev->low_power_auto_data.saved_vhv_timeout != 0)
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
			pdev->low_power_auto_data.saved_vhv_timeout;


	pdev->gen_cfg.phasecal_config__override = 0x00;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_config_low_power_auto_mode(
	VL53LX_general_config_t   *pgeneral,
	VL53LX_dynamic_config_t   *pdynamic,
	VL53LX_low_power_auto_data_t *plpadata
	)
{




	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");


	plpadata->is_low_power_auto_mode = 1;


	plpadata->low_power_auto_range_count = 0;


	pdynamic->system__sequence_config =
			VL53LX_SEQUENCE_VHV_EN |
			VL53LX_SEQUENCE_PHASECAL_EN |
			VL53LX_SEQUENCE_DSS1_EN |



			VL53LX_SEQUENCE_RANGE_EN;


	pgeneral->dss_config__manual_effective_spads_select = 200 << 8;
	pgeneral->dss_config__roi_mode_control =
		VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_low_power_auto_setup_manual_calibration(
	VL53LX_DEV        Dev)
{



	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	LOG_FUNCTION_START("");


	pdev->low_power_auto_data.saved_vhv_init =
		pdev->stat_nvm.vhv_config__init;
	pdev->low_power_auto_data.saved_vhv_timeout =
		pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound;


	pdev->stat_nvm.vhv_config__init &= 0x7F;

	pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound =
		(pdev->stat_nvm.vhv_config__timeout_macrop_loop_bound & 0x03) +
		(pdev->low_power_auto_data.vhv_loop_bound << 2);

	pdev->gen_cfg.phasecal_config__override = 0x01;
	pdev->low_power_auto_data.first_run_phasecal_result =
		pdev->VL53LX_LLDriverCommonData->dbg_results.phasecal_result__vcsel_start;
	pdev->gen_cfg.cal_config__vcsel_start =
		pdev->low_power_auto_data.first_run_phasecal_result;

	LOG_FUNCTION_END(status);

	return status;
}

VL53LX_Error VL53LX_low_power_auto_update_DSS(
	VL53LX_DEV        Dev)
{



	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);

	VL53LX_system_results_t *pS = &(pdev->sys_results);


	VL53LX_Error  status = VL53LX_ERROR_NONE;

	uint32_t utemp32a;

	LOG_FUNCTION_START("");




	utemp32a =
		pS->result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0
		 + pS->result__ambient_count_rate_mcps_sd0;


	if (utemp32a > 0xFFFF)
		utemp32a = 0xFFFF;



	utemp32a = utemp32a << 16;


	if (pdev->sys_results.result__dss_actual_effective_spads_sd0 == 0)
		status = VL53LX_ERROR_DIVISION_BY_ZERO;
	else {

		utemp32a = utemp32a /
		pdev->sys_results.result__dss_actual_effective_spads_sd0;

		pdev->low_power_auto_data.dss__total_rate_per_spad_mcps =
			utemp32a;


		utemp32a = pdev->stat_cfg.dss_config__target_total_rate_mcps <<
			16;


		if (pdev->low_power_auto_data.dss__total_rate_per_spad_mcps
				== 0)
			status = VL53LX_ERROR_DIVISION_BY_ZERO;
		else {

			utemp32a = utemp32a /
			pdev->low_power_auto_data.dss__total_rate_per_spad_mcps;


			if (utemp32a > 0xFFFF)
				utemp32a = 0xFFFF;


			pdev->low_power_auto_data.dss__required_spads =
				(uint16_t)utemp32a;


			pdev->gen_cfg.dss_config__manual_effective_spads_select
			= pdev->low_power_auto_data.dss__required_spads;
			pdev->gen_cfg.dss_config__roi_mode_control =
			VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;
		}

	}

	if (status == VL53LX_ERROR_DIVISION_BY_ZERO) {



		pdev->low_power_auto_data.dss__required_spads = 0x8000;


		pdev->gen_cfg.dss_config__manual_effective_spads_select =
			pdev->low_power_auto_data.dss__required_spads;
		pdev->gen_cfg.dss_config__roi_mode_control =
			VL53LX_DEVICEDSSMODE__REQUESTED_EFFFECTIVE_SPADS;


		status = VL53LX_ERROR_NONE;
	}

	LOG_FUNCTION_END(status);

	return status;
}




VL53LX_Error VL53LX_compute_histo_merge_nb(
	VL53LX_DEV        Dev,	uint8_t *histo_merge_nb)
{
	VL53LX_LLDriverData_t *pdev = VL53LXDevStructGetLLDriverHandle(Dev);
	VL53LX_Error  status = VL53LX_ERROR_NONE;
	uint8_t i, timing;
	uint8_t sum = 0;

	timing = (pdev->VL53LX_LLDriverCommonData->hist_data.bin_seq[0] == 7 ? 1 : 0);
	for (i = 0; i < VL53LX_BIN_REC_SIZE; i++)
		if (pdev->multi_bins_rec[i][timing][7] > 0)
			sum++;
	*histo_merge_nb = sum;

	return status;
}

