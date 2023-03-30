
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53LX_API_PRESET_MODES_H_
#define _VL53LX_API_PRESET_MODES_H_

#include "vl53lx_ll_def.h"
#include "vl53lx_dmax_structs.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53LX_Error VL53LX_init_refspadchar_config_struct(
	VL53LX_refspadchar_config_t     *pdata);




VL53LX_Error VL53LX_init_ssc_config_struct(
	VL53LX_ssc_config_t     *pdata);




VL53LX_Error VL53LX_init_xtalk_config_struct(
		VL53LX_customer_nvm_managed_t *pnvm,
		VL53LX_xtalk_config_t   *pdata);



VL53LX_Error VL53LX_init_xtalk_extract_config_struct(
		VL53LX_xtalkextract_config_t   *pdata);



VL53LX_Error VL53LX_init_offset_cal_config_struct(
	VL53LX_offsetcal_config_t   *pdata);



VL53LX_Error VL53LX_init_zone_cal_config_struct(
	VL53LX_zonecal_config_t   *pdata);



VL53LX_Error VL53LX_init_hist_post_process_config_struct(
	uint8_t                              xtalk_compensation_enable,
	VL53LX_hist_post_process_config_t   *pdata);




VL53LX_Error VL53LX_init_dmax_calibration_data_struct(
	VL53LX_dmax_calibration_data_t   *pdata);




VL53LX_Error VL53LX_init_tuning_parm_storage_struct(
	VL53LX_tuning_parm_storage_t   *pdata);



VL53LX_Error VL53LX_init_hist_gen3_dmax_config_struct(
	VL53LX_hist_gen3_dmax_config_t   *pdata);




VL53LX_Error VL53LX_preset_mode_standard_ranging(
	VL53LX_static_config_t     *pstatic,
	VL53LX_histogram_config_t  *phistogram,
	VL53LX_general_config_t    *pgeneral,
	VL53LX_timing_config_t     *ptiming,
	VL53LX_dynamic_config_t    *pdynamic,
	VL53LX_system_control_t    *psystem,
	VL53LX_tuning_parm_storage_t *ptuning_parms,
	VL53LX_zone_config_t       *pzone_cfg);




VL53LX_Error VL53LX_preset_mode_histogram_ranging(
	VL53LX_hist_post_process_config_t *phistpostprocess,
	VL53LX_static_config_t            *pstatic,
	VL53LX_histogram_config_t         *phistogram,
	VL53LX_general_config_t           *pgeneral,
	VL53LX_timing_config_t            *ptiming,
	VL53LX_dynamic_config_t           *pdynamic,
	VL53LX_system_control_t           *psystem,
	VL53LX_tuning_parm_storage_t      *ptuning_parms,
	VL53LX_zone_config_t              *pzone_cfg);




VL53LX_Error VL53LX_preset_mode_histogram_long_range(
	VL53LX_hist_post_process_config_t *phistpostprocess,
	VL53LX_static_config_t            *pstatic,
	VL53LX_histogram_config_t         *phistogram,
	VL53LX_general_config_t           *pgeneral,
	VL53LX_timing_config_t            *ptiming,
	VL53LX_dynamic_config_t           *pdynamic,
	VL53LX_system_control_t           *psystem,
	VL53LX_tuning_parm_storage_t      *ptuning_parms,
	VL53LX_zone_config_t              *pzone_cfg);




VL53LX_Error VL53LX_preset_mode_histogram_medium_range(
	VL53LX_hist_post_process_config_t *phistpostprocess,
	VL53LX_static_config_t            *pstatic,
	VL53LX_histogram_config_t         *phistogram,
	VL53LX_general_config_t           *pgeneral,
	VL53LX_timing_config_t            *ptiming,
	VL53LX_dynamic_config_t           *pdynamic,
	VL53LX_system_control_t           *psystem,
	VL53LX_tuning_parm_storage_t      *ptuning_parms,
	VL53LX_zone_config_t              *pzone_cfg);




VL53LX_Error VL53LX_preset_mode_histogram_short_range(
	VL53LX_hist_post_process_config_t *phistpostprocess,
	VL53LX_static_config_t            *pstatic,
	VL53LX_histogram_config_t         *phistogram,
	VL53LX_general_config_t           *pgeneral,
	VL53LX_timing_config_t            *ptiming,
	VL53LX_dynamic_config_t           *pdynamic,
	VL53LX_system_control_t           *psystem,
	VL53LX_tuning_parm_storage_t      *ptuning_parms,
	VL53LX_zone_config_t              *pzone_cfg);




void VL53LX_copy_hist_cfg_to_static_cfg(
	VL53LX_histogram_config_t  *phistogram,
	VL53LX_static_config_t     *pstatic,
	VL53LX_general_config_t    *pgeneral,
	VL53LX_timing_config_t     *ptiming,
	VL53LX_dynamic_config_t    *pdynamic);



void VL53LX_copy_hist_bins_to_static_cfg(
	VL53LX_histogram_config_t *phistogram,
	VL53LX_static_config_t    *pstatic,
	VL53LX_timing_config_t    *ptiming);

#ifdef __cplusplus
}
#endif

#endif

