
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53LX_API_CORE_H_
#define _VL53LX_API_CORE_H_

#include "vl53lx_platform.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53LX_Error VL53LX_get_version(
	VL53LX_DEV            Dev,
	VL53LX_ll_version_t  *pversion);





VL53LX_Error VL53LX_data_init(
	VL53LX_DEV         Dev,
	uint8_t            read_p2p_data);




VL53LX_Error VL53LX_read_p2p_data(
	VL53LX_DEV      Dev);




VL53LX_Error VL53LX_set_part_to_part_data(
	VL53LX_DEV                            Dev,
	VL53LX_calibration_data_t            *pcal_data);




VL53LX_Error VL53LX_get_part_to_part_data(
	VL53LX_DEV                            Dev,
	VL53LX_calibration_data_t            *pcal_data);




VL53LX_Error VL53LX_get_tuning_debug_data(
	VL53LX_DEV                            Dev,
	VL53LX_tuning_parameters_t            *ptun_data);




VL53LX_Error VL53LX_set_inter_measurement_period_ms(
	VL53LX_DEV          Dev,
	uint32_t            inter_measurement_period_ms);




VL53LX_Error VL53LX_get_inter_measurement_period_ms(
	VL53LX_DEV          Dev,
	uint32_t           *pinter_measurement_period_ms);




VL53LX_Error VL53LX_set_timeouts_us(
	VL53LX_DEV          Dev,
	uint32_t            phasecal_config_timeout_us,
	uint32_t            mm_config_timeout_us,
	uint32_t            range_config_timeout_us);




VL53LX_Error VL53LX_get_timeouts_us(
	VL53LX_DEV          Dev,
	uint32_t           *pphasecal_config_timeout_us,
	uint32_t           *pmm_config_timeout_us,
	uint32_t           *prange_config_timeout_us);




VL53LX_Error VL53LX_set_user_zone(
	VL53LX_DEV          Dev,
	VL53LX_user_zone_t *puser_zone);




VL53LX_Error VL53LX_get_user_zone(
	VL53LX_DEV          Dev,
	VL53LX_user_zone_t *puser_zone);




VL53LX_Error VL53LX_get_mode_mitigation_roi(
	VL53LX_DEV          Dev,
	VL53LX_user_zone_t *pmm_roi);

VL53LX_Error VL53LX_init_zone_config_histogram_bins(VL53LX_zone_config_t *pdata);
VL53LX_Error VL53LX_set_zone_config(VL53LX_DEV Dev, VL53LX_zone_config_t *pzone_cfg);
VL53LX_Error VL53LX_get_zone_config(VL53LX_DEV Dev, VL53LX_zone_config_t *pzone_cfg);



VL53LX_Error VL53LX_set_preset_mode(
	VL53LX_DEV                   Dev,
	VL53LX_DevicePresetModes     device_preset_mode,
	uint16_t                     dss_config__target_total_rate_mcps,
	uint32_t                     phasecal_config_timeout_us,
	uint32_t                     mm_config_timeout_us,
	uint32_t                     range_config_timeout_us,
	uint32_t                     inter_measurement_period_ms);




VL53LX_Error VL53LX_get_preset_mode_timing_cfg(
	VL53LX_DEV                   Dev,
	VL53LX_DevicePresetModes     device_preset_mode,
	uint16_t                    *pdss_config__target_total_rate_mcps,
	uint32_t                    *pphasecal_config_timeout_us,
	uint32_t                    *pmm_config_timeout_us,
	uint32_t                    *prange_config_timeout_us);



VL53LX_Error VL53LX_enable_xtalk_compensation(
	VL53LX_DEV                 Dev);



VL53LX_Error VL53LX_disable_xtalk_compensation(
	VL53LX_DEV                 Dev);




void VL53LX_get_xtalk_compensation_enable(
	VL53LX_DEV    Dev,
	uint8_t       *pcrosstalk_compensation_enable);



VL53LX_Error VL53LX_init_and_start_range(
	VL53LX_DEV                      Dev,
	uint8_t                         measurement_mode,
	VL53LX_DeviceConfigLevel        device_config_level);




VL53LX_Error VL53LX_stop_range(
	VL53LX_DEV  Dev);




VL53LX_Error VL53LX_get_measurement_results(
	VL53LX_DEV                  Dev,
	VL53LX_DeviceResultsLevel   device_result_level);




VL53LX_Error VL53LX_get_device_results(
	VL53LX_DEV                 Dev,
	VL53LX_DeviceResultsLevel  device_result_level,
	VL53LX_range_results_t    *prange_results);




VL53LX_Error VL53LX_clear_interrupt_and_enable_next_range(
	VL53LX_DEV       Dev,
	uint8_t          measurement_mode);




VL53LX_Error VL53LX_get_histogram_bin_data(
	VL53LX_DEV                   Dev,
	VL53LX_histogram_bin_data_t *phist_data);




void VL53LX_copy_sys_and_core_results_to_range_results(
	int32_t                           gain_factor,
	VL53LX_system_results_t          *psys,
	VL53LX_core_results_t            *pcore,
	VL53LX_range_results_t           *presults);



VL53LX_Error VL53LX_set_zone_dss_config(
	  VL53LX_DEV                      Dev,
	  VL53LX_zone_private_dyn_cfg_t  *pzone_dyn_cfg);




VL53LX_Error VL53LX_set_dmax_mode(
	VL53LX_DEV              Dev,
	VL53LX_DeviceDmaxMode   dmax_mode);



VL53LX_Error VL53LX_get_dmax_mode(
	VL53LX_DEV               Dev,
	VL53LX_DeviceDmaxMode   *pdmax_mode);




VL53LX_Error VL53LX_get_dmax_calibration_data(
	VL53LX_DEV                      Dev,
	VL53LX_DeviceDmaxMode           dmax_mode,
	VL53LX_dmax_calibration_data_t *pdmax_cal);




VL53LX_Error VL53LX_set_offset_correction_mode(
	VL53LX_DEV                     Dev,
	VL53LX_OffsetCalibrationMode   offset_cor_mode);




VL53LX_Error VL53LX_get_offset_correction_mode(
	VL53LX_DEV                    Dev,
	VL53LX_OffsetCorrectionMode  *poffset_cor_mode);




VL53LX_Error VL53LX_get_tuning_parm(
	VL53LX_DEV                     Dev,
	VL53LX_TuningParms             tuning_parm_key,
	int32_t                       *ptuning_parm_value);



VL53LX_Error VL53LX_set_tuning_parm(
	VL53LX_DEV                     Dev,
	VL53LX_TuningParms             tuning_parm_key,
	int32_t                        tuning_parm_value);



VL53LX_Error VL53LX_dynamic_xtalk_correction_enable(
	VL53LX_DEV                     Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_disable(
	VL53LX_DEV                     Dev
	);




VL53LX_Error VL53LX_dynamic_xtalk_correction_apply_enable(
	VL53LX_DEV                          Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_apply_disable(
	VL53LX_DEV                          Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_single_apply_enable(
	VL53LX_DEV                          Dev
	);



VL53LX_Error VL53LX_dynamic_xtalk_correction_single_apply_disable(
	VL53LX_DEV                          Dev
	);



VL53LX_Error VL53LX_get_current_xtalk_settings(
	VL53LX_DEV                          Dev,
	VL53LX_xtalk_calibration_results_t *pxtalk
	);



VL53LX_Error VL53LX_set_current_xtalk_settings(
	VL53LX_DEV                          Dev,
	VL53LX_xtalk_calibration_results_t *pxtalk
	);

VL53LX_Error VL53LX_load_patch(VL53LX_DEV Dev);

VL53LX_Error VL53LX_unload_patch(VL53LX_DEV Dev);

#ifdef __cplusplus
}
#endif

#endif

