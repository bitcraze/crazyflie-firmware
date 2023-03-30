
/* SPDX-License-Identifier: BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX Protected and is dual licensed,
 either 'STMicroelectronics Proprietary license'
 or 'BSD 3-clause "New" or "Revised" License' , at your option.

 ******************************************************************************

 'STMicroelectronics Proprietary license'

 ******************************************************************************

 License terms: STMicroelectronics Proprietary in accordance with licensing
 terms at www.st.com/sla0081

 ******************************************************************************
 */







#ifndef _VL53LX_XTALK_H_
#define _VL53LX_XTALK_H_

#include "vl53lx_types.h"
#include "vl53lx_ll_def.h"
#include "vl53lx_xtalk_private_structs.h"

#ifdef __cplusplus
extern "C" {
#endif




VL53LX_Error VL53LX_xtalk_calibration_process_data(
	VL53LX_xtalk_range_results_t        *pxtalk_ranges,
	VL53LX_xtalk_histogram_data_t       *pxtalk_shape,
	VL53LX_xtalk_calibration_results_t  *pxtalk_cal);




VL53LX_Error VL53LX_f_041(
		VL53LX_histogram_bin_data_t        *pavg_bins,
		VL53LX_xtalk_algo_data_t           *pdebug,
		VL53LX_xtalk_range_data_t          *pxtalk_data,
		uint8_t                             histogram__window_start,
		uint8_t                             histogram__window_end,
		VL53LX_xtalk_histogram_shape_t     *pxtalk_shape);



VL53LX_Error VL53LX_f_039(
	VL53LX_xtalk_range_results_t  *pxtalk_results,
	VL53LX_xtalk_algo_data_t      *pdebug,
	int16_t                       *xgradient,
	int16_t                       *ygradient);




VL53LX_Error VL53LX_f_040(
	VL53LX_xtalk_range_data_t *pxtalk_data,
	VL53LX_xtalk_algo_data_t  *pdebug,
	uint32_t                  *xtalk_mean_offset_kcps);




VL53LX_Error VL53LX_f_045(
	VL53LX_histogram_bin_data_t	   *phist_data,
	VL53LX_xtalk_range_data_t      *pxtalk_data,
	VL53LX_xtalk_algo_data_t       *pdebug,
	VL53LX_xtalk_histogram_shape_t *pxtalk_histo);





VL53LX_Error VL53LX_f_032(
	uint32_t                       mean_offset,
	int16_t                        xgradient,
	int16_t                        ygradient,
	int8_t                         centre_offset_x,
	int8_t                         centre_offset_y,
	uint16_t                       roi_effective_spads,
	uint8_t                        roi_centre_spad,
	uint8_t                        roi_xy_size,
	uint32_t                      *xtalk_rate_kcps);




VL53LX_Error VL53LX_f_033(
	VL53LX_histogram_bin_data_t    *phist_data,
	VL53LX_xtalk_histogram_shape_t *pxtalk_data,
	uint32_t                        xtalk_rate_kcps,
	VL53LX_histogram_bin_data_t    *pxtalkcount_data);




VL53LX_Error VL53LX_f_047(
	VL53LX_histogram_bin_data_t   *phist_data,
	VL53LX_histogram_bin_data_t   *pxtalk_data,
	uint8_t                        xtalk_bin_offset);



VL53LX_Error VL53LX_f_044(
	VL53LX_histogram_bin_data_t       *pxtalk_data,
	uint32_t                           amb_threshold,
	uint8_t                            VL53LX_p_019,
	uint8_t                            VL53LX_p_024);



VL53LX_Error VL53LX_f_046(
	VL53LX_customer_nvm_managed_t *pcustomer,
	VL53LX_dynamic_config_t       *pdyn_cfg,
	VL53LX_xtalk_histogram_data_t *pxtalk_shape,
	VL53LX_histogram_bin_data_t   *pip_hist_data,
	VL53LX_histogram_bin_data_t   *pop_hist_data,
	VL53LX_histogram_bin_data_t   *pxtalk_count_data);




VL53LX_Error VL53LX_f_043(
		uint8_t                      sigma_mult,
		int32_t                      VL53LX_p_028,
		uint32_t                    *ambient_noise);


#ifdef __cplusplus
}
#endif

#endif

