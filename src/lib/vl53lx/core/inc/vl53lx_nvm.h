
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */







#ifndef _VL53LX_NVM_H_
#define _VL53LX_NVM_H_

#include "vl53lx_platform.h"
#include "vl53lx_ll_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define VL53LX_NVM_POWER_UP_DELAY_US             50
#define VL53LX_NVM_READ_TRIGGER_DELAY_US          5



VL53LX_Error VL53LX_nvm_enable(
	VL53LX_DEV     Dev,
	uint16_t       nvm_ctrl_pulse_width,
	int32_t        nvm_power_up_delay_us);




VL53LX_Error VL53LX_nvm_read(
	VL53LX_DEV     Dev,
	uint8_t        start_address,
	uint8_t        count,
	uint8_t       *pdata);




VL53LX_Error VL53LX_nvm_disable(
	VL53LX_DEV     Dev);




VL53LX_Error VL53LX_nvm_format_decode(
	uint16_t                   buf_size,
	uint8_t                   *pbuffer,
	VL53LX_decoded_nvm_data_t *pdata);




VL53LX_Error VL53LX_nvm_decode_optical_centre(
	uint16_t                             buf_size,
	uint8_t                             *pbuffer,
	VL53LX_optical_centre_t             *pdata);




VL53LX_Error VL53LX_nvm_decode_cal_peak_rate_map(
	uint16_t                             buf_size,
	uint8_t                             *pbuffer,
	VL53LX_cal_peak_rate_map_t          *pdata);




VL53LX_Error VL53LX_nvm_decode_additional_offset_cal_data(
	uint16_t                             buf_size,
	uint8_t                             *pbuffer,
	VL53LX_additional_offset_cal_data_t *pdata);




VL53LX_Error VL53LX_nvm_decode_fmt_range_results_data(
	uint16_t                             buf_size,
	uint8_t                             *pbuffer,
	VL53LX_decoded_nvm_fmt_range_data_t *pdata);




VL53LX_Error VL53LX_nvm_decode_fmt_info(
	uint16_t                       buf_size,
	uint8_t                       *pbuffer,
	VL53LX_decoded_nvm_fmt_info_t *pdata);




VL53LX_Error VL53LX_nvm_decode_ews_info(
	uint16_t                       buf_size,
	uint8_t                       *pbuffer,
	VL53LX_decoded_nvm_ews_info_t *pdata);




void VL53LX_nvm_format_encode(
	VL53LX_decoded_nvm_data_t *pnvm_info,
	uint8_t                   *pnvm_data);




VL53LX_Error VL53LX_read_nvm_raw_data(
	VL53LX_DEV     Dev,
	uint8_t        start_address,
	uint8_t        count,
	uint8_t       *pnvm_raw_data);




VL53LX_Error VL53LX_read_nvm(
	VL53LX_DEV                 Dev,
	uint8_t                    nvm_format,
	VL53LX_decoded_nvm_data_t *pnvm_info);




VL53LX_Error VL53LX_read_nvm_optical_centre(
	VL53LX_DEV                           Dev,
	VL53LX_optical_centre_t             *pcentre);




VL53LX_Error VL53LX_read_nvm_cal_peak_rate_map(
	VL53LX_DEV                           Dev,
	VL53LX_cal_peak_rate_map_t          *pcal_data);




VL53LX_Error VL53LX_read_nvm_additional_offset_cal_data(
	VL53LX_DEV                           Dev,
	VL53LX_additional_offset_cal_data_t *pcal_data);




VL53LX_Error VL53LX_read_nvm_fmt_range_results_data(
	VL53LX_DEV                           Dev,
	uint16_t                             range_results_select,
	VL53LX_decoded_nvm_fmt_range_data_t *prange_data);


#ifdef __cplusplus
}
#endif

#endif

