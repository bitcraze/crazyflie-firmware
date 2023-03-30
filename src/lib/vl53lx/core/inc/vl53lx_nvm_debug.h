
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifndef _VL53LX_NVM_DEBUG_H_
#define _VL53LX_NVM_DEBUG_H_

#include "vl53lx_ll_def.h"
#include "vl53lx_nvm_structs.h"



#ifdef __cplusplus
extern "C"
{
#endif

#ifdef VL53LX_LOG_ENABLE



void VL53LX_print_nvm_raw_data(
	uint8_t                       *pnvm_raw_data,
	uint32_t                       trace_flags);




void VL53LX_print_decoded_nvm_data(
	VL53LX_decoded_nvm_data_t *pdata,
	char                      *pprefix,
	uint32_t                   trace_flags);




void VL53LX_print_decoded_nvm_fmt_range_data(
	VL53LX_decoded_nvm_fmt_range_data_t *pdata,
	char                                *pprefix,
	uint32_t                             trace_flags);




void VL53LX_print_decoded_nvm_fmt_info(
	VL53LX_decoded_nvm_fmt_info_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);



void VL53LX_print_decoded_nvm_ews_info(
	VL53LX_decoded_nvm_ews_info_t *pdata,
	char                          *pprefix,
	uint32_t                       trace_flags);

#endif

#ifdef __cplusplus
}
#endif

#endif

