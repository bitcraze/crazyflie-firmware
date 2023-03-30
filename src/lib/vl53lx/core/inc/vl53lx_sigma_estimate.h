
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




#ifndef _VL53LX_SIGMA_ESTIMATE_H_
#define _VL53LX_SIGMA_ESTIMATE_H_

#include "vl53lx_types.h"
#include "vl53lx_ll_def.h"

#ifdef __cplusplus
extern "C" {
#endif

#define  VL53LX_D_002  0xFFFF


#define VL53LX_D_008	0xFFFF
#define VL53LX_D_003	0xFFFFFF
#define VL53LX_D_007	0xFFFFFFFF
#define VL53LX_D_005	0x7FFFFFFFFF
#define VL53LX_D_009	0xFFFFFFFFFF
#define VL53LX_D_010	0xFFFFFFFFFFFF
#define VL53LX_D_004	0xFFFFFFFFFFFFFF
#define VL53LX_D_006	0x7FFFFFFFFFFFFFFF
#define VL53LX_D_011	0xFFFFFFFFFFFFFFFF






VL53LX_Error  VL53LX_f_023(
	uint8_t	      sigma_estimator__sigma_ref_mm,
	uint32_t      VL53LX_p_007,
	uint32_t      VL53LX_p_032,
	uint32_t      VL53LX_p_001,
	uint32_t      a_zp,
	uint32_t      c_zp,
	uint32_t      bx,
	uint32_t      ax_zp,
	uint32_t      cx_zp,
	uint32_t      VL53LX_p_028,
	uint16_t      fast_osc_frequency,
	uint16_t      *psigma_est);



#ifdef __cplusplus
}
#endif

#endif

