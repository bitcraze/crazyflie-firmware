
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

#ifdef VL53LX_NEEDS_IPP
#  undef VL53LX_IPP_API
#  define VL53LX_IPP_API  __declspec(dllimport)
#  pragma comment (lib, "EwokPlus25API_IPP")
#endif

