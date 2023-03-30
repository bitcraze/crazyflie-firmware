
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */




#ifndef _VL53LX_ERROR_EXCEPTIONS_H_
#define _VL53LX_ERROR_EXCEPTIONS_H_

#define IGNORE_DIVISION_BY_ZERO                                0

#define IGNORE_XTALK_EXTRACTION_NO_SAMPLE_FAIL                 0
#define IGNORE_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL               0
#define IGNORE_XTALK_EXTRACTION_NO_SAMPLE_FOR_GRADIENT_WARN    0
#define IGNORE_XTALK_EXTRACTION_SIGMA_LIMIT_FOR_GRADIENT_WARN  0
#define IGNORE_XTALK_EXTRACTION_MISSING_SAMPLES_WARN           0

#define IGNORE_REF_SPAD_CHAR_NOT_ENOUGH_SPADS                  0
#define IGNORE_REF_SPAD_CHAR_RATE_TOO_HIGH                     0
#define IGNORE_REF_SPAD_CHAR_RATE_TOO_LOW                      0

#define IGNORE_OFFSET_CAL_MISSING_SAMPLES                      0
#define IGNORE_OFFSET_CAL_SIGMA_TOO_HIGH                       0
#define IGNORE_OFFSET_CAL_RATE_TOO_HIGH                        0
#define IGNORE_OFFSET_CAL_SPAD_COUNT_TOO_LOW				   0

#define IGNORE_ZONE_CAL_MISSING_SAMPLES                        0
#define IGNORE_ZONE_CAL_SIGMA_TOO_HIGH                         0
#define IGNORE_ZONE_CAL_RATE_TOO_HIGH                          0

#endif

