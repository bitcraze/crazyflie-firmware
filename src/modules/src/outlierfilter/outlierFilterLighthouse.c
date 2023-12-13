/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2023 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * outlierFilter.c: Outlier rejection filter for the Lighthouse system
 */

#include <math.h>
#include "outlierFilterLighthouse.h"
#include "stabilizer_types.h"
#include "log.h"
#include "param.h"
#include "debug.h"


static int32_t lhValidityTimeMs = 100;
static float lhMaxError = 0.2f;

void outlierFilterLighthouseReset(OutlierFilterLhState_t* this, const uint32_t nowMs) {
  this->validUntilMs = 0;
}

static bool isFilterClosed;
static float error;
bool outlierFilterLighthouseValidateSweep(OutlierFilterLhState_t* this, const float distanceToBs, const float angleError, const uint32_t nowMs) {
  // float error = distanceToBs * tan(angleError);
  // We use an approximation
  error = distanceToBs * angleError;
  bool isGoodSample = (fabsf(error) < lhMaxError);
  if (isGoodSample) {
    this->validUntilMs = nowMs + lhValidityTimeMs;
  }

  bool result = true;
  isFilterClosed = (nowMs < this->validUntilMs);
  if (isFilterClosed) {
    result = isGoodSample;
  } else {
    // Seems as if we have not got any good measurements for a while. Let all samples through to enable the kalman
    // filter to converge again.
    result = true;
  }

  return result;
}


LOG_GROUP_START(outlierf)
LOG_ADD(LOG_UINT8, lhclosed, &isFilterClosed)
LOG_ADD(LOG_FLOAT, lhErr, &error)
LOG_GROUP_STOP(outlierf)

PARAM_GROUP_START(outlierf)
PARAM_ADD(PARAM_UINT8, valTime, &lhValidityTimeMs)
PARAM_ADD(PARAM_FLOAT, maxErr, &lhMaxError)
PARAM_GROUP_STOP(outlierf)
