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
#include "debug.h"


#define LH_MS_PER_FRAME (1000 / 120)
static const int32_t lhMinWindowTimeMs = -2 * LH_MS_PER_FRAME;
static const int32_t lhMaxWindowTimeMs = 5 * LH_MS_PER_FRAME;
static const int32_t lhBadSampleWindowChangeMs = -LH_MS_PER_FRAME;
static const int32_t lhGoodSampleWindowChangeMs = LH_MS_PER_FRAME / 2;
static const float lhMaxError = 0.05f;

void outlierFilterLighthouseReset(OutlierFilterLhState_t* this, const uint32_t nowMs) {
  this->openingTimeMs = nowMs;
  this->openingWindowMs = lhMinWindowTimeMs;
}


bool outlierFilterLighthouseValidateSweep(OutlierFilterLhState_t* this, const float distanceToBs, const float angleError, const uint32_t nowMs) {
  // float error = distanceToBs * tan(angleError);
  // We use an approximattion
  float error = distanceToBs * angleError;

  bool isGoodSample = (fabsf(error) < lhMaxError);
  if (isGoodSample) {
    this->openingWindowMs += lhGoodSampleWindowChangeMs;
    if (this->openingWindowMs > lhMaxWindowTimeMs) {
      this->openingWindowMs = lhMaxWindowTimeMs;
    }
  } else {
    this->openingWindowMs += lhBadSampleWindowChangeMs;
    if (this->openingWindowMs < lhMinWindowTimeMs) {
      this->openingWindowMs = lhMinWindowTimeMs;
    }
  }

  bool result = true;
  bool isFilterClosed = (nowMs < this->openingTimeMs);
  if (isFilterClosed) {
    result = isGoodSample;
  }

  this->openingTimeMs = nowMs + this->openingWindowMs;

  return result;
}
