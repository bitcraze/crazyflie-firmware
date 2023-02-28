/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * Outlier rejection filter for the kalman filter
 */

#pragma once

#include "stabilizer_types.h"

typedef struct {
    uint32_t openingTimeMs;
    int32_t openingWindowMs;
} OutlierFilterLhState_t;

bool outlierFilterLighthouseValidateSweep(OutlierFilterLhState_t* this, const float distanceToBs, const float angleError, const uint32_t nowMs);
void outlierFilterLighthouseReset(OutlierFilterLhState_t* this, const uint32_t nowMs);
