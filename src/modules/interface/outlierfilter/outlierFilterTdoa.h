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
    float integrator;
    uint32_t latestUpdateMs;
    bool isFilterOpen;
} OutlierFilterTdoaState_t;

void outlierFilterTdoaReset(OutlierFilterTdoaState_t* this);
bool outlierFilterTdoaValidateIntegrator(OutlierFilterTdoaState_t* this, const tdoaMeasurement_t* tdoa, const float error, const uint32_t nowMs);
