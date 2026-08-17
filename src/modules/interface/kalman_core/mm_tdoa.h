/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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
 */

#pragma once

#include "kalman_core.h"
#include "outlierFilterTdoa.h"

// Measurements of a UWB Tx/Rx
void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState);

// Innovation (error = measured - predicted distance difference) of a TDoA
// measurement against the current state, without updating anything.
// Returns NAN in the degenerate case (state coincides with an anchor), where
// the firmware skips the sample entirely. Used by the Python bindings to
// replay logged data with an externalized (pluggable) outlier filter.
float kalmanCoreTdoaInnovation(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa);

// TDoA measurement update with NO outlier filter: the caller has already
// decided the sample is good. No-op in the degenerate case. Bindings-replay
// companion of kalmanCoreTdoaInnovation; not used by the flight code path.
void kalmanCoreUpdateWithTdoaUnfiltered(kalmanCoreData_t* this, tdoaMeasurement_t* tdoa);
