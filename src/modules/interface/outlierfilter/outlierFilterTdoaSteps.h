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

// This functionality is deprecated and will be removed after September 2023. Use outlierFilterTdoaValidateIntegrator() instead.
// This code is kept here for a while to make it possible to switch back to the "old" outlier filter if the new one is
// not performing well.

bool outlierFilterTdoaValidateSteps(const tdoaMeasurement_t* tdoa, const float error, const vector_t* jacobian, const point_t* estPos);
