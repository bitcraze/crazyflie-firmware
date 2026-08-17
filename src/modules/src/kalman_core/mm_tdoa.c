/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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

#include "mm_tdoa.h"
#include "test_support.h"
#include <math.h>

#if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
#include "outlierFilterTdoaSteps.h"
#endif

// Computes the innovation (error = measured - predicted) and the measurement
// jacobian h for a TDoA measurement, given the current state. Returns false
// in the degenerate case (the state coincides with an anchor position), in
// which case h is left zeroed and the measurement must be skipped.
static bool measurementModel(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa, float* error, float h[KC_STATE_DIM]) {
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

  float measurement = tdoa->distanceDiff;

  // predict based on current state
  float x = this->S[KC_STATE_X];
  float y = this->S[KC_STATE_Y];
  float z = this->S[KC_STATE_Z];

  float x1 = tdoa->anchorPositions[1].x, y1 = tdoa->anchorPositions[1].y, z1 = tdoa->anchorPositions[1].z;
  float x0 = tdoa->anchorPositions[0].x, y0 = tdoa->anchorPositions[0].y, z0 = tdoa->anchorPositions[0].z;

  float dx1 = x - x1;
  float dy1 = y - y1;
  float dz1 = z - z1;

  float dy0 = y - y0;
  float dx0 = x - x0;
  float dz0 = z - z0;

  float d1 = sqrtf(powf(dx1, 2) + powf(dy1, 2) + powf(dz1, 2));
  float d0 = sqrtf(powf(dx0, 2) + powf(dy0, 2) + powf(dz0, 2));

  float predicted = d1 - d0;
  *error = measurement - predicted;

  if ((d0 == 0.0f) || (d1 == 0.0f)) {
    return false;
  }

  h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
  h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
  h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);
  return true;
}

void kalmanCoreUpdateWithTdoa(kalmanCoreData_t* this, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return;
  }
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

#if CONFIG_ESTIMATOR_KALMAN_TDOA_OUTLIERFILTER_FALLBACK
  vector_t jacobian = {
    .x = h[KC_STATE_X],
    .y = h[KC_STATE_Y],
    .z = h[KC_STATE_Z],
  };

  point_t estimatedPosition = {
    .x = this->S[KC_STATE_X],
    .y = this->S[KC_STATE_Y],
    .z = this->S[KC_STATE_Z],
  };

  bool sampleIsGood = outlierFilterTdoaValidateSteps(tdoa, error, &jacobian, &estimatedPosition);
#else
  bool sampleIsGood = outlierFilterTdoaValidateIntegrator(outlierFilterState, tdoa, error, nowMs);
#endif

  if (sampleIsGood) {
    kalmanCoreScalarUpdate(this, &H, error, tdoa->stdDev);
  }
}

float kalmanCoreTdoaInnovation(const kalmanCoreData_t* this, const tdoaMeasurement_t* tdoa)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return NAN;
  }
  return error;
}

void kalmanCoreUpdateWithTdoaUnfiltered(kalmanCoreData_t* this, tdoaMeasurement_t* tdoa)
{
  float error;
  float h[KC_STATE_DIM] = {0};
  if (!measurementModel(this, tdoa, &error, h)) {
    return;
  }
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
  kalmanCoreScalarUpdate(this, &H, error, tdoa->stdDev);
}
