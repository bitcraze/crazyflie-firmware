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

#include "mm_distance.h"

// Measurement model where the measurement is the distance to a known point in space
void kalmanCoreUpdateWithDistance(kalmanCoreData_t* this, distanceMeasurement_t* d) {
  // a measurement of distance to point (x, y, z)
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  float dx = this->S[KC_STATE_X] - d->x;
  float dy = this->S[KC_STATE_Y] - d->y;
  float dz = this->S[KC_STATE_Z] - d->z;

  float measuredDistance = d->distance;

  float predictedDistance = arm_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
  if (predictedDistance != 0.0f) {
    // The measurement is: z = sqrt(dx^2 + dy^2 + dz^2). The derivative dz/dX gives h.
    h[KC_STATE_X] = dx/predictedDistance;
    h[KC_STATE_Y] = dy/predictedDistance;
    h[KC_STATE_Z] = dz/predictedDistance;
  } else {
    // Avoid divide by zero
    h[KC_STATE_X] = 1.0f;
    h[KC_STATE_Y] = 0.0f;
    h[KC_STATE_Z] = 0.0f;
  }

  kalmanCoreScalarUpdate(this, &H, measuredDistance-predictedDistance, d->stdDev);
}
