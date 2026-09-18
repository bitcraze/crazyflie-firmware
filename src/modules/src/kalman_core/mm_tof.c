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

#include "mm_tof.h"
#include "log.h"

void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    float predictedDistance = this->S[KC_STATE_Z] / cosf(angle);
    float measuredDistance = tof->distance; // [m]

    /*
    The sensor model (Pg.95-96, https://lup.lub.lu.se/student-papers/search/publication/8905295)
    
    h = z/((R*z_b).z_b) = z/cos(alpha)
    
    Here,
    h (Measured variable)[m] = Distance given by TOF sensor. This is the closest point from any surface to the sensor in the measurement cone
    z (Estimated variable)[m] = THe actual elevation of the crazyflie
    z_b = Basis vector in z direction of body coordinate system
    R = Rotation matrix made from ZYX Tait-Bryan angles. Assumed to be stationary
    alpha = angle between [line made by measured point <---> sensor] and [the intertial z-axis] 
    */

    h[KC_STATE_Z] = 1 / cosf(angle); // This just acts like a gain for the sensor model. Further updates are done in the scalar update function below

    // Scalar update
    kalmanCoreScalarUpdate(this, &H, measuredDistance-predictedDistance, tof->stdDev);
  }
}

#ifdef CONFIG_ESTIMATOR_KALMAN_TERRAIN
static uint32_t terrainSteps;

void kalmanCoreUpdateWithTofTerrain(kalmanCoreData_t* this, const kalmanCoreParams_t *params, tofMeasurement_t *tof)
{
  static uint8_t outsideGate = 0;

  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    const float cosAngle = cosf(angle);
    const float measuredDistance = tof->distance; // [m]

    // The distance is measured to the terrain: d = (z - t) / cos(alpha)
    h[KC_STATE_Z] = 1 / cosAngle;
    h[KC_STATE_T] = -1 / cosAngle;
    float error = measuredDistance - (this->S[KC_STATE_Z] - this->S[KC_STATE_T]) / cosAngle;

    const float innovationVariance = h[KC_STATE_Z] * h[KC_STATE_Z] * this->P[KC_STATE_Z][KC_STATE_Z]
      + 2 * h[KC_STATE_Z] * h[KC_STATE_T] * this->P[KC_STATE_Z][KC_STATE_T]
      + h[KC_STATE_T] * h[KC_STATE_T] * this->P[KC_STATE_T][KC_STATE_T]
      + tof->stdDev * tof->stdDev;

    if (error * error > params->terrainGate * params->terrainGate * innovationVariance) {
      outsideGate++;
      if (outsideGate < params->terrainConfirm) {
        // Could be a single reading on an edge, wait for the next one
        return;
      }
      // A step in the terrain: move it into the terrain state and let the next readings refine it
      this->S[KC_STATE_T] = this->S[KC_STATE_Z] - measuredDistance * cosAngle;
      this->P[KC_STATE_T][KC_STATE_T] = params->terrainResetVariance;
      error = measuredDistance - (this->S[KC_STATE_Z] - this->S[KC_STATE_T]) / cosAngle;
      terrainSteps++;
    }
    outsideGate = 0;

    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
}

LOG_GROUP_START(kalman_terr)
  /**
   * @brief Number of terrain steps detected by the ToF measurement model
   */
  LOG_ADD(LOG_UINT32, steps, &terrainSteps)
LOG_GROUP_STOP(kalman_terr)
#endif
