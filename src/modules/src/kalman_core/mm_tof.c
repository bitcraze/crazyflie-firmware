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
#include "FreeRTOS.h"
#include "task.h"

static float predictedTerrainHeight = 0.0f;
static float predictedElevation = 0.0f;
static float measuredElevation = 0.0f;

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
    predictedElevation = this->S[KC_STATE_Z];
    float measuredDistance = tof->distance; // [m]
    measuredElevation = tof->distance * cosf(angle) + predictedTerrainHeight; // [m]
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
        float residual = measuredDistance - predictedDistance;

    // Check if residual elevation indicates a new terrain feature below
    static float terrainThreshold = INFINITY;
    if (fabsf(predictedElevation - measuredElevation) > terrainThreshold) {
        // Significant change detected, prepare terrain height measurement
        predictedTerrainHeight += predictedElevation - measuredElevation;  // Update terrain height
    }

    // set terrain threshold after first iteration
    // sensor update rate we receive here is about 25Hz, assuming a maximum vertical velocity of 1m/s, we can expect up to 4cm change in elevation without terrain change
    terrainThreshold = 0.04f;

    residual += predictedTerrainHeight; // Add predicted terrain height to residual

    // Perform Kalman update with the computed residual
    kalmanCoreScalarUpdate(this, &H, residual, tof->stdDev);
  }
}

LOG_GROUP_START(mmtof)
  LOG_ADD(LOG_FLOAT, prdTerrainHeight, &predictedTerrainHeight)
  LOG_ADD(LOG_FLOAT, prdElevation, &predictedElevation)
  LOG_ADD(LOG_FLOAT, measElevation, &measuredElevation)
LOG_GROUP_STOP(mmtof)
