/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
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


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "estimator.h"
#include "math3d.h"
#include "kalman_core.h"
#include "outlierFilterTdoa.h"
#include "param.h"
#include "system.h"

#define DEBUG_MODULE "LOCOPOS"
#include "debug.h"

typedef struct {
  // Reusing the kalmanCoreData_t to be able to reuse the update and outlier filter implementations from the kalman filter.
  kalmanCoreData_t kalmanCoreData;
  OutlierFilterTdoaState_t outlierFilterState;

  // Determines how much new measurements will influence the current state. A higher value gives faster tracking but
  // more sensitive to noise.
  float mixin;

  // Semaphore to signal that we got data from the stabilizer loop to process
  SemaphoreHandle_t runTaskSemaphore;
} State_t;

static State_t this = {
  .mixin = 0.03,
};


static void updateWithTdoa(kalmanCoreData_t* kcd, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState);

// appMain runs in a separate task
void appMain() {
  systemWaitStart();

  this.runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(this.runTaskSemaphore);

  while (true) {
    xSemaphoreTake(this.runTaskSemaphore, portMAX_DELAY);
    uint32_t nowMs = T2M(xTaskGetTickCount()); // would be nice if this had a precision higher than 1ms...

    // Pull the latest sensors values of interest; discard the rest
    measurement_t m;
    while (estimatorDequeue(&m)) {
      switch (m.type)
      {
        case MeasurementTypeTDOA:
          updateWithTdoa(&this.kalmanCoreData, &m.data.tdoa, nowMs, &this.outlierFilterState);
        break;
      default:
        break;
      }
    }
  }
}

void estimatorOutOfTreeInit(void) {
  this.kalmanCoreData.S[KC_STATE_X] = 0.0;
  this.kalmanCoreData.S[KC_STATE_Y] = 0.0;
  this.kalmanCoreData.S[KC_STATE_Z] = 0.0;

  outlierFilterTdoaReset(&this.outlierFilterState);

  DEBUG_PRINT("Initialized the LocoPos estimator\n");
}

bool estimatorOutOfTreeTest(void) {
  return true;
}

void estimatorOutOfTree(state_t *state, const stabilizerStep_t stabilizerStep) {
  // It is a bit sloppy to jus copy the data here, should probably be protected by a mutex
  state->position.x = this.kalmanCoreData.S[KC_STATE_X];
  state->position.y = this.kalmanCoreData.S[KC_STATE_Y];
  state->position.z = this.kalmanCoreData.S[KC_STATE_Z];

  xSemaphoreGive(this.runTaskSemaphore);
}

// This is a copy from mm_tdoa.c, unfortunately required to avoid the call to kalmanCoreScalarUpdate() at the end
static void updateWithTdoa(kalmanCoreData_t* kcd, tdoaMeasurement_t *tdoa, const uint32_t nowMs, OutlierFilterTdoaState_t* outlierFilterState) {
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

  float measurement = tdoa->distanceDiff;

  // predict based on current state
  float x = kcd->S[KC_STATE_X];
  float y = kcd->S[KC_STATE_Y];
  float z = kcd->S[KC_STATE_Z];

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
  float error = measurement - predicted;

  float h[3] = {0};

  if ((d0 != 0.0f) && (d1 != 0.0f)) {
    h[KC_STATE_X] = (dx1 / d1 - dx0 / d0);
    h[KC_STATE_Y] = (dy1 / d1 - dy0 / d0);
    h[KC_STATE_Z] = (dz1 / d1 - dz0 / d0);

    bool sampleIsGood = outlierFilterTdoaValidateIntegrator(outlierFilterState, tdoa, error, nowMs);
    if (sampleIsGood) {
      const float f = error * this.mixin;
      kcd->S[KC_STATE_X] += h[KC_STATE_X] * f;
      kcd->S[KC_STATE_Y] += h[KC_STATE_Y] * f;
      kcd->S[KC_STATE_Z] += h[KC_STATE_Z] * f;
    }
  }
}

PARAM_GROUP_START(locopos)
PARAM_ADD(PARAM_FLOAT, mixin, &this.mixin)
PARAM_GROUP_STOP(locopos)
