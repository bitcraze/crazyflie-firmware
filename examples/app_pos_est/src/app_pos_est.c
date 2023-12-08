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
#include "outlierFilterLighthouse.h"
#include "param.h"
#include "system.h"

#define DEBUG_MODULE "POSEST"
#include "debug.h"

typedef struct {
  // Reusing the kalmanCoreData_t to be able to reuse the update and outlier filter implementations from the kalman filter.
  kalmanCoreData_t kalmanCoreData;
  OutlierFilterLhState_t sweepOutlierFilterState;

  // Determines how much new measurements will influence the current state. A higher value gives faster tracking but
  // more sensitive to noise.
  float mixin;

  // Semaphore to signal that we got data from the stabilizer loop to process
  SemaphoreHandle_t runTaskSemaphore;
} State_t;

static State_t this = {
  .mixin = 5.0f,
};


static void updateWithSweepAngles(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState);
static float predictSweepAngle(const kalmanCoreData_t *this, const sweepAngleMeasurement_t *sweepInfo, vec3d *sensPos_r);
static void scalarUpdatePosition(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState, const vec3d *sensPos_r, const float error);

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
      case MeasurementTypeSweepAngle:
        updateWithSweepAngles(&this.kalmanCoreData, &m.data.sweepAngle, nowMs, &this.sweepOutlierFilterState);
        break;
      default:
        break;
      }
    }
  }
}

void estimatorOutOfTreeInit(void) {
    uint32_t nowMs = T2M(xTaskGetTickCount());

  this.kalmanCoreData.S[KC_STATE_X] = 0.0;
  this.kalmanCoreData.S[KC_STATE_Y] = 0.0;
  this.kalmanCoreData.S[KC_STATE_Z] = 0.0;

  outlierFilterLighthouseReset(&this.sweepOutlierFilterState, nowMs);

  DEBUG_PRINT("Initialized the position estimator\n");
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

// This is a copy from mm_sweep_angles.c, unfortunately required to avoid the call to kalmanCoreScalarUpdate() at the end
void updateWithSweepAngles(kalmanCoreData_t *kcd, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState) {
  vec3d sensPos1_r;
  const float predictedSweepAngle1 = predictSweepAngle(kcd, sweepInfo, &sensPos1_r);
  const float error1 = sweepInfo->measuredSweepAngle1 - predictedSweepAngle1;
  scalarUpdatePosition(kcd, sweepInfo, nowMs, sweepOutlierFilterState, &sensPos1_r, error1);
}

// A difference to the kalman filter implementation is that we do not know about our rotation, and therefore we can not
// rotate the sensor to the correct position. This can be solved in two ways:
// 1. Only use one sensor and consider this to be the position
// 2. Use all 4 sensors and get some sort of average. This solution might add more noise, depending on the mixin factor.
static float predictSweepAngle(const kalmanCoreData_t *kcd, const sweepAngleMeasurement_t *sweepInfo, vec3d *sensPos_r) {
  // Rotate the sensor position from the CF reference frame to global reference frame,
  // using the CF rotation matrix
  // vec3d s;
  // arm_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)*R};
  // arm_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sensPos_cf};
  // arm_matrix_instance_f32 s_ = {3, 1, s};
  // mat_mult(&Rcf_, &scf_, &s_);

  // Get the current state values of the position of the crazyflie (global reference frame) and add the relative sensor pos
  // vec3d pcf = {kcd->S[KC_STATE_X] + s[0], kcd->S[KC_STATE_Y] + s[1], kcd->S[KC_STATE_Z] + s[2]};
  vec3d pcf = {kcd->S[KC_STATE_X], kcd->S[KC_STATE_Y], kcd->S[KC_STATE_Z]};

  // Calculate the difference between the rotor and the sensor on the CF (global reference frame)
  const vec3d* pr = sweepInfo->rotorPos;
  vec3d stmp = {pcf[0] - (*pr)[0], pcf[1] - (*pr)[1], pcf[2] - (*pr)[2]};
  arm_matrix_instance_f32 stmp_ = {3, 1, stmp};

  // Rotate the difference in position to the rotor reference frame,
  // using the rotor inverse rotation matrix
  arm_matrix_instance_f32 Rr_inv_ = {3, 3, (float32_t *)(*sweepInfo->rotorRotInv)};
  arm_matrix_instance_f32 sr_ = {3, 1, *sensPos_r};
  mat_mult(&Rr_inv_, &stmp_, &sr_);

  // The following computations are in the rotor reference frame
  const float x = (*sensPos_r)[0];
  const float y = (*sensPos_r)[1];
  const float z = (*sensPos_r)[2];
  const float t = sweepInfo->t;

  const float predictedSweepAngle = sweepInfo->calibrationMeasurementModel(x, y, z, t, sweepInfo->calib);
  return predictedSweepAngle;
}

static void scalarUpdatePosition(kalmanCoreData_t *kcd, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState, const vec3d *sensPos_r, const float error) {
  const float x = (*sensPos_r)[0];
  const float y = (*sensPos_r)[1];
  const float z = (*sensPos_r)[2];

  const float r2 = x * x + y * y;
  const float r = arm_sqrt(r2);

  if (outlierFilterLighthouseValidateSweep(sweepOutlierFilterState, r, error, nowMs)) {
    const float tan_t = tanf(sweepInfo->t);

    // Calculate H vector (in the rotor reference frame)
    const float z_tan_t = z * tan_t;
    const float qNum = r2 - z_tan_t * z_tan_t;
    // Avoid singularity
    if (qNum > 0.0001f) {
      const float q = tan_t / arm_sqrt(qNum);
      vec3d gr = {(-y - x * z * q) / r2, (x - y * z * q) / r2 , q};

      // gr is in the rotor reference frame, rotate back to the global
      // reference frame using the rotor rotation matrix
      vec3d g;
      arm_matrix_instance_f32 gr_ = {3, 1, gr};
      arm_matrix_instance_f32 Rr_ = {3, 3, (float32_t *)(*sweepInfo->rotorRot)};
      arm_matrix_instance_f32 g_ = {3, 1, g};
      mat_mult(&Rr_, &gr_, &g_);

      float h[KC_STATE_DIM] = {0};
      h[KC_STATE_X] = g[0];
      h[KC_STATE_Y] = g[1];
      h[KC_STATE_Z] = g[2];

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
