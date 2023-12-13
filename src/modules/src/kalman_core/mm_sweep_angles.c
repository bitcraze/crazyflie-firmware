/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021-2023 Bitcraze AB
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

#include "mm_sweep_angles.h"
#include "log.h"
static void logAngles(const sweepAngleMeasurement_t *sweepInfo);


static float predictSweepAngle(const kalmanCoreData_t *this, const sweepAngleMeasurement_t *sweepInfo, const vec3d *sensPos_cf, vec3d *sensPos_r, const mat3d *R);
static void scalarUpdatePosition(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState, const vec3d *sensPos_r, const float error);
static void yawRotMatrix(const float yaw, const mat3d* R, mat3d* result);

void kalmanCoreUpdateWithSweepAngles(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState) {
  // const float D2 = this->S[KC_STATE_D2];

  vec3d sensPos1_r;
  const float predictedSweepAngle1 = predictSweepAngle(this, sweepInfo, sweepInfo->sensorPos1, &sensPos1_r, &this->R);
  const float error1 = sweepInfo->measuredSweepAngle1 - predictedSweepAngle1;
  scalarUpdatePosition(this, sweepInfo, nowMs, sweepOutlierFilterState, &sensPos1_r, error1);

#if 0
  const uint8_t INVALID = 0xff;
  if (sweepInfo->sensorId2 != INVALID) {
    // Estimate yaw
    vec3d tmp_pos;
    const float predictedSweepAngle2 = predictSweepAngle(this, sweepInfo, sweepInfo->sensorPos2, &tmp_pos, &this->R);
    const float measuredAngDelta = sweepInfo->measuredSweepAngle2 - sweepInfo->measuredSweepAngle1;
    const float predictedAngDelta = predictedSweepAngle2 - predictedSweepAngle1;
    const float errorAngDelta = measuredAngDelta - predictedAngDelta;

    // It is tricky to to calculate the derivative of the angle delta analytically, estimate numerically instead
    // Create a rotation matrix from the current rotation matrix and yaw it dD2.
    const float dD2 = 0.1;
    mat3d RAndSome;
    yawRotMatrix(dD2, &this->R, &RAndSome);

    const float predictedSweepAngle1AndSome = predictSweepAngle(this, sweepInfo, sweepInfo->sensorPos1, &tmp_pos, &RAndSome);
    const float predictedSweepAngle2AndSome = predictSweepAngle(this, sweepInfo, sweepInfo->sensorPos2, &tmp_pos, &RAndSome);
    const float predictedAngDeltaAndSome = predictedSweepAngle2AndSome - predictedSweepAngle1AndSome;
    const float changeAngDelta = predictedAngDeltaAndSome - predictedAngDelta;
    const float dAd_dD2 = changeAngDelta / dD2;

    float h[KC_STATE_DIM] = {0};
    h[KC_STATE_D2] = dAd_dD2;
    arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
    kalmanCoreScalarUpdate(this, &H, errorAngDelta, sweepInfo->stdDevAngleDiff);
  }
#endif
}

static float predictSweepAngle(const kalmanCoreData_t *this, const sweepAngleMeasurement_t *sweepInfo, const vec3d *sensPos_cf, vec3d *sensPos_r, const mat3d *R) {
  // Rotate the sensor position from the CF reference frame to global reference frame,
  // using the CF rotation matrix
  vec3d s;
  arm_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)*R};
  arm_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sensPos_cf};
  arm_matrix_instance_f32 s_ = {3, 1, s};
  mat_mult(&Rcf_, &scf_, &s_);

  // Get the current state values of the position of the crazyflie (global reference frame) and add the relative sensor pos
  vec3d pcf = {this->S[KC_STATE_X] + s[0], this->S[KC_STATE_Y] + s[1], this->S[KC_STATE_Z] + s[2]};

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

static void scalarUpdatePosition(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState, const vec3d *sensPos_r, const float error) {
  const float x = (*sensPos_r)[0];
  const float y = (*sensPos_r)[1];
  const float z = (*sensPos_r)[2];

  const float r2 = x * x + y * y;
  const float r = arm_sqrt(r2);

  if (outlierFilterLighthouseValidateSweep(sweepOutlierFilterState, r, error, nowMs)) {
    logAngles(sweepInfo);

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

      arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
      kalmanCoreScalarUpdate(this, &H, error, sweepInfo->stdDevAngle);
    }
  }
}

static void yawRotMatrix(const float yaw, const mat3d* R, mat3d* result) {
  mat3d Ryaw = {
    {cosf(yaw), -sinf(yaw), 0.0},
    {sinf(yaw), cosf(yaw), 0.0},
    {0.0, 0.0, 1.0},
    };

  arm_matrix_instance_f32 R_ = {3, 3, (float*)*R};
  arm_matrix_instance_f32 Ryaw_ = {3, 3, (float*)Ryaw};
  arm_matrix_instance_f32 result_ = {3, 3, (float*)*result};
  mat_mult(&Ryaw_, &R_, &result_);
}


extern uint8_t logBaseStationId;
extern uint8_t logSensor;
static float logAngle1;
static float logAngle2;


static void logAngles(const sweepAngleMeasurement_t *sweepInfo) {
  if (sweepInfo->baseStationId == logBaseStationId) {
    float angle = 0.0;
    bool doLog = false;
    if (sweepInfo->sensorId1 == logSensor) {
      angle = sweepInfo->measuredSweepAngle1;
      doLog = true;
    } else if (sweepInfo->sensorId2 == logSensor) {
      angle = sweepInfo->measuredSweepAngle2;
      doLog = true;
    }

    if (doLog) {
      if (sweepInfo->sweepId == 0) {
        logAngle1 = angle;
      } else {
        logAngle2 = angle;
      }
    }
  }
}


LOG_GROUP_START(lighthouse2)
LOG_ADD(LOG_FLOAT, ang1f, &logAngle1)
LOG_ADD(LOG_FLOAT, ang2f, &logAngle2)
LOG_GROUP_STOP(lighthouse2)
