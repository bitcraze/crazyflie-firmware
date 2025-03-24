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

#include "mm_sweep_angles.h"


void kalmanCoreUpdateWithSweepAngles(kalmanCoreData_t *this, sweepAngleMeasurement_t *sweepInfo, const uint32_t nowMs, OutlierFilterLhState_t* sweepOutlierFilterState) {
  // Relative sensor position in global reference frame
  vec3d s;
  arm_matrix_instance_f32 s_ = {3, 1, s};
  // Rotation matrix from Crazyflie to global reference frame
  arm_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)this->R};
  // Relative sensor position in Crazyflie reference frame
  arm_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sweepInfo->sensorPos};

  // Rotate the relative sensor position to the global reference frame
  mat_mult(&Rcf_, &scf_, &s_);

  // Sensor position in global reference frame
  // Gets the current state values of the position of the Crazyflie in the global reference frame and add the relative sensor pos
  vec3d ps = {this->S[KC_STATE_X] + s[0], this->S[KC_STATE_Y] + s[1], this->S[KC_STATE_Z] + s[2]};

  // Rotor position in global reference frame
  const vec3d* pr = sweepInfo->rotorPos;
  // Difference in position between the rotor and the sensor in global reference frame
  vec3d stmp = {ps[0] - (*pr)[0], ps[1] - (*pr)[1], ps[2] - (*pr)[2]};
  arm_matrix_instance_f32 stmp_ = {3, 1, stmp};

  // Rotate the difference in position to the rotor reference frame,
  // using the rotor inverse rotation matrix
  vec3d sr;
  arm_matrix_instance_f32 Rr_inv_ = {3, 3, (float32_t *)(*sweepInfo->rotorRotInv)};
  arm_matrix_instance_f32 sr_ = {3, 1, sr};
  mat_mult(&Rr_inv_, &stmp_, &sr_);

  // The following computations are in the rotor refernece frame
  const float x = sr[0];
  const float y = sr[1];
  const float z = sr[2];
  const float t = sweepInfo->t;
  const float tan_t = tanf(t);

  const float r2 = x * x + y * y;
  const float r = arm_sqrt(r2);

  const float predictedSweepAngle = sweepInfo->calibrationMeasurementModel(x, y, z, t, sweepInfo->calib);
  const float measuredSweepAngle = sweepInfo->measuredSweepAngle;
  const float error = measuredSweepAngle - predictedSweepAngle;

  if (outlierFilterLighthouseValidateSweep(sweepOutlierFilterState, r, error, nowMs)) {
    // Calculate H vector (in the rotor reference frame)
    const float z_tan_t = z * tan_t;
    const float qNum = r2 - z_tan_t * z_tan_t;
    // Avoid singularity
    if (qNum > 0.0001f) {
      // Position Jacobians: ∂α/∂x, ∂α/∂y, ∂α/∂z, where x y and z are the sensor position in the global reference frame

      const float q = tan_t / arm_sqrt(qNum);

      // Jacobian of the sweep angle measurement α w.r.t sensor position (x, y, z)
      // Computed in the rotor reference frame:
      // gr = [ ∂α/∂x, ∂α/∂y, ∂α/∂z ] (rotor frame)
      //
      // For derivation review the paper:
      // Taffanel et al. (2021), "Lighthouse positioning system: Dataset, accuracy, and precision for UAV research", arXiv:2104.11523
      vec3d gr = {(-y - x * z * q) / r2, (x - y * z * q) / r2 , q};

      // Jacobian of the sweep angle measurement α w.r.t sensor position,
      // rotated from rotor frame (gr) into the global reference frame:
      // g = [ ∂α/∂X, ∂α/∂Y, ∂α/∂Z ] (global frame)
      vec3d g;

      arm_matrix_instance_f32 gr_ = {3, 1, gr};
      arm_matrix_instance_f32 Rr_ = {3, 3, (float32_t *)(*sweepInfo->rotorRot)};
      arm_matrix_instance_f32 g_ = {3, 1, g};
      mat_mult(&Rr_, &gr_, &g_);

      float h[KC_STATE_DIM] = {0};
      h[KC_STATE_X] = g[0]; // ∂α/∂X
      h[KC_STATE_Y] = g[1]; // ∂α/∂Y
      h[KC_STATE_Z] = g[2]; // ∂α/∂Z

      // Orientation Jacobians: ∂α/∂φ, ∂α/∂θ, ∂α/∂ψ
      //
      // Applying the chain rule ∂α/∂φ = ∂α/∂x * ∂x/∂φ + ∂α/∂y * ∂y/∂φ + ∂α/∂z * ∂z/∂φ
      // Similar for ∂α/∂θ, ∂α/∂ψ
      //
      // We already have ∂α/∂X, ∂α/∂Y, ∂α/∂Z from the position Jacobians
      // We need ∂x/∂φ, ∂y/∂φ, ∂z/∂φ, ∂x/∂θ, ∂y/∂θ, ∂z/∂θ, ∂x/∂ψ, ∂y/∂ψ, ∂z/∂ψ
      // These can be derived from the rotation matrices

      float dx_droll = 0.0f; // ∂x/∂φ
      float dy_droll = -s[2]; // ∂y/∂φ
      float dz_droll = s[1]; // ∂z/∂φ

      float dx_dpitch = -s[2]; // ∂x/∂θ
      float dy_dpitch = 0.0f; // ∂y/∂θ
      float dz_dpitch = s[0]; // ∂z/∂θ

      float dx_dyaw = -s[1]; // ∂x/∂ψ
      float dy_dyaw = s[0]; // ∂y/∂ψ
      float dz_dyaw = 0.0f; // ∂z/∂ψ

      h[KC_STATE_D0] = dx_droll*g[0] + dy_droll*g[1] + dz_droll*g[2]; // ∂α/∂φ
      h[KC_STATE_D1] = dx_dpitch*g[0] + dy_dpitch*g[1] + dz_dpitch*g[2]; // ∂α/∂θ
      h[KC_STATE_D2] = dx_dyaw*g[0] + dy_dyaw*g[1] + dz_dyaw*g[2]; // ∂α/∂ψ

      arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
      kalmanCoreScalarUpdate(this, &H, error, sweepInfo->stdDev);
    }
  }
}
