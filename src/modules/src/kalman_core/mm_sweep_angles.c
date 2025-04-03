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
  // In order to find the sensor's position in the global reference frame, we first calculate the
  // sensor's temporary position in the global reference frame due to Crazyflie's rotation.
  // Then, we add the result to the Crazyflie's global position.

  // Temporary sensor position in global reference frame due to the rotation of the Cf.
  vec3d temp_ps;
  arm_matrix_instance_f32 temp_ps_ = {3, 1, temp_ps};
  // Rotation matrix from Crazyflie to global reference frame
  arm_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)this->R};
  // Sensor position in Crazyflie reference frame
  arm_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sweepInfo->sensorPos};
  // Rotate the temporary sensor position to the global reference frame
  mat_mult(&Rcf_, &scf_, &temp_ps_);

  // Crazyflie position in global reference frame
  vec3d pcf = {this->S[KC_STATE_X], this->S[KC_STATE_Y], this->S[KC_STATE_Z]};

  // Sensor position in global reference frame
  vec3d ps = {pcf[0] + temp_ps[0], pcf[1] + temp_ps[1], pcf[2] + temp_ps[2]};
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
    // Calculate H vector that contains the Position and Orientation Jacobians in the global reference frame
    const float z_tan_t = z * tan_t;
    const float qDen = r2 - z_tan_t * z_tan_t;
    // Avoid singularity
    if (qDen > 0.0001f) {
      // Position Jacobians: ∂α/∂X, ∂α/∂Y, ∂α/∂Z, where X, Y, Z are the sensor position in the global reference frame

      const float q = tan_t / arm_sqrt(qDen);

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

      // Update the H vector with Position Jacobians (in the global reference frame)
      float h[KC_STATE_DIM] = {0};
      h[KC_STATE_X] = g[0]; // ∂α/∂X
      h[KC_STATE_Y] = g[1]; // ∂α/∂Y
      h[KC_STATE_Z] = g[2]; // ∂α/∂Z

      // Let δφ, δθ, δψ be the orientation errors (small changes in orientation) in the global reference frame
      // Then, our orientation Jacobians are: ∂α/∂(δφ), ∂α/∂(δθ), ∂α/∂(δψ)
      //
      // Applying the chain rule (e.g. for ∂α/∂(∂φ)), we get:
      // ∂α/∂(∂φ) = ∂α/∂X * ∂X/∂(δφ) + ∂α/∂Y * ∂Y/∂(δφ) + ∂α/∂Z * ∂Z/∂(δφ)
      //
      // We know ∂α/∂X, ∂α/∂Y, ∂α/∂Z from the position Jacobians
      // We still need the partial derivatives of position w.r.t the orientation error
      //
      // We need ∂X/∂(δφ), ∂Y/∂(δφ), ∂Z/∂(δφ), ∂X/∂(δθ), ∂Y/∂(δθ), ∂Z/∂(δθ), ∂X/∂(δψ), ∂Y/∂(δψ), ∂Z/∂(δψ)
      // These can be derived from the rotation matrices.
      // Let's fall back to the Crazyflie body frame for the orientation error, because we know how to derive the rotation matrices in that frame
      //
      //        [1    0        0   ]
      // R(φ) = [0  cos(φ)  -sin(φ)]
      //        [0  sin(φ)   cos(φ)]
      //
      //        [cos(Θ)   0   sin(θ)]
      // R(θ) = [  0      1     0   ]
      //        [-sin(θ)  0   cos(θ)]
      //
      //        [cos(ψ)  -sin(ψ)  0]
      // R(ψ) = [sin(ψ)   cos(ψ)  0]
      //        [  0        0     1]

      float dx_droll = 0.0f; // ∂X/∂(δφ)
      float dy_droll = -ps[2]; // ∂Y/∂(δφ)
      float dz_droll = ps[1]; // ∂Z/∂(δφ)

      float dx_dpitch = -ps[2]; // ∂X/∂(δθ)
      float dy_dpitch = 0.0f; // ∂Y/∂(δθ)
      float dz_dpitch = ps[0]; // ∂Z/∂(δθ)

      float dx_dyaw = -ps[1]; // ∂X/∂(δψ)
      float dy_dyaw = ps[0]; // ∂Y/∂(δψ)
      float dz_dyaw = 0.0f; // ∂Z/∂(δψ)

      // Update the H vector with Orientation Jacobians (in the global reference frame)
      h[KC_STATE_D0] = g[0]*dx_droll + g[1]*dy_droll + g[2]*dz_droll;
      h[KC_STATE_D1] = g[0]*dx_dpitch + g[1]*dy_dpitch + g[2]*dz_dpitch;
      h[KC_STATE_D2] = g[0]*dx_dyaw + g[1]*dy_dyaw + g[2]*dz_dyaw;

      arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
      kalmanCoreScalarUpdate(this, &H, error, sweepInfo->stdDev);
    }
  }
}
