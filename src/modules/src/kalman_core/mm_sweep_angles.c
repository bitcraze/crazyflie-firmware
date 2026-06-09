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
  // Relative sensor position in global reference frame
  vec3d sensor_offset_global;
  arm_matrix_instance_f32 temp_ps_ = {3, 1, sensor_offset_global};
  // Rotation matrix from Crazyflie to global reference frame
  arm_matrix_instance_f32 Rcf_ = {3, 3, (float32_t *)this->R};
  // Relative sensor position in Crazyflie reference frame
  arm_matrix_instance_f32 scf_ = {3, 1, (float32_t *)*sweepInfo->sensorPos};

  // Rotate the relative sensor position to the global reference frame
  mat_mult(&Rcf_, &scf_, &temp_ps_);

  // Sensor position in global reference frame
  // Gets the current state values of the position of the Crazyflie in the global reference frame and add the relative sensor pos
  vec3d ps = {this->S[KC_STATE_X] + sensor_offset_global[0], this->S[KC_STATE_Y] + sensor_offset_global[1], this->S[KC_STATE_Z] + sensor_offset_global[2]};

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
    const float qDen = r2 - z_tan_t * z_tan_t;
    // Avoid singularity
    if (qDen > 0.0001f) {
      // Position Jacobians: ∂α/∂x, ∂α/∂y, ∂α/∂z, where x y and z are the sensor position in the rotor reference frame

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

      // Let δφ, δθ, δψ be the orientation error states (KC_STATE_D0/D1/D2).
      // Then, our orientation Jacobians are: ∂α/∂(δφ), ∂α/∂(δθ), ∂α/∂(δψ)
      //
      // Applying the chain rule (e.g. for ∂α/∂(δφ)), we get:
      // ∂α/∂(δφ) = ∂α/∂X * ∂X/∂(δφ) + ∂α/∂Y * ∂Y/∂(δφ) + ∂α/∂Z * ∂Z/∂(δφ)
      //
      // We know ∂α/∂X, ∂α/∂Y, ∂α/∂Z from the position Jacobians (g, global frame).
      // We still need the partial derivatives of the global sensor position w.r.t the
      // orientation error, ∂(ps)/∂(δφ), ∂(ps)/∂(δθ), ∂(ps)/∂(δψ).
      //
      // The orientation error is a *body-frame* perturbation of the attitude: R = R0 * R(δ)
      // (this is the convention used by kalmanCoreFinalize, and by the "position from
      // attitude error" terms in the predict step, where ∂(R·u)/∂(δ_i) = R·(e_i × u)).
      // So the small rotation acts on the body-frame sensor offset s_cf *before* it is
      // mapped to the global frame by R. Using the principal-axis rotation matrices:
      //
      //        [1    0        0   ]            [cos(θ)   0   sin(θ)]            [cos(ψ)  -sin(ψ)  0]
      // R(φ) = [0  cos(φ)  -sin(φ)]    R(θ) =  [  0      1     0   ]    R(ψ) =  [sin(ψ)   cos(ψ)  0]
      //        [0  sin(φ)   cos(φ)]            [-sin(θ)  0   cos(θ)]            [  0        0     1]
      //
      // differentiating R(δ)·s_cf at δ = 0 gives (e_i × s_cf) per axis, then R maps it
      // to the global frame:  ∂(ps)/∂(δ_i) = R · (e_i × s_cf), with s_cf = (sx, sy, sz):
      //   e_x × s_cf = (0, -sz, sy)
      //   e_y × s_cf = (sz, 0, -sx)
      //   e_z × s_cf = (-sy, sx, 0)
      const float sx = (*sweepInfo->sensorPos)[0];
      const float sy = (*sweepInfo->sensorPos)[1];
      const float sz = (*sweepInfo->sensorPos)[2];

      vec3d dps_droll  = {sy*this->R[0][2] - sz*this->R[0][1],   // ∂(ps)/∂(δφ) = R·(0,-sz,sy)
                          sy*this->R[1][2] - sz*this->R[1][1],
                          sy*this->R[2][2] - sz*this->R[2][1]};
      vec3d dps_dpitch = {sz*this->R[0][0] - sx*this->R[0][2],   // ∂(ps)/∂(δθ) = R·(sz,0,-sx)
                          sz*this->R[1][0] - sx*this->R[1][2],
                          sz*this->R[2][0] - sx*this->R[2][2]};
      vec3d dps_dyaw   = {sx*this->R[0][1] - sy*this->R[0][0],   // ∂(ps)/∂(δψ) = R·(-sy,sx,0)
                          sx*this->R[1][1] - sy*this->R[1][0],
                          sx*this->R[2][1] - sy*this->R[2][0]};

      // Update the H vector with Orientation Jacobians: h[D_i] = g · ∂(ps)/∂(δ_i)
      h[KC_STATE_D0] = g[0]*dps_droll[0]  + g[1]*dps_droll[1]  + g[2]*dps_droll[2];
      h[KC_STATE_D1] = g[0]*dps_dpitch[0] + g[1]*dps_dpitch[1] + g[2]*dps_dpitch[2];
      h[KC_STATE_D2] = g[0]*dps_dyaw[0]   + g[1]*dps_dyaw[1]   + g[2]*dps_dyaw[2];

      arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
      kalmanCoreScalarUpdate(this, &H, error, sweepInfo->stdDev);
    }
  }
}
