/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D'Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D'Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 */

#pragma once

#include "cf_math.h"
#include "stabilizer_types.h"

// Indexes to access the quad's state, stored as a column vector
typedef enum
{
  KC_STATE_X, KC_STATE_Y, KC_STATE_Z, KC_STATE_PX, KC_STATE_PY, KC_STATE_PZ, KC_STATE_D0, KC_STATE_D1, KC_STATE_D2, KC_STATE_DIM
} kalmanCoreStateIdx_t;


// The data used by the kalman core implementation.
typedef struct {
  /**
   * Quadrocopter State
   *
   * The internally-estimated state is:
   * - X, Y, Z: the quad's position in the global frame
   * - PX, PY, PZ: the quad's velocity in its body frame
   * - D0, D1, D2: attitude error
   *
   * For more information, refer to the paper
   */
  float S[KC_STATE_DIM];

  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  float q[4];

  // The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  float R[3][3];

  // The covariance matrix
  __attribute__((aligned(4))) float P[KC_STATE_DIM][KC_STATE_DIM];
  arm_matrix_instance_f32 Pm;

  float baroReferenceHeight;

  // Quaternion used for initial orientation [w,x,y,z]
  float initialQuaternion[4];

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  bool isUpdated;

  uint32_t lastPredictionMs;
  uint32_t lastProcessNoiseUpdateMs;
} kalmanCoreData_t;

// The parameters used by the filter
typedef struct {
  // Initial variances, uncertain of position, but know we're stationary and roughly flat
  float stdDevInitialPosition_xy;
  float stdDevInitialPosition_z;
  float stdDevInitialVelocity;
  float stdDevInitialAttitude_rollpitch;
  float stdDevInitialAttitude_yaw;

  float procNoiseAcc_xy;
  float procNoiseAcc_z;
  float procNoiseVel;
  float procNoisePos;
  float procNoiseAtt;
  float measNoiseBaro;           // meters
  float measNoiseGyro_rollpitch; // radians per second
  float measNoiseGyro_yaw;       // radians per second

  float initialX;
  float initialY;
  float initialZ;

  // Initial yaw of the Crazyflie in radians.
  // 0 --- facing positive X
  // PI / 2 --- facing positive Y
  // PI --- facing negative X
  // 3 * PI / 2 --- facing negative Y
  float initialYaw;
} kalmanCoreParams_t;

/*  - Load default parameters */
void kalmanCoreDefaultParams(kalmanCoreParams_t *params);

/*  - Initialize Kalman State */
void kalmanCoreInit(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs);

/*  - Measurement updates based on sensors */

// Barometer
void kalmanCoreUpdateWithBaro(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float baroAsl, bool quadIsFlying);

/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
void kalmanCorePredict(kalmanCoreData_t *this, Axis3f *acc, Axis3f *gyro, const uint32_t nowMs, bool quadIsFlying);

void kalmanCoreAddProcessNoise(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs);

/**
 * @brief Finalization to incorporate attitude error into body attitude
 *
 * @param this Core data
 * @return true The state was finalized
 * @return false The state was not changed and did not require finalization
 */
bool kalmanCoreFinalize(kalmanCoreData_t* this);

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc);

void kalmanCoreDecoupleXY(kalmanCoreData_t* this);

void kalmanCoreScalarUpdate(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise);

void kalmanCoreUpdateWithPKE(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, arm_matrix_instance_f32 *Km, arm_matrix_instance_f32 *P_w_m, float error);
