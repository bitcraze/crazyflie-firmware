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
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 */

#include "kalman_core.h"
#include "cfassert.h"
#include "autoconf.h"

#include "physicalConstants.h"

#include "math3d.h"
#include "static_mem.h"

// #define DEBUG_STATE_CHECK

// the reversion of pitch and roll to zero
#ifdef CONFIG_DECK_LOCO_2D_POSITION
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif


/**
 * Supporting and utility functions
 */

#ifdef DEBUG_STATE_CHECK
static void assertStateNotNaN(const kalmanCoreData_t* this) {
  if ((isnan(this->S[KC_STATE_X])) ||
      (isnan(this->S[KC_STATE_Y])) ||
      (isnan(this->S[KC_STATE_Z])) ||
      (isnan(this->S[KC_STATE_PX])) ||
      (isnan(this->S[KC_STATE_PY])) ||
      (isnan(this->S[KC_STATE_PZ])) ||
      (isnan(this->S[KC_STATE_D0])) ||
      (isnan(this->S[KC_STATE_D1])) ||
      (isnan(this->S[KC_STATE_D2])) ||
      (isnan(this->q[0])) ||
      (isnan(this->q[1])) ||
      (isnan(this->q[2])) ||
      (isnan(this->q[3])))
  {
    ASSERT(false);
  }

  for(int i=0; i<KC_STATE_DIM; i++) {
    for(int j=0; j<KC_STATE_DIM; j++)
    {
      if (isnan(this->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertStateNotNaN(const kalmanCoreData_t* this)
{
  return;
}
#endif


// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Small number epsilon, to prevent dividing by zero
#define EPS (1e-6f)

void kalmanCoreDefaultParams(kalmanCoreParams_t* params)
{
  // Initial variances, uncertain of position, but know we're stationary and roughly flat
  params->stdDevInitialPosition_xy = 100;
  params->stdDevInitialPosition_z = 1;
  params->stdDevInitialVelocity = 0.01;
  params->stdDevInitialAttitude_rollpitch = 0.01;
  params->stdDevInitialAttitude_yaw = 0.01;

  params->procNoiseAcc_xy = 0.5f;
  params->procNoiseAcc_z = 1.0f;
  params->procNoiseVel = 0;
  params->procNoisePos = 0;
  params->procNoiseAtt = 0;
  params->measNoiseBaro = 2.0f;           // meters
  params->measNoiseGyro_rollpitch = 0.1f; // radians per second
  params->measNoiseGyro_yaw = 0.1f;       // radians per second

  params->initialX = 0.0;
  params->initialY = 0.0;
  params->initialZ = 0.0;

  // Initial yaw of the Crazyflie in radians.
  // 0 --- facing positive X
  // PI / 2 --- facing positive Y
  // PI --- facing negative X
  // 3 * PI / 2 --- facing negative Y
  params->initialYaw = 0.0;
}

void kalmanCoreInit(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs)
{
  // Reset all data to 0 (like upon system reset)
  memset(this, 0, sizeof(kalmanCoreData_t));

  this->S[KC_STATE_X] = params->initialX;
  this->S[KC_STATE_Y] = params->initialY;
  this->S[KC_STATE_Z] = params->initialZ;
//  this->S[KC_STATE_PX] = 0;
//  this->S[KC_STATE_PY] = 0;
//  this->S[KC_STATE_PZ] = 0;
//  this->S[KC_STATE_D0] = 0;
//  this->S[KC_STATE_D1] = 0;
//  this->S[KC_STATE_D2] = 0;

  // reset the attitude quaternion
  this->initialQuaternion[0] = arm_cos_f32(params->initialYaw / 2);
  this->initialQuaternion[1] = 0.0;
  this->initialQuaternion[2] = 0.0;
  this->initialQuaternion[3] = arm_sin_f32(params->initialYaw / 2);
  for (int i = 0; i < 4; i++) { this->q[i] = this->initialQuaternion[i]; }

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { this->R[i][j] = i==j ? 1 : 0; }}

  for (int i=0; i< KC_STATE_DIM; i++) {
    for (int j=0; j < KC_STATE_DIM; j++) {
      this->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  this->P[KC_STATE_X][KC_STATE_X]  = powf(params->stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Y][KC_STATE_Y]  = powf(params->stdDevInitialPosition_xy, 2);
  this->P[KC_STATE_Z][KC_STATE_Z]  = powf(params->stdDevInitialPosition_z, 2);

  this->P[KC_STATE_PX][KC_STATE_PX] = powf(params->stdDevInitialVelocity, 2);
  this->P[KC_STATE_PY][KC_STATE_PY] = powf(params->stdDevInitialVelocity, 2);
  this->P[KC_STATE_PZ][KC_STATE_PZ] = powf(params->stdDevInitialVelocity, 2);

  this->P[KC_STATE_D0][KC_STATE_D0] = powf(params->stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] = powf(params->stdDevInitialAttitude_rollpitch, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] = powf(params->stdDevInitialAttitude_yaw, 2);

  this->Pm.numRows = KC_STATE_DIM;
  this->Pm.numCols = KC_STATE_DIM;
  this->Pm.pData = (float*)this->P;

  this->baroReferenceHeight = 0.0;

  this->isUpdated = false;
  this->lastPredictionMs = nowMs;
  this->lastProcessNoiseUpdateMs = nowMs;
}

void kalmanCoreScalarUpdate(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // The Kalman gain as a column vector
  NO_DMA_CCM_SAFE_ZERO_INIT static float K[KC_STATE_DIM];
  static arm_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[KC_STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[KC_STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

  ASSERT(Hm->numRows == 1);
  ASSERT(Hm->numCols == KC_STATE_DIM);

  // ====== INNOVATION COVARIANCE ======

  mat_trans(Hm, &HTm);
  mat_mult(&this->Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<KC_STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  ASSERT(!isnan(HPHR));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  for (int i=0; i<KC_STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    this->S[i] = this->S[i] + K[i] * error; // state update
  }
  assertStateNotNaN(this);

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Km, Hm, &tmpNN1m); // KH
  for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[KC_STATE_DIM*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &this->Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &this->Pm); // (KH - I)*P*(KH - I)'
  assertStateNotNaN(this);
  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);

  this->isUpdated = true;
}

void kalmanCoreUpdateWithPKE(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, arm_matrix_instance_f32 *Km, arm_matrix_instance_f32 *P_w_m, float error)
{
    // kalman filter update with weighted covariance matrix P_w_m, kalman gain Km, and innovation error
    // Temporary matrices for the covariance updates
    static float tmpNN1d[KC_STATE_DIM][KC_STATE_DIM];
    static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, (float *)tmpNN1d};
    for (int i=0; i<KC_STATE_DIM; i++){
        this->S[i] = this->S[i] + Km->pData[i] * error;
    }
    // ====== COVARIANCE UPDATE ====== //
    mat_mult(Km, Hm, &tmpNN1m);                 // KH,  the Kalman Gain and H are the updated Kalman Gain and H
    mat_scale(&tmpNN1m, -1.0f, &tmpNN1m);       //  I-KH
    for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[i][i] = 1.0f + tmpNN1d[i][i]; }
    float Ppo[KC_STATE_DIM][KC_STATE_DIM]={0};
    arm_matrix_instance_f32 Ppom = {KC_STATE_DIM, KC_STATE_DIM, (float *)Ppo};
    mat_mult(&tmpNN1m, P_w_m, &Ppom);          // Pm = (I-KH)*P_w_m
    memcpy(this->P, Ppo, sizeof(this->P));

    assertStateNotNaN(this);

    for (int i=0; i<KC_STATE_DIM; i++) {
        for (int j=i; j<KC_STATE_DIM; j++) {
        float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
        if (isnan(p) || p > MAX_COVARIANCE) {
            this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
        } else if ( i==j && p < MIN_COVARIANCE ) {
            this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
        } else {
            this->P[i][j] = this->P[j][i] = p;
            }
        }
    }
    assertStateNotNaN(this);

    this->isUpdated = true;
}

void kalmanCoreUpdateWithBaro(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float baroAsl, bool quadIsFlying)
{
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  h[KC_STATE_Z] = 1;

  if (!quadIsFlying || this->baroReferenceHeight < 1) {
    //TODO: maybe we could track the zero height as a state. Would be especially useful if UWB anchors had barometers.
    this->baroReferenceHeight = baroAsl;
  }

  float meas = (baroAsl - this->baroReferenceHeight);
  kalmanCoreScalarUpdate(this, &H, meas - this->S[KC_STATE_Z], params->measNoiseBaro);
}

static void predictDt(kalmanCoreData_t* this, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying)
{
  /* Here we discretize (euler forward) and linearise the quadrocopter dynamics in order
   * to push the covariance forward.
   *
   * QUADROCOPTER DYNAMICS (see paper):
   *
   * \dot{x} = R(I + [[d]])p
   * \dot{p} = f/m * e3 - [[\omega]]p - g(I - [[d]])R^-1 e3 //drag negligible
   * \dot{d} = \omega
   *
   * where [[.]] is the cross-product matrix of .
   *       \omega are the gyro measurements
   *       e3 is the column vector [0 0 1]'
   *       I is the identity
   *       R is the current attitude as a rotation matrix
   *       f/m is the mass-normalized motor force (acceleration in the body's z direction)
   *       g is gravity
   *       x, p, d are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  // The linearized update matrix
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { KC_STATE_DIM, KC_STATE_DIM, (float *)A}; // linearized dynamics for covariance update;

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  float dt2 = dt*dt;

  // ====== DYNAMICS LINEARIZATION ======
  // Initialize as the identity
  A[KC_STATE_X][KC_STATE_X] = 1;
  A[KC_STATE_Y][KC_STATE_Y] = 1;
  A[KC_STATE_Z][KC_STATE_Z] = 1;

  A[KC_STATE_PX][KC_STATE_PX] = 1;
  A[KC_STATE_PY][KC_STATE_PY] = 1;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1;

  A[KC_STATE_D0][KC_STATE_D0] = 1;
  A[KC_STATE_D1][KC_STATE_D1] = 1;
  A[KC_STATE_D2][KC_STATE_D2] = 1;

  // position from body-frame velocity
  A[KC_STATE_X][KC_STATE_PX] = this->R[0][0]*dt;
  A[KC_STATE_Y][KC_STATE_PX] = this->R[1][0]*dt;
  A[KC_STATE_Z][KC_STATE_PX] = this->R[2][0]*dt;

  A[KC_STATE_X][KC_STATE_PY] = this->R[0][1]*dt;
  A[KC_STATE_Y][KC_STATE_PY] = this->R[1][1]*dt;
  A[KC_STATE_Z][KC_STATE_PY] = this->R[2][1]*dt;

  A[KC_STATE_X][KC_STATE_PZ] = this->R[0][2]*dt;
  A[KC_STATE_Y][KC_STATE_PZ] = this->R[1][2]*dt;
  A[KC_STATE_Z][KC_STATE_PZ] = this->R[2][2]*dt;

  // position from attitude error
  A[KC_STATE_X][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[0][2] - this->S[KC_STATE_PZ]*this->R[0][1])*dt;
  A[KC_STATE_Y][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[1][2] - this->S[KC_STATE_PZ]*this->R[1][1])*dt;
  A[KC_STATE_Z][KC_STATE_D0] = (this->S[KC_STATE_PY]*this->R[2][2] - this->S[KC_STATE_PZ]*this->R[2][1])*dt;

  A[KC_STATE_X][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[0][2] + this->S[KC_STATE_PZ]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[1][2] + this->S[KC_STATE_PZ]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D1] = (- this->S[KC_STATE_PX]*this->R[2][2] + this->S[KC_STATE_PZ]*this->R[2][0])*dt;

  A[KC_STATE_X][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[0][1] - this->S[KC_STATE_PY]*this->R[0][0])*dt;
  A[KC_STATE_Y][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[1][1] - this->S[KC_STATE_PY]*this->R[1][0])*dt;
  A[KC_STATE_Z][KC_STATE_D2] = (this->S[KC_STATE_PX]*this->R[2][1] - this->S[KC_STATE_PY]*this->R[2][0])*dt;

  // body-frame velocity from body-frame velocity
  A[KC_STATE_PX][KC_STATE_PX] = 1; //drag negligible
  A[KC_STATE_PY][KC_STATE_PX] =-gyro->z*dt;
  A[KC_STATE_PZ][KC_STATE_PX] = gyro->y*dt;

  A[KC_STATE_PX][KC_STATE_PY] = gyro->z*dt;
  A[KC_STATE_PY][KC_STATE_PY] = 1; //drag negligible
  A[KC_STATE_PZ][KC_STATE_PY] =-gyro->x*dt;

  A[KC_STATE_PX][KC_STATE_PZ] =-gyro->y*dt;
  A[KC_STATE_PY][KC_STATE_PZ] = gyro->x*dt;
  A[KC_STATE_PZ][KC_STATE_PZ] = 1; //drag negligible

  // body-frame velocity from attitude error
  A[KC_STATE_PX][KC_STATE_D0] =  0;
  A[KC_STATE_PY][KC_STATE_D0] = -GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PZ][KC_STATE_D0] =  GRAVITY_MAGNITUDE*this->R[2][1]*dt;

  A[KC_STATE_PX][KC_STATE_D1] =  GRAVITY_MAGNITUDE*this->R[2][2]*dt;
  A[KC_STATE_PY][KC_STATE_D1] =  0;
  A[KC_STATE_PZ][KC_STATE_D1] = -GRAVITY_MAGNITUDE*this->R[2][0]*dt;

  A[KC_STATE_PX][KC_STATE_D2] = -GRAVITY_MAGNITUDE*this->R[2][1]*dt;
  A[KC_STATE_PY][KC_STATE_D2] =  GRAVITY_MAGNITUDE*this->R[2][0]*dt;
  A[KC_STATE_PZ][KC_STATE_D2] =  0;

  // attitude error from attitude error
  /**
   * At first glance, it may not be clear where the next values come from, since they do not appear directly in the
   * dynamics. In this prediction step, we skip the step of first updating attitude-error, and then incorporating the
   * new error into the current attitude (which requires a rotation of the attitude-error covariance). Instead, we
   * directly update the body attitude, however still need to rotate the covariance, which is what you see below.
   *
   * This comes from a second order approximation to:
   * Sigma_post = exps(-d) Sigma_pre exps(-d)'
   *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
   * where d is the attitude error expressed as Rodriges parameters, ie. d0 = 1/2*gyro.x*dt under the assumption that
   * d = [0,0,0] at the beginning of each prediction step and that gyro.x is constant over the sampling period
   *
   * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
   * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
   */
  float d0 = gyro->x*dt/2;
  float d1 = gyro->y*dt/2;
  float d2 = gyro->z*dt/2;

  A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
  A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

  A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
  A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

  A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
  A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
  A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


  // ====== COVARIANCE UPDATE ======
  mat_mult(&Am, &this->Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &this->Pm); // A P A'
  // Process noise is added after the return from the prediction step

  // ====== PREDICTION STEP ======
  // The prediction depends on whether we're on the ground, or in flight.
  // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)

  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;
  float zacc;

  if (quadIsFlying) // only acceleration in z direction
  {
    // Use accelerometer and not commanded thrust, as this has proper physical units
    zacc = acc->z;

    // position updates in the body frame (will be rotated to inertial frame)
    dx = this->S[KC_STATE_PX] * dt;
    dy = this->S[KC_STATE_PY] * dt;
    dz = this->S[KC_STATE_PZ] * dt + zacc * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    this->S[KC_STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }
  else // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
  {
    // position updates in the body frame (will be rotated to inertial frame)
    dx = this->S[KC_STATE_PX] * dt + acc->x * dt2 / 2.0f;
    dy = this->S[KC_STATE_PY] * dt + acc->y * dt2 / 2.0f;
    dz = this->S[KC_STATE_PZ] * dt + acc->z * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    this->S[KC_STATE_X] += this->R[0][0] * dx + this->R[0][1] * dy + this->R[0][2] * dz;
    this->S[KC_STATE_Y] += this->R[1][0] * dx + this->R[1][1] * dy + this->R[1][2] * dz;
    this->S[KC_STATE_Z] += this->R[2][0] * dx + this->R[2][1] * dy + this->R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = this->S[KC_STATE_PX];
    tmpSPY = this->S[KC_STATE_PY];
    tmpSPZ = this->S[KC_STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    this->S[KC_STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][0]);
    this->S[KC_STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * this->R[2][1]);
    this->S[KC_STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * this->R[2][2]);
  }

  // attitude update (rotate by gyroscope), we do this in quaternions
  // this is the gyroscope angular velocity integrated over the sample period
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // compute the quaternion values in [w,x,y,z] order
  float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
  float ca = arm_cos_f32(angle/2.0f);
  float sa = arm_sin_f32(angle/2.0f);
  float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

  float tmpq0;
  float tmpq1;
  float tmpq2;
  float tmpq3;

  // rotate the quad's attitude by the delta quaternion vector computed above
  tmpq0 = dq[0]*this->q[0] - dq[1]*this->q[1] - dq[2]*this->q[2] - dq[3]*this->q[3];
  tmpq1 = dq[1]*this->q[0] + dq[0]*this->q[1] + dq[3]*this->q[2] - dq[2]*this->q[3];
  tmpq2 = dq[2]*this->q[0] - dq[3]*this->q[1] + dq[0]*this->q[2] + dq[1]*this->q[3];
  tmpq3 = dq[3]*this->q[0] + dq[2]*this->q[1] - dq[1]*this->q[2] + dq[0]*this->q[3];

  if (! quadIsFlying) {
    float keep = 1.0f - ROLLPITCH_ZERO_REVERSION;

    tmpq0 = keep * tmpq0 + ROLLPITCH_ZERO_REVERSION * this->initialQuaternion[0];
    tmpq1 = keep * tmpq1 + ROLLPITCH_ZERO_REVERSION * this->initialQuaternion[1];
    tmpq2 = keep * tmpq2 + ROLLPITCH_ZERO_REVERSION * this->initialQuaternion[2];
    tmpq3 = keep * tmpq3 + ROLLPITCH_ZERO_REVERSION * this->initialQuaternion[3];
  }

  // normalize and store the result
  float norm = arm_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3) + EPS;
  this->q[0] = tmpq0/norm; this->q[1] = tmpq1/norm; this->q[2] = tmpq2/norm; this->q[3] = tmpq3/norm;
  assertStateNotNaN(this);

  this->isUpdated = true;
}

void kalmanCorePredict(kalmanCoreData_t* this, Axis3f *acc, Axis3f *gyro, const uint32_t nowMs, bool quadIsFlying) {
  float dt = (nowMs - this->lastPredictionMs) / 1000.0f;
  predictDt(this, acc, gyro, dt, quadIsFlying);
  this->lastPredictionMs = nowMs;
}


static void addProcessNoiseDt(kalmanCoreData_t *this, const kalmanCoreParams_t *params, float dt)
{
  this->P[KC_STATE_X][KC_STATE_X] += powf(params->procNoiseAcc_xy*dt*dt + params->procNoiseVel*dt + params->procNoisePos, 2);  // add process noise on position
  this->P[KC_STATE_Y][KC_STATE_Y] += powf(params->procNoiseAcc_xy*dt*dt + params->procNoiseVel*dt + params->procNoisePos, 2);  // add process noise on position
  this->P[KC_STATE_Z][KC_STATE_Z] += powf(params->procNoiseAcc_z*dt*dt + params->procNoiseVel*dt + params->procNoisePos, 2);  // add process noise on position

  this->P[KC_STATE_PX][KC_STATE_PX] += powf(params->procNoiseAcc_xy*dt + params->procNoiseVel, 2); // add process noise on velocity
  this->P[KC_STATE_PY][KC_STATE_PY] += powf(params->procNoiseAcc_xy*dt + params->procNoiseVel, 2); // add process noise on velocity
  this->P[KC_STATE_PZ][KC_STATE_PZ] += powf(params->procNoiseAcc_z*dt + params->procNoiseVel, 2); // add process noise on velocity

  this->P[KC_STATE_D0][KC_STATE_D0] += powf(params->measNoiseGyro_rollpitch * dt + params->procNoiseAtt, 2);
  this->P[KC_STATE_D1][KC_STATE_D1] += powf(params->measNoiseGyro_rollpitch * dt + params->procNoiseAtt, 2);
  this->P[KC_STATE_D2][KC_STATE_D2] += powf(params->measNoiseGyro_yaw * dt + params->procNoiseAtt, 2);

  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);
}

void kalmanCoreAddProcessNoise(kalmanCoreData_t *this, const kalmanCoreParams_t *params, const uint32_t nowMs) {
  float dt = (nowMs - this->lastProcessNoiseUpdateMs) / 1000.0f;
  if (dt > 0.0f) {
    addProcessNoiseDt(this, params, dt);
    this->lastProcessNoiseUpdateMs = nowMs;
  }
}

bool kalmanCoreFinalize(kalmanCoreData_t* this)
{
  // Only finalize if data is updated
  if (! this->isUpdated) {
    return false;
  }


  // Matrix to rotate the attitude covariances once updated
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  static arm_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float *)A};

  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  // Incorporate the attitude error (Kalman filter state) with the attitude
  float v0 = this->S[KC_STATE_D0];
  float v1 = this->S[KC_STATE_D1];
  float v2 = this->S[KC_STATE_D2];

  // Move attitude error into attitude if any of the angle errors are large enough
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
    float ca = arm_cos_f32(angle / 2.0f);
    float sa = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

    // rotate the quad's attitude by the delta quaternion vector computed above
    float tmpq0 = dq[0] * this->q[0] - dq[1] * this->q[1] - dq[2] * this->q[2] - dq[3] * this->q[3];
    float tmpq1 = dq[1] * this->q[0] + dq[0] * this->q[1] + dq[3] * this->q[2] - dq[2] * this->q[3];
    float tmpq2 = dq[2] * this->q[0] - dq[3] * this->q[1] + dq[0] * this->q[2] + dq[1] * this->q[3];
    float tmpq3 = dq[3] * this->q[0] + dq[2] * this->q[1] - dq[1] * this->q[2] + dq[0] * this->q[3];

    // normalize and store the result
    float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPS;
    this->q[0] = tmpq0 / norm;
    this->q[1] = tmpq1 / norm;
    this->q[2] = tmpq2 / norm;
    this->q[3] = tmpq3 / norm;

    /** Rotate the covariance, since we've rotated the body
     *
     * This comes from a second order approximation to:
     * Sigma_post = exps(-d) Sigma_pre exps(-d)'
     *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
     * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
     *
     * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
     * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
     */

    float d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
    float d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
    float d2 = v2/2;

    A[KC_STATE_X][KC_STATE_X] = 1;
    A[KC_STATE_Y][KC_STATE_Y] = 1;
    A[KC_STATE_Z][KC_STATE_Z] = 1;

    A[KC_STATE_PX][KC_STATE_PX] = 1;
    A[KC_STATE_PY][KC_STATE_PY] = 1;
    A[KC_STATE_PZ][KC_STATE_PZ] = 1;

    A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
    A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

    A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
    A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

    A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
    A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
    A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&Am, &tmpNN1m); // A'
    mat_mult(&Am, &this->Pm, &tmpNN2m); // AP
    mat_mult(&tmpNN2m, &tmpNN1m, &this->Pm); //APA'
  }

  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  this->R[0][0] = this->q[0] * this->q[0] + this->q[1] * this->q[1] - this->q[2] * this->q[2] - this->q[3] * this->q[3];
  this->R[0][1] = 2 * this->q[1] * this->q[2] - 2 * this->q[0] * this->q[3];
  this->R[0][2] = 2 * this->q[1] * this->q[3] + 2 * this->q[0] * this->q[2];

  this->R[1][0] = 2 * this->q[1] * this->q[2] + 2 * this->q[0] * this->q[3];
  this->R[1][1] = this->q[0] * this->q[0] - this->q[1] * this->q[1] + this->q[2] * this->q[2] - this->q[3] * this->q[3];
  this->R[1][2] = 2 * this->q[2] * this->q[3] - 2 * this->q[0] * this->q[1];

  this->R[2][0] = 2 * this->q[1] * this->q[3] - 2 * this->q[0] * this->q[2];
  this->R[2][1] = 2 * this->q[2] * this->q[3] + 2 * this->q[0] * this->q[1];
  this->R[2][2] = this->q[0] * this->q[0] - this->q[1] * this->q[1] - this->q[2] * this->q[2] + this->q[3] * this->q[3];

  // reset the attitude error
  this->S[KC_STATE_D0] = 0;
  this->S[KC_STATE_D1] = 0;
  this->S[KC_STATE_D2] = 0;

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<KC_STATE_DIM; i++) {
    for (int j=i; j<KC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertStateNotNaN(this);

  this->isUpdated = false;
  return true;
}

void kalmanCoreExternalizeState(const kalmanCoreData_t* this, state_t *state, const Axis3f *acc)
{
  // position state is already in world frame
  state->position = (point_t){
      .x = this->S[KC_STATE_X],
      .y = this->S[KC_STATE_Y],
      .z = this->S[KC_STATE_Z]
  };

  // velocity is in body frame and needs to be rotated to world frame
  state->velocity = (velocity_t){
      .x = this->R[0][0]*this->S[KC_STATE_PX] + this->R[0][1]*this->S[KC_STATE_PY] + this->R[0][2]*this->S[KC_STATE_PZ],
      .y = this->R[1][0]*this->S[KC_STATE_PX] + this->R[1][1]*this->S[KC_STATE_PY] + this->R[1][2]*this->S[KC_STATE_PZ],
      .z = this->R[2][0]*this->S[KC_STATE_PX] + this->R[2][1]*this->S[KC_STATE_PY] + this->R[2][2]*this->S[KC_STATE_PZ]
  };

  // Accelerometer measurements are in the body frame and need to be rotated to world frame.
  // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
  // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
  state->acc = (acc_t){
      .x = this->R[0][0]*acc->x + this->R[0][1]*acc->y + this->R[0][2]*acc->z,
      .y = this->R[1][0]*acc->x + this->R[1][1]*acc->y + this->R[1][2]*acc->z,
      .z = this->R[2][0]*acc->x + this->R[2][1]*acc->y + this->R[2][2]*acc->z - 1
  };

  // convert the new attitude into Euler YPR
  float yaw = atan2f(2*(this->q[1]*this->q[2]+this->q[0]*this->q[3]) , this->q[0]*this->q[0] + this->q[1]*this->q[1] - this->q[2]*this->q[2] - this->q[3]*this->q[3]);
  float pitch = asinf(-2*(this->q[1]*this->q[3] - this->q[0]*this->q[2]));
  float roll = atan2f(2*(this->q[2]*this->q[3]+this->q[0]*this->q[1]) , this->q[0]*this->q[0] - this->q[1]*this->q[1] - this->q[2]*this->q[2] + this->q[3]*this->q[3]);

  // Save attitude, adjusted for the legacy CF2 body coordinate system
  state->attitude = (attitude_t){
      .roll = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw = yaw*RAD_TO_DEG
  };

  // Save quaternion, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  state->attitudeQuaternion = (quaternion_t){
      .w = this->q[0],
      .x = this->q[1],
      .y = this->q[2],
      .z = this->q[3]
  };

  assertStateNotNaN(this);
}

// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(kalmanCoreData_t* this, kalmanCoreStateIdx_t state)
{
  // Set all covariance to 0
  for(int i=0; i<KC_STATE_DIM; i++) {
    this->P[state][i] = 0;
    this->P[i][state] = 0;
  }
  // Set state variance to maximum
  this->P[state][state] = MAX_COVARIANCE;
  // set state to zero
  this->S[state] = 0;
}

void kalmanCoreDecoupleXY(kalmanCoreData_t* this)
{
  decoupleState(this, KC_STATE_X);
  decoupleState(this, KC_STATE_PX);
  decoupleState(this, KC_STATE_Y);
  decoupleState(this, KC_STATE_PY);
}
