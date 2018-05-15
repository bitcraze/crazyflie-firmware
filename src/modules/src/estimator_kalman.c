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
 *
 */

#include "estimator_kalman.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "sensors.h"

#include "log.h"
#include "param.h"

#include "math.h"
#include "arm_math.h"

//#define KALMAN_USE_BARO_UPDATE
//#define KALMAN_NAN_CHECK


/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
static void stateEstimatorPredict(float thrust, Axis3f *acc, Axis3f *gyro, float dt);
static void stateEstimatorAddProcessNoise(float dt);

/*  - Measurement updates based on sensors */
static void stateEstimatorScalarUpdate(arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise);
static void stateEstimatorUpdateWithAccOnGround(Axis3f *acc);
#ifdef KALMAN_USE_BARO_UPDATE
static void stateEstimatorUpdateWithBaro(baro_t *baro);
#endif

/*  - Finalization to incorporate attitude error into body attitude */
static void stateEstimatorFinalize(sensorData_t *sensors, uint32_t tick);

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
static void stateEstimatorExternalizeState(state_t *state, sensorData_t *sensors, uint32_t tick);


/**
 * Additionally, the filter supports the incorporation of additional sensors into the state estimate
 *
 * This is done via the external functions:
 * - bool estimatorKalmanEnqueueUWBPacket(uwbPacket_t *uwb)
 * - bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
 * - bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
 *
 * As well as by the following internal functions and datatypes
 */

// Distance-to-point measurements
static xQueueHandle distDataQueue;
#define DIST_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithDistance(distanceMeasurement_t *dist);

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Direct measurements of Crazyflie position
static xQueueHandle posDataQueue;
#define POS_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithPosition(positionMeasurement_t *pos);

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Measurements of a UWB Tx/Rx
static xQueueHandle tdoaDataQueue;
#define UWB_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithTDOA(tdoaMeasurement_t *uwb);

static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb) {
  return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}


// Measurements of flow (dnx, dny)
static xQueueHandle flowDataQueue;
#define FLOW_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithFlow(flowMeasurement_t *flow, sensorData_t *sensors);

static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow) {
  return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}

// Measurements of TOF from laser sensor
static xQueueHandle tofDataQueue;
#define TOF_QUEUE_LENGTH (10)

static void stateEstimatorUpdateWithTof(tofMeasurement_t *tof);

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

/**
 * Constants used in the estimator
 */

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define GRAVITY_MAGNITUDE (9.81f) // we use the magnitude such that the sign/direction is explicit in calculations
#define CRAZYFLIE_WEIGHT_grams (27.0f)

//thrust is thrust mapped for 65536 <==> 60 GRAMS!
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/CRAZYFLIE_WEIGHT_grams/65536.0f)

#define SPEED_OF_LIGHT (299792458)

// TODO: Decouple the TDOA implementation from the Kalman filter...
#define METERS_PER_TDOATICK (4.691763979e-3f)
#define SECONDS_PER_TDOATICK (15.650040064e-12f)


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

// the point at which the dynamics change from stationary to flying
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE*0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

// the reversion of pitch and roll to zero
#define ROLLPITCH_ZERO_REVERSION (0.001f)

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// The bounds on states, these shouldn't be hit...
#define MAX_POSITION (100) //meters
#define MAX_VELOCITY (10) //meters per second

// Initial variances, uncertain of position, but know we're stationary and roughly flat
static const float stdDevInitialPosition_xy = 100;
static const float stdDevInitialPosition_z = 1;
static const float stdDevInitialVelocity = 0.01;
static const float stdDevInitialAttitude_rollpitch = 0.01;
static const float stdDevInitialAttitude_yaw = 0.01;

static float procNoiseAcc_xy = 0.5f;
static float procNoiseAcc_z = 1.0f;
static float procNoiseVel = 0;
static float procNoisePos = 0;
static float procNoiseAtt = 0;
static float measNoiseBaro = 2.0f; // meters
static float measNoiseGyro_rollpitch = 0.1f; // radians per second
static float measNoiseGyro_yaw = 0.1f; // radians per second

static float initialX = 0.5;
static float initialY = 0.5;
static float initialZ = 0.0;

// We track a TDOA skew as part of the Kalman filter
static const float stdDevInitialSkew = 0.1;
static float procNoiseSkew = 10e-6f; // seconds per second^2 (is multiplied by dt to give skew noise)

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 * - SKEW: the skew from anchor system clock to quad clock
 *
 * For more information, refer to the paper
 */

// The quad's state, stored as a column vector
typedef enum
{
  STATE_X, STATE_Y, STATE_Z, STATE_PX, STATE_PY, STATE_PZ, STATE_D0, STATE_D1, STATE_D2, STATE_DIM
} stateIdx_t;

static float S[STATE_DIM];

// The quad's attitude as a quaternion (w,x,y,z)
// We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
// while also being robust against singularities (in comparison to euler angles)
static float q[4] = {1,0,0,0};

// The quad's attitude as a rotation matrix (used by the prediction, updated by the finalization)
static float R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

// The covariance matrix
static float P[STATE_DIM][STATE_DIM];
static arm_matrix_instance_f32 Pm = {STATE_DIM, STATE_DIM, (float *)P};


/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;
static bool resetEstimation = true;
static int32_t lastPrediction;
static int32_t lastBaroUpdate;
static int32_t lastPNUpdate;
static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
static baro_t baroAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static bool quadIsFlying = false;
static int32_t lastTDOAUpdate;
static float stateSkew;
static float varSkew;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;
static uint32_t tdoaCount;

/**
 * Supporting and utility functions
 */

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

#ifdef KALMAN_NAN_CHECK
static void stateEstimatorAssertNotNaN() {
  if ((isnan(S[STATE_X])) ||
      (isnan(S[STATE_Y])) ||
      (isnan(S[STATE_Z])) ||
      (isnan(S[STATE_PX])) ||
      (isnan(S[STATE_PY])) ||
      (isnan(S[STATE_PZ])) ||
      (isnan(S[STATE_D0])) ||
      (isnan(S[STATE_D1])) ||
      (isnan(S[STATE_D2])) ||
      (isnan(q[0])) ||
      (isnan(q[1])) ||
      (isnan(q[2])) ||
      (isnan(q[3]))) { resetEstimation = true; }

  for(int i=0; i<STATE_DIM; i++) {
    for(int j=0; j<STATE_DIM; j++)
    {
      if (isnan(P[i][j]))
      {
        resetEstimation = true;
      }
    }
  }
}
#else
static void stateEstimatorAssertNotNaN()
{
  return;
}
#endif

#ifdef KALMAN_DECOUPLE_XY
// Reset a state to 0 with max covariance
// If called often, this decouples the state to the rest of the filter
static void decoupleState(stateIdx_t state)
{
  // Set all covariance to 0
  for(int i=0; i<STATE_DIM; i++) {
    P[state][i] = 0;
    P[i][state] = 0;
  }
  // Set state variance to maximum
  P[state][state] = MAX_COVARIANCE;
  // set state to zero
  S[state] = 0;
}
#endif

// --------------------------------------------------


void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
  // If the client (via a parameter update) triggers an estimator reset:
  if (resetEstimation) { estimatorKalmanInit(); resetEstimation = false; }

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  bool doneUpdate = false;

  uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

#ifdef KALMAN_DECOUPLE_XY
  // Decouple position states
  decoupleState(STATE_X);
  decoupleState(STATE_PX);
  decoupleState(STATE_Y);
  decoupleState(STATE_PY);
#endif

  // Average the last IMU measurements. We do this because the prediction loop is
  // slower than the IMU loop, but the IMU information is required externally at
  // a higher rate (for body rate control).
  if (sensorsReadAcc(&sensors->acc)) {
    accAccumulator.x += GRAVITY_MAGNITUDE*sensors->acc.x; // accelerometer is in Gs
    accAccumulator.y += GRAVITY_MAGNITUDE*sensors->acc.y; // but the estimator requires ms^-2
    accAccumulator.z += GRAVITY_MAGNITUDE*sensors->acc.z;
    accAccumulatorCount++;
  }

  if (sensorsReadGyro(&sensors->gyro)) {
    gyroAccumulator.x += sensors->gyro.x * DEG_TO_RAD; // gyro is in deg/sec
    gyroAccumulator.y += sensors->gyro.y * DEG_TO_RAD; // but the estimator requires rad/sec
    gyroAccumulator.z += sensors->gyro.z * DEG_TO_RAD;
    gyroAccumulatorCount++;
  }

  if (sensorsReadMag(&sensors->mag)) {
      // Currently the magnetometer doesn't play a part in the estimation
  }

  // Average the thrust command from the last timestep, generated externally by the controller
  thrustAccumulator += control->thrust * CONTROL_TO_ACC; // thrust is in grams, we need ms^-2
  thrustAccumulatorCount++;

  // Run the system dynamics to predict the state forward.
  if ((osTick-lastPrediction) >= configTICK_RATE_HZ/PREDICT_RATE // update at the PREDICT_RATE
      && gyroAccumulatorCount > 0
      && accAccumulatorCount > 0
      && thrustAccumulatorCount > 0)
  {
    gyroAccumulator.x /= gyroAccumulatorCount;
    gyroAccumulator.y /= gyroAccumulatorCount;
    gyroAccumulator.z /= gyroAccumulatorCount;

    accAccumulator.x /= accAccumulatorCount;
    accAccumulator.y /= accAccumulatorCount;
    accAccumulator.z /= accAccumulatorCount;

    thrustAccumulator /= thrustAccumulatorCount;

    float dt = (float)(osTick-lastPrediction)/configTICK_RATE_HZ;
    stateEstimatorPredict(thrustAccumulator, &accAccumulator, &gyroAccumulator, dt);

    if (!quadIsFlying) { // accelerometers give us information about attitude on slanted ground
      stateEstimatorUpdateWithAccOnGround(&accAccumulator);
    }

    lastPrediction = osTick;

    accAccumulator = (Axis3f){.axis={0}};
    accAccumulatorCount = 0;
    gyroAccumulator = (Axis3f){.axis={0}};
    gyroAccumulatorCount = 0;
    thrustAccumulator = 0;
    thrustAccumulatorCount = 0;

    doneUpdate = true;
  }


  /**
   * Add process noise every loop, rather than every prediction
   */
  stateEstimatorAddProcessNoise((float)(osTick-lastPNUpdate)/configTICK_RATE_HZ);
  lastPNUpdate = osTick;



  /**
   * Update the state estimate with the barometer measurements
   */
  // Accumulate the barometer measurements
  if (sensorsReadBaro(&sensors->baro)) {
#ifdef KALMAN_USE_BARO_UPDATE
    baroAccumulator.asl += sensors->baro.asl;
    baroAccumulatorCount++;
  }

  if ((osTick-lastBaroUpdate) >= configTICK_RATE_HZ/BARO_RATE // update at BARO_RATE
      && baroAccumulatorCount > 0)
  {
    baroAccumulator.asl /= baroAccumulatorCount;

    stateEstimatorUpdateWithBaro(&sensors->baro);

    baroAccumulator.asl = 0;
    baroAccumulatorCount = 0;
    lastBaroUpdate = osTick;
    doneUpdate = true;
#endif
  }

  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  tofMeasurement_t tof;
  while (stateEstimatorHasTOFPacket(&tof))
  {
    stateEstimatorUpdateWithTof(&tof);
    doneUpdate = true;
  }

  distanceMeasurement_t dist;
  while (stateEstimatorHasDistanceMeasurement(&dist))
  {
    stateEstimatorUpdateWithDistance(&dist);
    doneUpdate = true;
  }

  positionMeasurement_t pos;
  while (stateEstimatorHasPositionMeasurement(&pos))
  {
    stateEstimatorUpdateWithPosition(&pos);
    doneUpdate = true;
  }

  tdoaMeasurement_t tdoa;
  while (stateEstimatorHasTDOAPacket(&tdoa))
  {
    stateEstimatorUpdateWithTDOA(&tdoa);
    doneUpdate = true;
  }

  flowMeasurement_t flow;
  while (stateEstimatorHasFlowPacket(&flow))
  {
    stateEstimatorUpdateWithFlow(&flow, sensors);
    doneUpdate = true;
  }

  /**
   * If an update has been made, the state is finalized:
   * - the attitude error is moved into the body attitude quaternion,
   * - the body attitude is converted into a rotation matrix for the next prediction, and
   * - correctness of the covariance matrix is ensured
   */

  if (doneUpdate)
  {
    stateEstimatorFinalize(sensors, osTick);
    stateEstimatorAssertNotNaN();
  }

  /**
   * Finally, the internal state is externalized.
   * This is done every round, since the external state includes some sensor data
   */
  stateEstimatorExternalizeState(state, sensors, osTick);
  stateEstimatorAssertNotNaN();
}

static void stateEstimatorPredict(float cmdThrust, Axis3f *acc, Axis3f *gyro, float dt)
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
   *       x, p, d, skew are the quad's states
   * note that d (attitude error) is zero at the beginning of each iteration,
   * since error information is incorporated into R after each Kalman update.
   */

  // The linearized update matrix
  static float A[STATE_DIM][STATE_DIM];
  static arm_matrix_instance_f32 Am = { STATE_DIM, STATE_DIM, (float *)A}; // linearized dynamics for covariance update;

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = { STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = { STATE_DIM, STATE_DIM, tmpNN2d};

  float dt2 = dt*dt;

  // ====== DYNAMICS LINEARIZATION ======
  // Initialize as the identity
  A[STATE_X][STATE_X] = 1;
  A[STATE_Y][STATE_Y] = 1;
  A[STATE_Z][STATE_Z] = 1;

  A[STATE_PX][STATE_PX] = 1;
  A[STATE_PY][STATE_PY] = 1;
  A[STATE_PZ][STATE_PZ] = 1;

  A[STATE_D0][STATE_D0] = 1;
  A[STATE_D1][STATE_D1] = 1;
  A[STATE_D2][STATE_D2] = 1;

  // position from body-frame velocity
  A[STATE_X][STATE_PX] = R[0][0]*dt;
  A[STATE_Y][STATE_PX] = R[1][0]*dt;
  A[STATE_Z][STATE_PX] = R[2][0]*dt;

  A[STATE_X][STATE_PY] = R[0][1]*dt;
  A[STATE_Y][STATE_PY] = R[1][1]*dt;
  A[STATE_Z][STATE_PY] = R[2][1]*dt;

  A[STATE_X][STATE_PZ] = R[0][2]*dt;
  A[STATE_Y][STATE_PZ] = R[1][2]*dt;
  A[STATE_Z][STATE_PZ] = R[2][2]*dt;

  // position from attitude error
  A[STATE_X][STATE_D0] = (S[STATE_PY]*R[0][2] - S[STATE_PZ]*R[0][1])*dt;
  A[STATE_Y][STATE_D0] = (S[STATE_PY]*R[1][2] - S[STATE_PZ]*R[1][1])*dt;
  A[STATE_Z][STATE_D0] = (S[STATE_PY]*R[2][2] - S[STATE_PZ]*R[2][1])*dt;

  A[STATE_X][STATE_D1] = (- S[STATE_PX]*R[0][2] + S[STATE_PZ]*R[0][0])*dt;
  A[STATE_Y][STATE_D1] = (- S[STATE_PX]*R[1][2] + S[STATE_PZ]*R[1][0])*dt;
  A[STATE_Z][STATE_D1] = (- S[STATE_PX]*R[2][2] + S[STATE_PZ]*R[2][0])*dt;

  A[STATE_X][STATE_D2] = (S[STATE_PX]*R[0][1] - S[STATE_PY]*R[0][0])*dt;
  A[STATE_Y][STATE_D2] = (S[STATE_PX]*R[1][1] - S[STATE_PY]*R[1][0])*dt;
  A[STATE_Z][STATE_D2] = (S[STATE_PX]*R[2][1] - S[STATE_PY]*R[2][0])*dt;

  // body-frame velocity from body-frame velocity
  A[STATE_PX][STATE_PX] = 1; //drag negligible
  A[STATE_PY][STATE_PX] =-gyro->z*dt;
  A[STATE_PZ][STATE_PX] = gyro->y*dt;

  A[STATE_PX][STATE_PY] = gyro->z*dt;
  A[STATE_PY][STATE_PY] = 1; //drag negligible
  A[STATE_PZ][STATE_PY] =-gyro->x*dt;

  A[STATE_PX][STATE_PZ] =-gyro->y*dt;
  A[STATE_PY][STATE_PZ] = gyro->x*dt;
  A[STATE_PZ][STATE_PZ] = 1; //drag negligible

  // body-frame velocity from attitude error
  A[STATE_PX][STATE_D0] =  0;
  A[STATE_PY][STATE_D0] = -GRAVITY_MAGNITUDE*R[2][2]*dt;
  A[STATE_PZ][STATE_D0] =  GRAVITY_MAGNITUDE*R[2][1]*dt;

  A[STATE_PX][STATE_D1] =  GRAVITY_MAGNITUDE*R[2][2]*dt;
  A[STATE_PY][STATE_D1] =  0;
  A[STATE_PZ][STATE_D1] = -GRAVITY_MAGNITUDE*R[2][0]*dt;

  A[STATE_PX][STATE_D2] = -GRAVITY_MAGNITUDE*R[2][1]*dt;
  A[STATE_PY][STATE_D2] =  GRAVITY_MAGNITUDE*R[2][0]*dt;
  A[STATE_PZ][STATE_D2] =  0;

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

  A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
  A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

  A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
  A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

  A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
  A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
  A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;


  // ====== COVARIANCE UPDATE ======
  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'
  // Process noise is added after the return from the prediction step

  // ====== PREDICTION STEP ======
  // The prediction depends on whether we're on the ground, or in flight.
  // When flying, the accelerometer directly measures thrust (hence is useless to estimate body angle while flying)

  // TODO: Find a better check for whether the quad is flying
  // Assume that the flight begins when the thrust is large enough and for now we never stop "flying".
  if (cmdThrust > IN_FLIGHT_THRUST_THRESHOLD) {
    lastFlightCmd = xTaskGetTickCount();
    if (!quadIsFlying) {
      takeoffTime = lastFlightCmd;
    }
  }
  quadIsFlying = (xTaskGetTickCount()-lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;

  float dx, dy, dz;
  float tmpSPX, tmpSPY, tmpSPZ;
  float zacc;

  if (quadIsFlying) // only acceleration in z direction
  {
    // TODO: In the next lines, can either use cmdThrust/mass, or acc->z. Need to test which is more reliable.
    // cmdThrust's error comes from poorly calibrated mass, and inexact cmdThrust -> thrust map
    // acc->z's error comes from measurement noise and accelerometer scaling
    // float zacc = cmdThrust;
    zacc = acc->z;

    // position updates in the body frame (will be rotated to inertial frame)
    dx = S[STATE_PX] * dt;
    dy = S[STATE_PY] * dt;
    dz = S[STATE_PZ] * dt + zacc * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    S[STATE_X] += R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
    S[STATE_Y] += R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
    S[STATE_Z] += R[2][0] * dx + R[2][1] * dy + R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = S[STATE_PX];
    tmpSPY = S[STATE_PY];
    tmpSPZ = S[STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    S[STATE_PX] += dt * (gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * R[2][0]);
    S[STATE_PY] += dt * (-gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * R[2][1]);
    S[STATE_PZ] += dt * (zacc + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * R[2][2]);
  }
  else // Acceleration can be in any direction, as measured by the accelerometer. This occurs, eg. in freefall or while being carried.
  {
    // position updates in the body frame (will be rotated to inertial frame)
    dx = S[STATE_PX] * dt + acc->x * dt2 / 2.0f;
    dy = S[STATE_PY] * dt + acc->y * dt2 / 2.0f;
    dz = S[STATE_PZ] * dt + acc->z * dt2 / 2.0f; // thrust can only be produced in the body's Z direction

    // position update
    S[STATE_X] += R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
    S[STATE_Y] += R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
    S[STATE_Z] += R[2][0] * dx + R[2][1] * dy + R[2][2] * dz - GRAVITY_MAGNITUDE * dt2 / 2.0f;

    // keep previous time step's state for the update
    tmpSPX = S[STATE_PX];
    tmpSPY = S[STATE_PY];
    tmpSPZ = S[STATE_PZ];

    // body-velocity update: accelerometers - gyros cross velocity - gravity in body frame
    S[STATE_PX] += dt * (acc->x + gyro->z * tmpSPY - gyro->y * tmpSPZ - GRAVITY_MAGNITUDE * R[2][0]);
    S[STATE_PY] += dt * (acc->y - gyro->z * tmpSPX + gyro->x * tmpSPZ - GRAVITY_MAGNITUDE * R[2][1]);
    S[STATE_PZ] += dt * (acc->z + gyro->y * tmpSPX - gyro->x * tmpSPY - GRAVITY_MAGNITUDE * R[2][2]);
  }

  if(S[STATE_Z] < 0) {
    S[STATE_Z] = 0;
    S[STATE_PX] = 0;
    S[STATE_PY] = 0;
    S[STATE_PZ] = 0;
  }

  // attitude update (rotate by gyroscope), we do this in quaternions
  // this is the gyroscope angular velocity integrated over the sample period
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // compute the quaternion values in [w,x,y,z] order
  float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz);
  float ca = arm_cos_f32(angle/2.0f);
  float sa = arm_sin_f32(angle/2.0f);
  float dq[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

  float tmpq0;
  float tmpq1;
  float tmpq2;
  float tmpq3;

  if (quadIsFlying) {
    // rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = (dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3]);
    tmpq1 = (1.0f)*(dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3]);
    tmpq2 = (1.0f)*(dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3]);
    tmpq3 = (1.0f)*(dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3]);
  } else {
    // rotate the quad's attitude by the delta quaternion vector computed above
    tmpq0 = (dq[0]*q[0] - dq[1]*q[1] - dq[2]*q[2] - dq[3]*q[3]);
    tmpq1 = (1.0f-ROLLPITCH_ZERO_REVERSION)*(dq[1]*q[0] + dq[0]*q[1] + dq[3]*q[2] - dq[2]*q[3]);
    tmpq2 = (1.0f-ROLLPITCH_ZERO_REVERSION)*(dq[2]*q[0] - dq[3]*q[1] + dq[0]*q[2] + dq[1]*q[3]);
    tmpq3 = (1.0f-ROLLPITCH_ZERO_REVERSION)*(dq[3]*q[0] + dq[2]*q[1] - dq[1]*q[2] + dq[0]*q[3]);
  }


  // normalize and store the result
  float norm = arm_sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
  q[0] = tmpq0/norm; q[1] = tmpq1/norm; q[2] = tmpq2/norm; q[3] = tmpq3/norm;
  stateEstimatorAssertNotNaN();
}

static void stateEstimatorAddProcessNoise(float dt)
{
  if (dt>0)
  {
    P[STATE_X][STATE_X] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // add process noise on position
    P[STATE_Y][STATE_Y] += powf(procNoiseAcc_xy*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // add process noise on position
    P[STATE_Z][STATE_Z] += powf(procNoiseAcc_z*dt*dt + procNoiseVel*dt + procNoisePos, 2);  // add process noise on position

    P[STATE_PX][STATE_PX] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); // add process noise on velocity
    P[STATE_PY][STATE_PY] += powf(procNoiseAcc_xy*dt + procNoiseVel, 2); // add process noise on velocity
    P[STATE_PZ][STATE_PZ] += powf(procNoiseAcc_z*dt + procNoiseVel, 2); // add process noise on velocity

    P[STATE_D0][STATE_D0] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    P[STATE_D1][STATE_D1] += powf(measNoiseGyro_rollpitch * dt + procNoiseAtt, 2);
    P[STATE_D2][STATE_D2] += powf(measNoiseGyro_yaw * dt + procNoiseAtt, 2);
  }

  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}


static void stateEstimatorScalarUpdate(arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // The Kalman gain as a column vector
  static float K[STATE_DIM];
  static arm_matrix_instance_f32 Km = {STATE_DIM, 1, (float *)K};

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

  static float tmpNN3d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM, STATE_DIM, tmpNN3d};

  static float HTd[STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {STATE_DIM, 1, HTd};

  static float PHTd[STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {STATE_DIM, 1, PHTd};

  configASSERT(Hm->numRows == 1);
  configASSERT(Hm->numCols == STATE_DIM);

  // ====== INNOVATION COVARIANCE ======

  mat_trans(Hm, &HTm);
  mat_mult(&Pm, &HTm, &PHTm); // PH'
  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R; // HPH' + R
  for (int i=0; i<STATE_DIM; i++) { // Add the element of HPH' to the above
    HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  }
  configASSERT(!isnan(HPHR));

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  for (int i=0; i<STATE_DIM; i++) {
    K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
    S[i] = S[i] + K[i] * error; // state update
  }
  stateEstimatorAssertNotNaN();

  // ====== COVARIANCE UPDATE ======
  mat_mult(&Km, Hm, &tmpNN1m); // KH
  for (int i=0; i<STATE_DIM; i++) { tmpNN1d[STATE_DIM*i+i] -= 1; } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
  mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
  stateEstimatorAssertNotNaN();
  // add the measurement variance and ensure boundedness and symmetry
  // TODO: Why would it hit these bounds? Needs to be investigated.
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float v = K[i] * R * K[j];
      float p = 0.5f*P[i][j] + 0.5f*P[j][i] + v; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}

static void stateEstimatorUpdateWithAccOnGround(Axis3f *acc)
{
  // The following code is disabled due to the function not being complete (and that we aim for zero warnings).
#if 0
  // This update only makes sense on the ground, when no thrust is being produced,
  // since the accelerometers can then directly measure the direction of gravity
  float accMag = sqrtf(acc->x*acc->x + acc->y*acc->y + acc->z*acc->z);

  // Only do the update if the quad isn't flying, and if the accelerometers
  // are close enough to gravity that we can assume it is the only force
  if(!quadIsFlying && fabs(1-accMag/GRAVITY_MAGNITUDE) < 0.01) {
    float h[STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, STATE_DIM, h};

    float gravityInBodyX = GRAVITY_MAGNITUDE * R[2][0];
    float gravityInBodyY = GRAVITY_MAGNITUDE * R[2][1];
    float gravityInBodyZ = GRAVITY_MAGNITUDE * R[2][2];

    // TODO: What are the update equations?
  }
#endif
}

#ifdef KALMAN_USE_BARO_UPDATE
static void stateEstimatorUpdateWithBaro(baro_t *baro)
{
  static float baroReferenceHeight = 0;

  float h[STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, STATE_DIM, h};

  h[STATE_Z] = 1;

  if (!quadIsFlying || baroReferenceHeight < 1) {
    //TODO: maybe we could track the zero height as a state. Would be especially useful if UWB anchors had barometers.
    baroReferenceHeight = baro->asl;
  }

  float meas = (baro->asl - baroReferenceHeight);
  stateEstimatorScalarUpdate(&H, meas - S[STATE_Z], measNoiseBaro);
}
#endif

static void stateEstimatorUpdateWithPosition(positionMeasurement_t *xyz)
{
  // a direct measurement of states x, y, and z
  // do a scalar update for each state, since this should be faster than updating all together
  for (int i=0; i<3; i++) {
    float h[STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, STATE_DIM, h};
    h[STATE_X+i] = 1;
    stateEstimatorScalarUpdate(&H, xyz->pos[i] - S[STATE_X+i], xyz->stdDev);
  }
}

static void stateEstimatorUpdateWithDistance(distanceMeasurement_t *d)
{
  // a measurement of distance to point (x, y, z)
  float h[STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, STATE_DIM, h};

  float dx = S[STATE_X] - d->x;
  float dy = S[STATE_Y] - d->y;
  float dz = S[STATE_Z] - d->z;

  float predictedDistance = arm_sqrt(powf(dx, 2) + powf(dy, 2) + powf(dz, 2));
  float measuredDistance = d->distance;

  // The measurement is: z = sqrt(dx^2 + dy^2 + dz^2). The derivative dz/dX gives h.
  h[STATE_X] = dx/predictedDistance;
  h[STATE_Y] = dy/predictedDistance;
  h[STATE_Z] = dz/predictedDistance;

  stateEstimatorScalarUpdate(&H, measuredDistance-predictedDistance, d->stdDev);
}

static void stateEstimatorUpdateWithTDOA(tdoaMeasurement_t *tdoa)
{
  /**
   * Measurement equation:
   * dR = dT + d1 - d0
   */

  float measurement = tdoa->distanceDiff;

  // predict based on current state
  float x = S[STATE_X];
  float y = S[STATE_Y];
  float z = S[STATE_Z];

  float x1 = tdoa->anchorPosition[1].x, y1 = tdoa->anchorPosition[1].y, z1 = tdoa->anchorPosition[1].z;
  float x0 = tdoa->anchorPosition[0].x, y0 = tdoa->anchorPosition[0].y, z0 = tdoa->anchorPosition[0].z;

  float d1 = sqrtf(powf(x - x1, 2) + powf(y - y1, 2) + powf(z - z1, 2));
  float d0 = sqrtf(powf(x - x0, 2) + powf(y - y0, 2) + powf(z - z0, 2));

  float predicted = d1 - d0;
  float error = measurement - predicted;

  if (tdoaCount >= 100)
  {
    float h[STATE_DIM] = {0};
    arm_matrix_instance_f32 H = {1, STATE_DIM, h};

    h[STATE_X] = ((x - x1) / d1 - (x - x0) / d0);
    h[STATE_Y] = ((y - y1) / d1 - (y - y0) / d0);
    h[STATE_Z] = ((z - z1) / d1 - (z - z0) / d0);

    stateEstimatorScalarUpdate(&H, error, tdoa->stdDev);
  }

  tdoaCount++;
}

// TODO remove the temporary test variables (used for logging)
static float omegax_b;
static float omegay_b;
static float dx_g;
static float dy_g;
static float z_g;
static float predictedNX;
static float predictedNY;
static float measuredNX;
static float measuredNY;

static void stateEstimatorUpdateWithFlow(flowMeasurement_t *flow, sensorData_t *sensors)
{
  // Inclusion of flow measurements in the EKF done by two scalar updates

  // ~~~ Camera constants ~~~
  // The angle of aperture is guessed from the raw data register and thankfully look to be symmetric
  float Npix = 30.0;                      // [pixels] (same in x and y)
  //float thetapix = DEG_TO_RAD * 4.0f;     // [rad]    (same in x and y)
  float thetapix = DEG_TO_RAD * 4.2f;
  //~~~ Body rates ~~~
  // TODO check if this is feasible or if some filtering has to be done
  omegax_b = sensors->gyro.x * DEG_TO_RAD;
  omegay_b = sensors->gyro.y * DEG_TO_RAD;

  // ~~~ Moves the body velocity into the global coordinate system ~~~
  // [bar{x},bar{y},bar{z}]_G = R*[bar{x},bar{y},bar{z}]_B
  //
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  // \dot{x}_G = (R^T*[dot{x}_B,dot{y}_B,dot{z}_B])\dot \hat{x}_G
  //
  // where \hat{} denotes a basis vector, \dot{} denotes a derivative and
  // _G and _B refer to the global/body coordinate systems.

  // Modification 1
  //dx_g = R[0][0] * S[STATE_PX] + R[0][1] * S[STATE_PY] + R[0][2] * S[STATE_PZ];
  //dy_g = R[1][0] * S[STATE_PX] + R[1][1] * S[STATE_PY] + R[1][2] * S[STATE_PZ];


  dx_g = S[STATE_PX];
  dy_g = S[STATE_PY];
  // Saturate elevation in prediction and correction to avoid singularities
  if ( S[STATE_Z] < 0.1f ) {
      z_g = 0.1;
  } else {
      z_g = S[STATE_Z];
  }

  // ~~~ X velocity prediction and update ~~~
  // predics the number of accumulated pixels in the x-direction
  float omegaFactor = 1.25f;
  float hx[STATE_DIM] = {0};
  arm_matrix_instance_f32 Hx = {1, STATE_DIM, hx};
  predictedNX = (flow->dt * Npix / thetapix ) * ((dx_g * R[2][2] / z_g) - omegaFactor * omegay_b);
  measuredNX = flow->dpixelx;

  // derive measurement equation with respect to dx (and z?)
  hx[STATE_Z] = (Npix * flow->dt / thetapix) * ((R[2][2] * dx_g) / (-z_g * z_g));
  hx[STATE_PX] = (Npix * flow->dt / thetapix) * (R[2][2] / z_g);

  //First update
  stateEstimatorScalarUpdate(&Hx, measuredNX-predictedNX, flow->stdDevX);

  // ~~~ Y velocity prediction and update ~~~
  float hy[STATE_DIM] = {0};
  arm_matrix_instance_f32 Hy = {1, STATE_DIM, hy};
  predictedNY = (flow->dt * Npix / thetapix ) * ((dy_g * R[2][2] / z_g) + omegaFactor * omegax_b);
  measuredNY = flow->dpixely;

  // derive measurement equation with respect to dy (and z?)
  hy[STATE_Z] = (Npix * flow->dt / thetapix) * ((R[2][2] * dy_g) / (-z_g * z_g));
  hy[STATE_PY] = (Npix * flow->dt / thetapix) * (R[2][2] / z_g);

  // Second update
  stateEstimatorScalarUpdate(&Hy, measuredNY-predictedNY, flow->stdDevY);
}

static void stateEstimatorUpdateWithTof(tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(R[2][2]) > 0.1 && R[2][2] > 0){
    float angle = fabsf(acosf(R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    //float predictedDistance = S[STATE_Z] / cosf(angle);
    float predictedDistance = S[STATE_Z] / R[2][2];
    float measuredDistance = tof->distance; // [m]

    //Measurement equation
    //
    // h = z/((R*z_b)\dot z_b) = z/cos(alpha)
    h[STATE_Z] = 1 / R[2][2];
    //h[STATE_Z] = 1 / cosf(angle);

    // Scalar update
    stateEstimatorScalarUpdate(&H, measuredDistance-predictedDistance, tof->stdDev);
  }
}

static void stateEstimatorFinalize(sensorData_t *sensors, uint32_t tick)
{
  // Matrix to rotate the attitude covariances once updated
  static float A[STATE_DIM][STATE_DIM];
  static arm_matrix_instance_f32 Am = {STATE_DIM, STATE_DIM, (float *)A};

  // Temporary matrices for the covariance updates
  static float tmpNN1d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM, STATE_DIM, tmpNN1d};

  static float tmpNN2d[STATE_DIM * STATE_DIM];
  static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM, STATE_DIM, tmpNN2d};

  // Incorporate the attitude error (Kalman filter state) with the attitude
  float v0 = S[STATE_D0];
  float v1 = S[STATE_D1];
  float v2 = S[STATE_D2];

  // Move attitude error into attitude if any of the angle errors are large enough
  if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  {
    float angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2);
    float ca = arm_cos_f32(angle / 2.0f);
    float sa = arm_sin_f32(angle / 2.0f);
    float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

    // rotate the quad's attitude by the delta quaternion vector computed above
    float tmpq0 = dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3];
    float tmpq1 = dq[1] * q[0] + dq[0] * q[1] + dq[3] * q[2] - dq[2] * q[3];
    float tmpq2 = dq[2] * q[0] - dq[3] * q[1] + dq[0] * q[2] + dq[1] * q[3];
    float tmpq3 = dq[3] * q[0] + dq[2] * q[1] - dq[1] * q[2] + dq[0] * q[3];

    // normalize and store the result
    float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3);
    q[0] = tmpq0 / norm;
    q[1] = tmpq1 / norm;
    q[2] = tmpq2 / norm;
    q[3] = tmpq3 / norm;

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

    A[STATE_X][STATE_X] = 1;
    A[STATE_Y][STATE_Y] = 1;
    A[STATE_Z][STATE_Z] = 1;

    A[STATE_PX][STATE_PX] = 1;
    A[STATE_PY][STATE_PY] = 1;
    A[STATE_PZ][STATE_PZ] = 1;

    A[STATE_D0][STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
    A[STATE_D0][STATE_D1] =  d2 + d0*d1/2;
    A[STATE_D0][STATE_D2] = -d1 + d0*d2/2;

    A[STATE_D1][STATE_D0] = -d2 + d0*d1/2;
    A[STATE_D1][STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
    A[STATE_D1][STATE_D2] =  d0 + d1*d2/2;

    A[STATE_D2][STATE_D0] =  d1 + d0*d2/2;
    A[STATE_D2][STATE_D1] = -d0 + d1*d2/2;
    A[STATE_D2][STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

    mat_trans(&Am, &tmpNN1m); // A'
    mat_mult(&Am, &Pm, &tmpNN2m); // AP
    mat_mult(&tmpNN2m, &tmpNN1m, &Pm); //APA'
  }

  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
  R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];

  R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
  R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
  R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];

  R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
  R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
  R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  // reset the attitude error
  S[STATE_D0] = 0;
  S[STATE_D1] = 0;
  S[STATE_D2] = 0;

  // constrain the states
  for (int i=0; i<3; i++)
  {
    if (S[STATE_X+i] < -MAX_POSITION) { S[STATE_X+i] = -MAX_POSITION; }
    else if (S[STATE_X+i] > MAX_POSITION) { S[STATE_X+i] = MAX_POSITION; }

    if (S[STATE_PX+i] < -MAX_VELOCITY) { S[STATE_PX+i] = -MAX_VELOCITY; }
    else if (S[STATE_PX+i] > MAX_VELOCITY) { S[STATE_PX+i] = MAX_VELOCITY; }
  }

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<STATE_DIM; i++) {
    for (int j=i; j<STATE_DIM; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }

  stateEstimatorAssertNotNaN();
}


static void stateEstimatorExternalizeState(state_t *state, sensorData_t *sensors, uint32_t tick)
{
  // position state is already in world frame
  state->position = (point_t){
      .timestamp = tick,
      .x = S[STATE_X],
      .y = S[STATE_Y],
      .z = S[STATE_Z]
  };

  // velocity is in body frame and needs to be rotated to world frame
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = R[0][0]*S[STATE_PX] + R[0][1]*S[STATE_PY] + R[0][2]*S[STATE_PZ],
      .y = R[1][0]*S[STATE_PX] + R[1][1]*S[STATE_PY] + R[1][2]*S[STATE_PZ],
      .z = R[2][0]*S[STATE_PX] + R[2][1]*S[STATE_PY] + R[2][2]*S[STATE_PZ]
  };

  // Accelerometer measurements are in the body frame and need to be rotated to world frame.
  // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
  // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
  state->acc = (acc_t){
      .timestamp = tick,
      .x = R[0][0]*sensors->acc.x + R[0][1]*sensors->acc.y + R[0][2]*sensors->acc.z,
      .y = R[1][0]*sensors->acc.x + R[1][1]*sensors->acc.y + R[1][2]*sensors->acc.z,
      .z = R[2][0]*sensors->acc.x + R[2][1]*sensors->acc.y + R[2][2]*sensors->acc.z - 1
  };

  // convert the new attitude into Euler YPR
  float yaw = atan2f(2*(q[1]*q[2]+q[0]*q[3]) , q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
  float pitch = asinf(-2*(q[1]*q[3] - q[0]*q[2]));
  float roll = atan2f(2*(q[2]*q[3]+q[0]*q[1]) , q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

  // Save attitude, adjusted for the legacy CF2 body coordinate system
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll*RAD_TO_DEG,
      .pitch = -pitch*RAD_TO_DEG,
      .yaw = yaw*RAD_TO_DEG
  };

  // Save quaternion, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick,
      .w = q[0],
      .x = q[1],
      .y = q[2],
      .z = q[3]
  };
}


void estimatorKalmanInit(void) {
  if (!isInit)
  {
    distDataQueue = xQueueCreate(DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
    posDataQueue = xQueueCreate(POS_QUEUE_LENGTH, sizeof(positionMeasurement_t));
    tdoaDataQueue = xQueueCreate(UWB_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));
    flowDataQueue = xQueueCreate(FLOW_QUEUE_LENGTH, sizeof(flowMeasurement_t));
    tofDataQueue = xQueueCreate(TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
  }
  else
  {
    xQueueReset(distDataQueue);
    xQueueReset(posDataQueue);
    xQueueReset(tdoaDataQueue);
    xQueueReset(flowDataQueue);
    xQueueReset(tofDataQueue);
  }

  lastPrediction = xTaskGetTickCount();
  lastBaroUpdate = xTaskGetTickCount();
  lastTDOAUpdate = xTaskGetTickCount();
  lastPNUpdate = xTaskGetTickCount();

  accAccumulator = (Axis3f){.axis={0}};
  gyroAccumulator = (Axis3f){.axis={0}};
  thrustAccumulator = 0;
  baroAccumulator.asl = 0;

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;
  thrustAccumulatorCount = 0;
  baroAccumulatorCount = 0;

  // Reset all matrices to 0 (like uppon system reset)
  memset(q, 0, sizeof(q));
  memset(R, 0, sizeof(R));
  memset(P, 0, sizeof(S));

  // TODO: Can we initialize this more intelligently?
  S[STATE_X] = initialX;
  S[STATE_Y] = initialY;
  S[STATE_Z] = initialZ;
  S[STATE_PX] = 0;
  S[STATE_PY] = 0;
  S[STATE_PZ] = 0;
  S[STATE_D0] = 0;
  S[STATE_D1] = 0;
  S[STATE_D2] = 0;

  // reset the attitude quaternion
  q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { R[i][j] = i==j ? 1 : 0; }}

  for (int i=0; i< STATE_DIM; i++) {
    for (int j=0; j < STATE_DIM; j++) {
      P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  P[STATE_X][STATE_X]  = powf(stdDevInitialPosition_xy, 2);
  P[STATE_Y][STATE_Y]  = powf(stdDevInitialPosition_xy, 2);
  P[STATE_Z][STATE_Z]  = powf(stdDevInitialPosition_z, 2);

  P[STATE_PX][STATE_PX] = powf(stdDevInitialVelocity, 2);
  P[STATE_PY][STATE_PY] = powf(stdDevInitialVelocity, 2);
  P[STATE_PZ][STATE_PZ] = powf(stdDevInitialVelocity, 2);

  P[STATE_D0][STATE_D0] = powf(stdDevInitialAttitude_rollpitch, 2);
  P[STATE_D1][STATE_D1] = powf(stdDevInitialAttitude_rollpitch, 2);
  P[STATE_D2][STATE_D2] = powf(stdDevInitialAttitude_yaw, 2);

  varSkew = powf(stdDevInitialSkew, 2);

  tdoaCount = 0;
  isInit = true;
}

static bool stateEstimatorEnqueueExternalMeasurement(xQueueHandle queue, void *measurement)
{
  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
    if(xHigherPriorityTaskWoken == pdTRUE)
    {
      portYIELD();
    }
  } else {
    result = xQueueSend(queue, measurement, 0);
  }
  return (result==pdTRUE);
}

bool estimatorKalmanEnqueueTDOA(tdoaMeasurement_t *uwb)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(tdoaDataQueue, (void *)uwb);
}

bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(posDataQueue, (void *)pos);
}

bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(distDataQueue, (void *)dist);
}

bool estimatorKalmanEnqueueFlow(flowMeasurement_t *flow)
{
  // A flow measurement (dnx,  dny) [accumulated pixels]
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(flowDataQueue, (void *)flow);
}

bool estimatorKalmanEnqueueTOF(tofMeasurement_t *tof)
{
  // A distance (distance) [m] to the ground along the z_B axis.
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(tofDataQueue, (void *)tof);
}

bool estimatorKalmanTest(void)
{
  // TODO: Figure out what we could test?
  return isInit;
}

float estimatorKalmanGetElevation()
{
  // Return elevation, used in the optical flow
  return S[STATE_Z];
}

void estimatorKalmanSetShift(float deltax, float deltay)
{
  // Return elevation, used in the optical flow
  S[STATE_X] -= deltax;
  S[STATE_Y] -= deltay;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = S[STATE_X];
  pos->y = S[STATE_Y];
  pos->z = S[STATE_Z];
}

// Temporary development groups
LOG_GROUP_START(kalman_states)
  LOG_ADD(LOG_FLOAT, ox, &S[STATE_X])
  LOG_ADD(LOG_FLOAT, oy, &S[STATE_Y])
  LOG_ADD(LOG_FLOAT, vx, &S[STATE_PX])
  LOG_ADD(LOG_FLOAT, vy, &S[STATE_PY])
LOG_GROUP_STOP(kalman_states)

LOG_GROUP_START(kalman_pred)
  LOG_ADD(LOG_FLOAT, predNX, &predictedNX)
  LOG_ADD(LOG_FLOAT, predNY, &predictedNY)
  LOG_ADD(LOG_FLOAT, measNX, &measuredNX)
  LOG_ADD(LOG_FLOAT, measNY, &measuredNY)
LOG_GROUP_STOP(kalman_pred)

// Stock log groups
LOG_GROUP_START(kalman)
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  LOG_ADD(LOG_FLOAT, stateX, &S[STATE_X])
  LOG_ADD(LOG_FLOAT, stateY, &S[STATE_Y])
  LOG_ADD(LOG_FLOAT, stateZ, &S[STATE_Z])
  LOG_ADD(LOG_FLOAT, statePX, &S[STATE_PX])
  LOG_ADD(LOG_FLOAT, statePY, &S[STATE_PY])
  LOG_ADD(LOG_FLOAT, statePZ, &S[STATE_PZ])
  LOG_ADD(LOG_FLOAT, stateD0, &S[STATE_D0])
  LOG_ADD(LOG_FLOAT, stateD1, &S[STATE_D1])
  LOG_ADD(LOG_FLOAT, stateD2, &S[STATE_D2])
  LOG_ADD(LOG_FLOAT, stateSkew, &stateSkew)
  LOG_ADD(LOG_FLOAT, varX, &P[STATE_X][STATE_X])
  LOG_ADD(LOG_FLOAT, varY, &P[STATE_Y][STATE_Y])
  LOG_ADD(LOG_FLOAT, varZ, &P[STATE_Z][STATE_Z])
  LOG_ADD(LOG_FLOAT, varPX, &P[STATE_PX][STATE_PX])
  LOG_ADD(LOG_FLOAT, varPY, &P[STATE_PY][STATE_PY])
  LOG_ADD(LOG_FLOAT, varPZ, &P[STATE_PZ][STATE_PZ])
  LOG_ADD(LOG_FLOAT, varD0, &P[STATE_D0][STATE_D0])
  LOG_ADD(LOG_FLOAT, varD1, &P[STATE_D1][STATE_D1])
  LOG_ADD(LOG_FLOAT, varD2, &P[STATE_D2][STATE_D2])
  LOG_ADD(LOG_FLOAT, varSkew, &varSkew)
  LOG_ADD(LOG_FLOAT, q0, &q[0])
  LOG_ADD(LOG_FLOAT, q1, &q[1])
  LOG_ADD(LOG_FLOAT, q2, &q[2])
  LOG_ADD(LOG_FLOAT, q3, &q[3])
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &resetEstimation)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
  PARAM_ADD(PARAM_FLOAT, pNAcc_xy, &procNoiseAcc_xy)
  PARAM_ADD(PARAM_FLOAT, pNAcc_z, &procNoiseAcc_z)
  PARAM_ADD(PARAM_FLOAT, pNVel, &procNoiseVel)
  PARAM_ADD(PARAM_FLOAT, pNPos, &procNoisePos)
  PARAM_ADD(PARAM_FLOAT, pNAtt, &procNoiseAtt)
  PARAM_ADD(PARAM_FLOAT, pNSkew, &procNoiseSkew)
  PARAM_ADD(PARAM_FLOAT, mNBaro, &measNoiseBaro)
  PARAM_ADD(PARAM_FLOAT, mNGyro_rollpitch, &measNoiseGyro_rollpitch)
  PARAM_ADD(PARAM_FLOAT, mNGyro_yaw, &measNoiseGyro_yaw)
  PARAM_ADD(PARAM_FLOAT, initialX, &initialX)
  PARAM_ADD(PARAM_FLOAT, initialY, &initialY)
  PARAM_ADD(PARAM_FLOAT, initialZ, &initialZ)
PARAM_GROUP_STOP(kalman)
