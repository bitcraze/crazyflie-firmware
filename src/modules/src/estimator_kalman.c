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
 *
 */

#include "kalman_core.h"
#include "estimator_kalman.h"
#include "kalman_supervisor.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "sensors.h"

#include "log.h"
#include "param.h"
#include "physicalConstants.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"


// #define KALMAN_USE_BARO_UPDATE


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

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
  return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Direct measurements of Crazyflie position
static xQueueHandle posDataQueue;
#define POS_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
  return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Direct measurements of Crazyflie pose
static xQueueHandle poseDataQueue;
#define POSE_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasPoseMeasurement(poseMeasurement_t *pose) {
  return (pdTRUE == xQueueReceive(poseDataQueue, pose, 0));
}

// Measurements of a UWB Tx/Rx
static xQueueHandle tdoaDataQueue;
#define UWB_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb) {
  return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}


// Measurements of flow (dnx, dny)
static xQueueHandle flowDataQueue;
#define FLOW_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow) {
  return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}

// Measurements of TOF from laser sensor
static xQueueHandle tofDataQueue;
#define TOF_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
  return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

// Absolute height measurement along the room Z
static xQueueHandle heightDataQueue;
#define HEIGHT_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasHeightPacket(heightMeasurement_t *height) {
  return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}

/**
 * Constants used in the estimator
 */

#define CRAZYFLIE_WEIGHT_grams (27.0f)

//thrust is thrust mapped for 65536 <==> 60 GRAMS!
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/CRAZYFLIE_WEIGHT_grams/65536.0f)


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

// the point at which the dynamics change from stationary to flying
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE*0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)



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

static kalmanCoreData_t coreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;
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
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;

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



// --------------------------------------------------


void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
  // If the client (via a parameter update) triggers an estimator reset:
  if (coreData.resetEstimation) { estimatorKalmanInit(); coreData.resetEstimation = false; }

  // Tracks whether an update to the state has been made, and the state therefore requires finalization
  bool doneUpdate = false;

  uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

#ifdef KALMAN_DECOUPLE_XY
  kalmanCoreDecoupleXY(this);
#endif

  // Average the last IMU measurements. We do this because the prediction loop is
  // slower than the IMU loop, but the IMU information is required externally at
  // a higher rate (for body rate control).
  if (sensorsReadAcc(&sensors->acc)) {
    accAccumulator.x += sensors->acc.x;
    accAccumulator.y += sensors->acc.y;
    accAccumulator.z += sensors->acc.z;
    accAccumulatorCount++;
  }

  if (sensorsReadGyro(&sensors->gyro)) {
    gyroAccumulator.x += sensors->gyro.x;
    gyroAccumulator.y += sensors->gyro.y;
    gyroAccumulator.z += sensors->gyro.z;
    gyroAccumulatorCount++;
  }

  // Average the thrust command from the last time steps, generated externally by the controller
  thrustAccumulator += control->thrust;
  thrustAccumulatorCount++;

  // Run the system dynamics to predict the state forward.
  if ((osTick-lastPrediction) >= configTICK_RATE_HZ/PREDICT_RATE // update at the PREDICT_RATE
      && gyroAccumulatorCount > 0
      && accAccumulatorCount > 0
      && thrustAccumulatorCount > 0)
  {
    // gyro is in deg/sec but the estimator requires rad/sec
    gyroAccumulator.x *= DEG_TO_RAD;
    gyroAccumulator.y *= DEG_TO_RAD;
    gyroAccumulator.z *= DEG_TO_RAD;

    gyroAccumulator.x /= gyroAccumulatorCount;
    gyroAccumulator.y /= gyroAccumulatorCount;
    gyroAccumulator.z /= gyroAccumulatorCount;

    // accelerometer is in Gs but the estimator requires ms^-2
    accAccumulator.x *= GRAVITY_MAGNITUDE;
    accAccumulator.y *= GRAVITY_MAGNITUDE;
    accAccumulator.z *= GRAVITY_MAGNITUDE;

    accAccumulator.x /= accAccumulatorCount;
    accAccumulator.y /= accAccumulatorCount;
    accAccumulator.z /= accAccumulatorCount;

    // thrust is in grams, we need ms^-2
    thrustAccumulator *= CONTROL_TO_ACC;

    thrustAccumulator /= thrustAccumulatorCount;

    // TODO: Find a better check for whether the quad is flying
    // Assume that the flight begins when the thrust is large enough and for now we never stop "flying".
    if (thrustAccumulator > IN_FLIGHT_THRUST_THRESHOLD) {
      lastFlightCmd = xTaskGetTickCount();
      if (!quadIsFlying) {
        takeoffTime = lastFlightCmd;
      }
    }
    quadIsFlying = (xTaskGetTickCount()-lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;

    float dt = (float)(osTick-lastPrediction)/configTICK_RATE_HZ;
    kalmanCorePredict(&coreData, thrustAccumulator, &accAccumulator, &gyroAccumulator, dt, quadIsFlying);

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
  kalmanCoreAddProcessNoise(&coreData, (float)(osTick-lastPNUpdate)/configTICK_RATE_HZ);
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

    kalmanCoreUpdateWithBaro(&coreData, &sensors->baro, quadIsFlying);

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
    kalmanCoreUpdateWithTof(&coreData, &tof);
    doneUpdate = true;
  }

  heightMeasurement_t height;
  while (stateEstimatorHasHeightPacket(&height))
  {
    kalmanCoreUpdateWithAbsoluteHeight(&coreData, &height);
    doneUpdate = true;
  }

  distanceMeasurement_t dist;
  while (stateEstimatorHasDistanceMeasurement(&dist))
  {
    kalmanCoreUpdateWithDistance(&coreData, &dist);
    doneUpdate = true;
  }

  positionMeasurement_t pos;
  while (stateEstimatorHasPositionMeasurement(&pos))
  {
    kalmanCoreUpdateWithPosition(&coreData, &pos);
    doneUpdate = true;
  }

  poseMeasurement_t pose;
  while (stateEstimatorHasPoseMeasurement(&pose))
  {
    kalmanCoreUpdateWithPose(&coreData, &pose);
    doneUpdate = true;
  }

  tdoaMeasurement_t tdoa;
  while (stateEstimatorHasTDOAPacket(&tdoa))
  {
    kalmanCoreUpdateWithTDOA(&coreData, &tdoa);
    doneUpdate = true;
  }

  flowMeasurement_t flow;
  while (stateEstimatorHasFlowPacket(&flow))
  {
    kalmanCoreUpdateWithFlow(&coreData, &flow, sensors);
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
    kalmanCoreFinalize(&coreData, sensors, osTick);
    if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
      coreData.resetEstimation = true;
      DEBUG_PRINT("State out of bounds, resetting\n");
    }
  }

  /**
   * Finally, the internal state is externalized.
   * This is done every round, since the external state includes some sensor data
   */
  kalmanCoreExternalizeState(&coreData, state, sensors, osTick);
}


void estimatorKalmanInit(void) {
  if (!isInit)
  {
    distDataQueue = xQueueCreate(DIST_QUEUE_LENGTH, sizeof(distanceMeasurement_t));
    posDataQueue = xQueueCreate(POS_QUEUE_LENGTH, sizeof(positionMeasurement_t));
    poseDataQueue = xQueueCreate(POSE_QUEUE_LENGTH, sizeof(poseMeasurement_t));
    tdoaDataQueue = xQueueCreate(UWB_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));
    flowDataQueue = xQueueCreate(FLOW_QUEUE_LENGTH, sizeof(flowMeasurement_t));
    tofDataQueue = xQueueCreate(TOF_QUEUE_LENGTH, sizeof(tofMeasurement_t));
    heightDataQueue = xQueueCreate(HEIGHT_QUEUE_LENGTH, sizeof(heightMeasurement_t));
  }
  else
  {
    xQueueReset(distDataQueue);
    xQueueReset(posDataQueue);
    xQueueReset(poseDataQueue);
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

  kalmanCoreInit(&coreData);

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

bool estimatorKalmanEnqueueTDOA(const tdoaMeasurement_t *uwb)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(tdoaDataQueue, (void *)uwb);
}

bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(posDataQueue, (void *)pos);
}

bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(poseDataQueue, (void *)pose);
}

bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist)
{
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(distDataQueue, (void *)dist);
}

bool estimatorKalmanEnqueueFlow(const flowMeasurement_t *flow)
{
  // A flow measurement (dnx,  dny) [accumulated pixels]
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(flowDataQueue, (void *)flow);
}

bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof)
{
  // A distance (distance) [m] to the ground along the z_B axis.
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(tofDataQueue, (void *)tof);
}

bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
  // A distance (height) [m] to the ground along the z axis.
  ASSERT(isInit);
  return stateEstimatorEnqueueExternalMeasurement(heightDataQueue, (void *)height);
}

bool estimatorKalmanTest(void)
{
  return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
  pos->x = coreData.S[KC_STATE_X];
  pos->y = coreData.S[KC_STATE_Y];
  pos->z = coreData.S[KC_STATE_Z];
}

// Temporary development groups
LOG_GROUP_START(kalman_states)
  LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)


// Stock log groups
LOG_GROUP_START(kalman)
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
  LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
  LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
  LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
  LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
  LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
  LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
  LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
  LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
  LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
  LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
  LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
  LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
  PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
  PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_GROUP_STOP(kalman)
