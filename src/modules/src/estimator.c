#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"

#define DEBUG_MODULE "ESTIMATOR"
#include "debug.h"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"
#include "log.h"
#include "statsCnt.h"
#include "eventtrigger.h"
#include "quatcompress.h"

#define DEFAULT_ESTIMATOR complementaryEstimator
static StateEstimatorType currentEstimator = anyEstimator;


#define MEASUREMENTS_QUEUE_SIZE (20)
static xQueueHandle measurementsQueue;
STATIC_MEM_QUEUE_ALLOC(measurementsQueue, MEASUREMENTS_QUEUE_SIZE, sizeof(measurement_t));

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

// events
EVENTTRIGGER(estTDOA, uint8, idA, uint8, idB, float, distanceDiff)
EVENTTRIGGER(estPosition, uint8, source)
EVENTTRIGGER(estPose)
EVENTTRIGGER(estDistance, uint8, id, float, distance)
EVENTTRIGGER(estTOF)
EVENTTRIGGER(estAbsoluteHeight)
EVENTTRIGGER(estFlow)
EVENTTRIGGER(estYawError, float, yawError)
EVENTTRIGGER(estSweepAngle, uint8, sensorId, uint8, basestationId, uint8, sweepId, float, t, float, sweepAngle)
EVENTTRIGGER(estGyroscope)
EVENTTRIGGER(estAcceleration)
EVENTTRIGGER(estBarometer)

static void initEstimator(const StateEstimatorType estimator);
static void deinitEstimator(const StateEstimatorType estimator);

typedef struct {
  void (*init)(void);
  void (*deinit)(void);
  bool (*test)(void);
  void (*update)(state_t *state, const uint32_t tick);
  const char* name;
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

static EstimatorFcns estimatorFunctions[] = {
    {
        .init = NOT_IMPLEMENTED,
        .deinit = NOT_IMPLEMENTED,
        .test = NOT_IMPLEMENTED,
        .update = NOT_IMPLEMENTED,
        .name = "None",
    }, // Any estimator
    {
        .init = estimatorComplementaryInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorComplementaryTest,
        .update = estimatorComplementary,
        .name = "Complementary",
    },
    {
        .init = estimatorKalmanInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorKalmanTest,
        .update = estimatorKalman,
        .name = "Kalman",
    },
#ifdef OOT_ESTIMATOR
    {
        .init = estimatorOutOfTreeInit,
        .deinit = NOT_IMPLEMENTED,
        .test = estimatorOutOfTreeTest,
        .update = estimatorOutOfTree,
        .name = "OutOfTree",
    },
#endif
};

void stateEstimatorInit(StateEstimatorType estimator) {
  measurementsQueue = STATIC_MEM_QUEUE_CREATE(measurementsQueue);
  stateEstimatorSwitchTo(estimator);
}

void stateEstimatorSwitchTo(StateEstimatorType estimator) {
  if (estimator < 0 || estimator >= StateEstimatorTypeCount) {
    return;
  }

  StateEstimatorType newEstimator = estimator;

  if (anyEstimator == newEstimator) {
    newEstimator = DEFAULT_ESTIMATOR;
  }

  StateEstimatorType forcedEstimator = ESTIMATOR_NAME;
  if (forcedEstimator != anyEstimator) {
    DEBUG_PRINT("Estimator type forced\n");
    newEstimator = forcedEstimator;
  }

  initEstimator(newEstimator);
  StateEstimatorType previousEstimator = currentEstimator;
  currentEstimator = newEstimator;
  deinitEstimator(previousEstimator);

  DEBUG_PRINT("Using %s (%d) estimator\n", stateEstimatorGetName(), currentEstimator);
}

StateEstimatorType getStateEstimator(void) {
  return currentEstimator;
}

static void initEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].init) {
    estimatorFunctions[estimator].init();
  }
}

static void deinitEstimator(const StateEstimatorType estimator) {
  if (estimatorFunctions[estimator].deinit) {
    estimatorFunctions[estimator].deinit();
  }
}

bool stateEstimatorTest(void) {
  return estimatorFunctions[currentEstimator].test();
}

void stateEstimator(state_t *state, const uint32_t tick) {
  estimatorFunctions[currentEstimator].update(state, tick);
}

const char* stateEstimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}


void estimatorEnqueue(const measurement_t *measurement) {
  if (!measurementsQueue) {
    return;
  }

  portBASE_TYPE result;
  bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;
  if (isInInterrupt) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    result = xQueueSendFromISR(measurementsQueue, measurement, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
      portYIELD();
    }
  } else {
    result = xQueueSend(measurementsQueue, measurement, 0);
  }

  if (result == pdTRUE) {
    STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
  } else {
    STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
  }

  // events
  switch (measurement->type) {
    case MeasurementTypeTDOA:
      eventTrigger_estTDOA_payload.idA = measurement->data.tdoa.anchorIds[0];
      eventTrigger_estTDOA_payload.idB = measurement->data.tdoa.anchorIds[1];
      eventTrigger_estTDOA_payload.distanceDiff = measurement->data.tdoa.distanceDiff;
      eventTrigger(&eventTrigger_estTDOA);
      break;
    case MeasurementTypePosition:
      // for additional data, see locSrv.{x,y,z} and lighthouse.{x,y,z}
      eventTrigger_estPosition_payload.source = measurement->data.position.source;
      eventTrigger(&eventTrigger_estPosition);
      break;
    case MeasurementTypePose:
      // no payload needed, see locSrv.{x,y,z,qx,qy,qz,qw}
      eventTrigger(&eventTrigger_estPose);
      break;
    case MeasurementTypeDistance:
      eventTrigger_estDistance_payload.id = measurement->data.distance.anchorId;
      eventTrigger_estDistance_payload.distance = measurement->data.distance.distance;
      eventTrigger(&eventTrigger_estDistance);
      break;
    case MeasurementTypeTOF:
      // no payload needed, see range.zrange
      eventTrigger(&eventTrigger_estTOF);
      break;
    case MeasurementTypeAbsoluteHeight:
      // no payload needed, see LPS_2D_POSITION_HEIGHT
      eventTrigger(&eventTrigger_estAbsoluteHeight);
      break;
    case MeasurementTypeFlow:
      // no payload needed, see motion.{deltaX,deltaY}
      eventTrigger(&eventTrigger_estFlow);
      break;
    case MeasurementTypeYawError:
      eventTrigger_estYawError_payload.yawError = measurement->data.yawError.yawError;
      eventTrigger(&eventTrigger_estYawError);
      break;
    case MeasurementTypeSweepAngle:
      eventTrigger_estSweepAngle_payload.sensorId = measurement->data.sweepAngle.sensorId;
      eventTrigger_estSweepAngle_payload.basestationId = measurement->data.sweepAngle.basestationId;
      eventTrigger_estSweepAngle_payload.sweepId = measurement->data.sweepAngle.sweepId;
      eventTrigger_estSweepAngle_payload.t = measurement->data.sweepAngle.t;
      eventTrigger_estSweepAngle_payload.sweepAngle = measurement->data.sweepAngle.measuredSweepAngle;
      eventTrigger(&eventTrigger_estSweepAngle);
      break;
    case MeasurementTypeGyroscope:
      // no payload needed, see gyro.{x,y,z}
      eventTrigger(&eventTrigger_estGyroscope);
      break;
    case MeasurementTypeAcceleration:
      // no payload needed, see acc.{x,y,z}
      eventTrigger(&eventTrigger_estAcceleration);
      break;
    case MeasurementTypeBarometer:
      // no payload needed, see baro.asl
      eventTrigger(&eventTrigger_estBarometer);
      break;
    default:
      break;
  }
}

bool estimatorDequeue(measurement_t *measurement) {
  return pdTRUE == xQueueReceive(measurementsQueue, measurement, 0);
}

LOG_GROUP_START(estimator)
  STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
  STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(estimator)
