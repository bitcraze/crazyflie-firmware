#define DEBUG_MODULE "ESTIMATOR"
#include "debug.h"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"

#define DEFAULT_ESTIMATOR complementaryEstimator
static StateEstimatorType currentEstimator = anyEstimator;

static void initEstimator();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
  const char* name;
  bool (*estimatorEnqueueTDOA)(const tdoaMeasurement_t *uwb);
  bool (*estimatorEnqueuePosition)(const positionMeasurement_t *pos);
  bool (*estimatorEnqueuePose)(const poseMeasurement_t *pose);
  bool (*estimatorEnqueueDistance)(const distanceMeasurement_t *dist);
  bool (*estimatorEnqueueTOF)(const tofMeasurement_t *tof);
  bool (*estimatorEnqueueAbsoluteHeight)(const heightMeasurement_t *height);
  bool (*estimatorEnqueueFlow)(const flowMeasurement_t *flow);
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

static EstimatorFcns estimatorFunctions[] = {
  {
    .init = 0,
    .test = 0,
    .update = 0,
    .name = "None",
    .estimatorEnqueueTDOA = NOT_IMPLEMENTED,
    .estimatorEnqueuePosition = NOT_IMPLEMENTED,
    .estimatorEnqueuePose = NOT_IMPLEMENTED,
    .estimatorEnqueueDistance = NOT_IMPLEMENTED,
    .estimatorEnqueueTOF = NOT_IMPLEMENTED,
    .estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
    .estimatorEnqueueFlow = NOT_IMPLEMENTED,
  }, // Any estimator
  {
    .init = estimatorComplementaryInit,
    .test = estimatorComplementaryTest,
    .update = estimatorComplementary,
    .name = "Complementary",
    .estimatorEnqueueTDOA = NOT_IMPLEMENTED,
    .estimatorEnqueuePosition = NOT_IMPLEMENTED,
    .estimatorEnqueuePose = NOT_IMPLEMENTED,
    .estimatorEnqueueDistance = NOT_IMPLEMENTED,
    .estimatorEnqueueTOF = NOT_IMPLEMENTED,
    .estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
    .estimatorEnqueueFlow = NOT_IMPLEMENTED,
  },
  {
    .init = estimatorKalmanInit,
    .test = estimatorKalmanTest,
    .update = estimatorKalman,
    .name = "Kalman",
    .estimatorEnqueueTDOA = estimatorKalmanEnqueueTDOA,
    .estimatorEnqueuePosition = estimatorKalmanEnqueuePosition,
    .estimatorEnqueuePose = estimatorKalmanEnqueuePose,
    .estimatorEnqueueDistance = estimatorKalmanEnqueueDistance,
    .estimatorEnqueueTOF = estimatorKalmanEnqueueTOF,
    .estimatorEnqueueAbsoluteHeight = estimatorKalmanEnqueueAbsoluteHeight,
    .estimatorEnqueueFlow = estimatorKalmanEnqueueFlow,
    },
};


void stateEstimatorInit(StateEstimatorType estimator) {
  if (estimator < 0 || estimator >= StateEstimatorTypeCount) {
    return;
  }

  currentEstimator = estimator;

  if (anyEstimator == currentEstimator) {
    currentEstimator = DEFAULT_ESTIMATOR;
  }

  StateEstimatorType forcedEstimator = ESTIMATOR_NAME;
  if (forcedEstimator != anyEstimator) {
    DEBUG_PRINT("Estimator type forced\n");
    currentEstimator = forcedEstimator;
  }

  initEstimator();

  DEBUG_PRINT("Using %s (%d) estimator\n", stateEstimatorGetName(), currentEstimator);
}

StateEstimatorType getStateEstimator(void) {
  return currentEstimator;
}

static void initEstimator() {
  estimatorFunctions[currentEstimator].init();
}

bool stateEstimatorTest(void) {
  return estimatorFunctions[currentEstimator].test();
}

void stateEstimator(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
  estimatorFunctions[currentEstimator].update(state, sensors, control, tick);
}

const char* stateEstimatorGetName() {
  return estimatorFunctions[currentEstimator].name;
}


bool estimatorEnqueueTDOA(const tdoaMeasurement_t *uwb) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueTDOA) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueTDOA(uwb);
  }

  return false;
}

bool estimatorEnqueuePosition(const positionMeasurement_t *pos) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueuePosition) {
    return estimatorFunctions[currentEstimator].estimatorEnqueuePosition(pos);
  }

  return false;
}

bool estimatorEnqueuePose(const poseMeasurement_t *pose) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueuePose) {
    return estimatorFunctions[currentEstimator].estimatorEnqueuePose(pose);
  }

  return false;
}

bool estimatorEnqueueDistance(const distanceMeasurement_t *dist) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueDistance) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueDistance(dist);
  }

  return false;
}

bool estimatorEnqueueTOF(const tofMeasurement_t *tof) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueTOF) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueTOF(tof);
  }

  return false;
}

bool estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight(height);
  }

  return false;
}

bool estimatorEnqueueFlow(const flowMeasurement_t *flow) {
  if (estimatorFunctions[currentEstimator].estimatorEnqueueFlow) {
    return estimatorFunctions[currentEstimator].estimatorEnqueueFlow(flow);
  }

  return false;
}

