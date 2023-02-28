// File under test mm_tdoa.c
#include "mm_tdoa.h"

#include "unity.h"

#include "mock_kalman_core.h"
#include "mock_outlierFilterTdoa.h"
#include "kalman_core_mm_test_helpers.c"

// Default data initialized in setup()
static kalmanCoreData_t this;
static float expectedHm[KC_STATE_DIM];
static OutlierFilterTdoaState_t outlierFilterTdoaState;

void setUp(void) {
  memset(&this, 0, sizeof(this));
  memset(&expectedHm, 0, sizeof(expectedHm));

  initKalmanCoreScalarUpdateExpectationsSingleCall();

  outlierFilterTdoaReset_Ignore();
  outlierFilterTdoaReset(&outlierFilterTdoaState);
}

void tearDown(void) {
  // Empty
}

void testThatScalarUpdateIsCalledInSimpleCase() {
  // Fixture
  float expectedError = 0;
  float expectedStdMeasNoise = 0.123;

  this.S[KC_STATE_X] = 0.0;
  this.S[KC_STATE_Y] = 0.0;
  this.S[KC_STATE_Z] = 0.0;

  expectedHm[KC_STATE_X] = -2.0;
  expectedHm[KC_STATE_Y] = 0.0;
  expectedHm[KC_STATE_Z] = 0.0;

  tdoaMeasurement_t measurement = {
    .anchorPositions = {
      {.x = -1.0, .y = 0.0, .z = 0.0},
      {.x = 1.0, .y = 0.0, .z = 0.0},
    },
    .distanceDiff = 0.0,
    .stdDev = expectedStdMeasNoise,
  };

  setKalmanCoreScalarUpdateExpectationsSingleCall(&this, expectedHm, expectedError, expectedStdMeasNoise);
  outlierFilterTdoaValidateIntegrator_IgnoreAndReturn(true);

  // Test
  kalmanCoreUpdateWithTdoa(&this, &measurement, 0, &outlierFilterTdoaState);

  // Assert
  assertScalarUpdateWasCalled();
}


void testThatSampleWhereDroneIsInSamePositionAsAnchorIsIgnored() {
  // Fixture
  this.S[KC_STATE_X] = -1.0;
  this.S[KC_STATE_Y] = 0.0;
  this.S[KC_STATE_Z] = 0.0;

  tdoaMeasurement_t measurement = {
    .anchorPositions = {
      {.x = -1.0, .y = 0.0, .z = 0.0},
      {.x = 1.0, .y = 0.0, .z = 0.0},
    },
    .distanceDiff = 2.0,
    .stdDev = 0.123,
  };

  // Test
  kalmanCoreUpdateWithTdoa(&this, &measurement, 0, &outlierFilterTdoaState);

  // Assert
  assertScalarUpdateWasNotCalled();
}


void testThatScalarUpdateIsNotCalledWhenTheOutlierFilterIsBlocking() {
  // Fixture
  this.S[KC_STATE_X] = 0.0;
  this.S[KC_STATE_Y] = 0.0;
  this.S[KC_STATE_Z] = 0.0;

  tdoaMeasurement_t measurement = {
    .anchorPositions = {
      {.x = -1.0, .y = 0.0, .z = 0.0},
      {.x = 1.0, .y = 0.0, .z = 0.0},
    },
    .distanceDiff = 2.0,
    .stdDev = 0.123,
  };

  outlierFilterTdoaValidateIntegrator_IgnoreAndReturn(false);

  // Test
  kalmanCoreUpdateWithTdoa(&this, &measurement, 0, &outlierFilterTdoaState);

  // Assert
  assertScalarUpdateWasNotCalled();
}
