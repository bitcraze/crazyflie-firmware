// File under test mm_tof.c
#include "mm_tof.h"

#include <math.h>

#include "unity.h"

#include "mock_kalman_core.h"
#include "kalman_core_mm_test_helpers.c"

static kalmanCoreData_t this;
static float h[KC_STATE_DIM];


void setUp(void) {
  memset(&this, 0, sizeof(this));
  memset(&h, 0, sizeof(h));

  initKalmanCoreScalarUpdateExpectationsSingleCall();
}

void tearDown(void) {
  // Empty
}


void testThatTheFullTiltIsUsedWithoutCone() {
  // Fixture
  float currentZ = 1.5f;
  float tilt = 0.2f; // [rad]
  float measuredDistance = 1.6f;
  float expectedStdMeasNoise = 0.045f;

  this.S[KC_STATE_Z] = currentZ;
  this.R[2][2] = cosf(tilt);

  float angle = fabsf(acosf(this.R[2][2]));
  h[KC_STATE_Z] = 1.0f / cosf(angle);
  float expectedError = measuredDistance - currentZ / cosf(angle);

  tofMeasurement_t measurement = {
    .distance = measuredDistance,
    .stdDev = expectedStdMeasNoise,
    .coneHalfAngle = 0.0f,
  };

  setKalmanCoreScalarUpdateExpectationsSingleCall(&this, h, expectedError, expectedStdMeasNoise);

  // Test
  kalmanCoreUpdateWithTof(&this, &measurement);

  // Assert
  assertScalarUpdateWasCalled();
}

void testThatTheConeIsSubtractedFromTheTilt() {
  // Fixture
  float currentZ = 1.5f;
  float tilt = 0.3f; // [rad]
  float cone = 0.1f; // [rad]
  float measuredDistance = 1.6f;
  float expectedStdMeasNoise = 0.045f;

  this.S[KC_STATE_Z] = currentZ;
  this.R[2][2] = cosf(tilt);

  float angle = fabsf(acosf(this.R[2][2])) - cone;
  h[KC_STATE_Z] = 1.0f / cosf(angle);
  float expectedError = measuredDistance - currentZ / cosf(angle);

  tofMeasurement_t measurement = {
    .distance = measuredDistance,
    .stdDev = expectedStdMeasNoise,
    .coneHalfAngle = cone,
  };

  setKalmanCoreScalarUpdateExpectationsSingleCall(&this, h, expectedError, expectedStdMeasNoise);

  // Test
  kalmanCoreUpdateWithTof(&this, &measurement);

  // Assert
  assertScalarUpdateWasCalled();
}

void testThatATiltWithinTheConeIsTreatedAsLevel() {
  // Fixture
  float currentZ = 1.5f;
  float tilt = 0.05f; // [rad]
  float cone = 0.1f;  // [rad]
  float measuredDistance = 1.6f;
  float expectedStdMeasNoise = 0.045f;

  this.S[KC_STATE_Z] = currentZ;
  this.R[2][2] = cosf(tilt);

  h[KC_STATE_Z] = 1.0f;
  float expectedError = measuredDistance - currentZ;

  tofMeasurement_t measurement = {
    .distance = measuredDistance,
    .stdDev = expectedStdMeasNoise,
    .coneHalfAngle = cone,
  };

  setKalmanCoreScalarUpdateExpectationsSingleCall(&this, h, expectedError, expectedStdMeasNoise);

  // Test
  kalmanCoreUpdateWithTof(&this, &measurement);

  // Assert
  assertScalarUpdateWasCalled();
}
