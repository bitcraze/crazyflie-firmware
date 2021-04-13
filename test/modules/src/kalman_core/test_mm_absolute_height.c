// File under test mm_absolute_height.c
#include "mm_absolute_height.h"

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


void testThatCorrectValuesAreUsedInScalatUpdate() {
  // Fixture
  float currentZ = 47.11f;
  float measuredHeight = 12.34;
  float expectedError = measuredHeight - currentZ;
  float expectedStdMeasNoise = 0.123f;

  this.S[KC_STATE_Z] = currentZ;
  h[KC_STATE_Z] = 1.0f;

  heightMeasurement_t measurement = {
    .height = measuredHeight,
    .stdDev = expectedStdMeasNoise,
  };

  setKalmanCoreScalarUpdateExpectationsSingleCall(&this, h, expectedError, expectedStdMeasNoise);

  // Test
  kalmanCoreUpdateWithAbsoluteHeight(&this, &measurement);

  // Assert
  // Checked in mock
}
