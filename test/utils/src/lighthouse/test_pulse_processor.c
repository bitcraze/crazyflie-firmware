// File under test pulse_processor.c
#include "pulse_processor.h"

#include "unity.h"

#include "mock_lighthouse_calibration.h"

void setUp(void) {
}

void testThatResultStructIsCleared() {
  // Fixture
  pulseProcessorResult_t angles;
  angles.sensorMeasurements[2].baseStatonMeasurements[1].validCount = 2;

  // Test
  pulseProcessorClear(&angles, 1);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, angles.sensorMeasurements[2].baseStatonMeasurements[1].validCount);
}
