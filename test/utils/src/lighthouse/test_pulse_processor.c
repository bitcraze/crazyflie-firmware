// File under test pulse_processor.c
#include "pulse_processor.h"

#include "unity.h"

#include "mock_lighthouse_calibration.h"

void setUp(void) {
}

void testThatResultStructIsCleared() {
  // Fixture
  pulseProcessorResult_t angles;
  angles.sensorMeasurementsLh1[2].baseStatonMeasurements[1].validCount = 2;
  angles.sensorMeasurementsLh2[2].baseStatonMeasurements[1].validCount = 2;

  // Test
  pulseProcessorClear(&angles, 1);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, angles.sensorMeasurementsLh1[2].baseStatonMeasurements[1].validCount);
  TEST_ASSERT_EQUAL_INT(0, angles.sensorMeasurementsLh2[2].baseStatonMeasurements[1].validCount);
}

void testThatTsDiffReturnsDifference() {
  // Fixture
  uint32_t a = 100;
  uint32_t b = 60;
  uint32_t expected = 40;

  // Test
  uint32_t actual = TS_DIFF(a, b);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testThatTsDiffReturnsDifferenceWhenClocksWrap() {
  // Fixture
  uint32_t a = 100;
  uint32_t b = PULSE_PROCESSOR_TIMESTAMP_MAX + 1 - 20;
  uint32_t expected = 120;

  // Test
  uint32_t actual = TS_DIFF(a, b);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testThatTsDiffReturnsDifferenceWhenDifferenceWraps() {
  // Fixture
  uint32_t a = 40;
  uint32_t b = 60;
  uint32_t expected = PULSE_PROCESSOR_TIMESTAMP_MAX + 1 - 20;

  // Test
  uint32_t actual = TS_DIFF(a, b);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testThatTsAbsDiffGreaterThan() {
  // Fixture
  // Test
  // Assert
  TEST_ASSERT_FALSE(TS_ABS_DIFF_LARGER_THAN(100, 60, 200));
  TEST_ASSERT_FALSE(TS_ABS_DIFF_LARGER_THAN(60, 100, 200));
  TEST_ASSERT_FALSE(TS_ABS_DIFF_LARGER_THAN(100, 60, 40));
  TEST_ASSERT_FALSE(TS_ABS_DIFF_LARGER_THAN(60, 100, 40));

  TEST_ASSERT_TRUE(TS_ABS_DIFF_LARGER_THAN(100, 60, 39));
  TEST_ASSERT_TRUE(TS_ABS_DIFF_LARGER_THAN(60, 100, 39));
  TEST_ASSERT_TRUE(TS_ABS_DIFF_LARGER_THAN(100, 60, 1));
  TEST_ASSERT_TRUE(TS_ABS_DIFF_LARGER_THAN(60, 100, 1));

  TEST_ASSERT_FALSE(TS_ABS_DIFF_LARGER_THAN(PULSE_PROCESSOR_TIMESTAMP_MAX + 1 - 10, 30, 100));
  TEST_ASSERT_TRUE(TS_ABS_DIFF_LARGER_THAN(PULSE_PROCESSOR_TIMESTAMP_MAX + 1 - 10, 30, 1));
}
