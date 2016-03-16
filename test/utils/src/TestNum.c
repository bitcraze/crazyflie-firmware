#include "unity.h"
#include "num.h"


void testThatLimitUint16NotLimitInRange() {
  // Fixture
  int32_t value = 4711;
  uint16_t expected = 4711;

  // Test
  uint16_t actual = limitUint16(value);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}


void testThatLimitUint16LimitZero() {
  // Fixture
  int32_t value = -100;
  uint16_t expected = 0;

  // Test
  uint16_t actual = limitUint16(value);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}


void testThatLimitUint16LimitMax() {
  // Fixture
  int32_t value = UINT16_MAX + 100;
  uint16_t expected = UINT16_MAX;

  // Test
  uint16_t actual = limitUint16(value);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}


void testThatConstrainDoesNotLimitInRange() {
  // Fixture
  float value = 3.45;
  float expected = value;

  // Test
  float actual = constrain(value, 0.0, 100.0);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatConstrainLimitsMin() {
  // Fixture
  float value = -10.0;
  float min = 3.45;
  float expected = min;

  // Test
  float actual = constrain(value, min, 100.0);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatConstrainLimitsMax() {
  // Fixture
  float value = 10.0;
  float max = 3.45;
  float expected = max;

  // Test
  float actual = constrain(value, -100.0, max);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandInDeadbandPositive() {
  // Fixture
  float value = 0.5;
  float threshold = 1.0;
  float expected = 0.0;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandInDeadbandNegative() {
  // Fixture
  float value = -0.5;
  float threshold = 1.0;
  float expected = 0.0;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandOutsideDeadbandPositive() {
  // Fixture
  float value = 2.5;
  float threshold = 1.0;
  float expected = 1.5;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandOutsideDeadbandNegative() {
  // Fixture
  float value = -2.5;
  float threshold = 1.0;
  float expected = -1.5;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}
