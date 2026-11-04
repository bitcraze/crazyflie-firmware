#include "unity.h"
#include "num.h"

#include <math.h>

static float floatFromBits(uint32_t bits) {
  union {
    uint32_t asBits;
    float asFloat;
  } u;

  u.asBits = bits;
  return u.asFloat;
}


void testThatSingle2halfConvertsZero() {
  // Fixture
  float value = 0.0f;
  uint16_t expected = 0x0000;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsOne() {
  // Fixture
  float value = 1.0f;
  uint16_t expected = 0x3C00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsNegativeOne() {
  // Fixture
  float value = -1.0f;
  uint16_t expected = 0xBC00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsFractionalValue() {
  // Fixture
  float value = 2.5f;
  uint16_t expected = 0x4100;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfHandlesRoundingCarryIntoExponent() {
  // Fixture
  // Mantissa bits [22:13] are all 1 and the rounding bit [12] is 1, so
  // rounding the 23-bit mantissa down to 10 bits overflows and must carry
  // into the exponent field.
  float value = floatFromBits(0x3FFFF000); // 1.99951171875f, rounds to 2.0 in fp16
  uint16_t expected = 0x4000; // 2.0

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsMaxFiniteValue() {
  // Fixture
  float value = 65504.0f; // largest finite value representable in fp16
  uint16_t expected = 0x7BFF;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfClampsPositiveOverflowToInfinity() {
  // Fixture
  float value = 1.0e10f; // exponent too large to fit in fp16
  uint16_t expected = 0x7C00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfClampsNegativeOverflowToInfinity() {
  // Fixture
  float value = -1.0e10f; // exponent too large to fit in fp16
  uint16_t expected = 0xFC00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsPositiveInfinity() {
  // Fixture
  float value = INFINITY;
  uint16_t expected = 0x7C00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsNegativeInfinity() {
  // Fixture
  float value = -INFINITY;
  uint16_t expected = 0xFC00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfConvertsNaN() {
  // Fixture
  float value = NAN;
  uint16_t expected = 0x7E00;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


void testThatSingle2halfFlushesTinyValueToZero() {
  // Fixture
  float value = 1.0e-10f; // normalized, but below fp16's minimum normal magnitude
  uint16_t expected = 0x0000;

  // Test
  uint16_t actual = single2half(value);

  // Assert
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}


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
