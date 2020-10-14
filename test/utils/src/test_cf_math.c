// File under test
#include "cf_math.h"
#include "unity.h"

void testThatClip1ReturnsValueInRange() {
  // Fixture
  float expected = 0.123f;

  // Test
  float actual = clip1(expected);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void testThatClip1Returns1ForLargeValues() {
    // Fixture
    float expected = 1.0f;

    // Test
    float actual = clip1(2.0f);

    // Assert
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void testThatClip1ReturnsMinus1ForNegativeLargeValues() {
    // Fixture
    float expected = -1.0f;

    // Test
    float actual = clip1(-2.0f);

    // Assert
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void testThatLimPosReturnsValueInRange() {
    // Fixture
    float expected = 1.0f;

    // Test
    float actual = limPos(expected);

    // Assert
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void testThatLimPosReturnsZeroForNegativeValues() {
    // Fixture
    float expected = 0.0f;

    // Test
    float actual = limPos(-1.0f);

    // Assert
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}
