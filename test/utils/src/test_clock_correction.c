// FIle under test
#include "clockCorrectionEngine.h"

#include "unity.h"

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_SPEC_MAX (1.0 + MAX_CLOCK_DEVIATION_SPEC * 2)

#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
#define CLOCK_CORRECTION_FILTER 0.1
#define CLOCK_CORRECTION_BUCKET_MAX 4

void setUp(void) {
}

void tearDown(void) {
}

void testGetClockCorrection() {
  // Fixture
  const double clockCorrection = 12345.6789;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = 0
  };

  // Test
  const double result = clockCorrectionEngineGet(&clockCorrectionStorage);

  // Assert
  const double expectedResult = clockCorrection;
  TEST_ASSERT_EQUAL_DOUBLE(expectedResult, result);
}

void testCalculateClockCorrectionWithValidInputDataWithoutWrapAround() {
  // Fixture
  const double clockCorrection = 1.0057;
  const uint64_t mask = 0xFFFFFFFFFF; // 40 bits
  const uint64_t difference_in_cl_x = 10000;

  const uint64_t old_t_in_cl_x = 1000;
  const uint64_t new_t_in_cl_x = old_t_in_cl_x + difference_in_cl_x; // Does not wrap around
  const uint64_t old_t_in_cl_reference = 56789;
  const uint64_t new_t_in_cl_reference = old_t_in_cl_reference + clockCorrection * difference_in_cl_x; // Does not wrap around

  // Test
  const double result = clockCorrectionEngineCalculate(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  const double expectedClockCorrection = clockCorrection;
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testCalculateClockCorrectionWithValidInputDataWithWrapAround() {
  // Fixture
  const double clockCorrection = 1.0057;
  const uint64_t mask = 0xFFFFFFFFFF; // 40 bits
  const uint64_t difference_in_cl_x = 10000;

  const uint64_t old_t_in_cl_x = mask - difference_in_cl_x / 2;
  const uint64_t new_t_in_cl_x = (old_t_in_cl_x + difference_in_cl_x) & mask; // Wraps around
  const uint64_t old_t_in_cl_reference = 56789;
  const uint64_t new_t_in_cl_reference = old_t_in_cl_reference + clockCorrection * difference_in_cl_x; // Does not wrap around

  // Test
  const double result1 = clockCorrectionEngineCalculate(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);
  // This second part of the test veryfies that when the bit mask has a higher number of bits than the timestamps, the result is wrong if wrap around happens
  const uint64_t wrongMask = 0x1FFFFFFFFFF; // 41 bits
  const double result2 = clockCorrectionEngineCalculate(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, wrongMask);

  // Assert
  const double expectedClockCorrection = clockCorrection;
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result1);
  TEST_ASSERT_NOT_EQUAL(expectedClockCorrection, result2);
}

void testCalculateClockCorrectionWithInvalidInputData() {
  // Fixture
  const uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  const uint64_t old_t_in_cl_x = 1000;
  const uint64_t new_t_in_cl_x = 1000;
  const uint64_t old_t_in_cl_reference = 56789;
  const uint64_t new_t_in_cl_reference = 56789;

  // Test
  const double result = clockCorrectionEngineCalculate(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  const double expectedClockCorrection = -1;
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheSpecs() {
  // Fixture
  const double clockCorrection = 5;
  const unsigned int clockCorrectionBucket = 3;
  const double clockCorrectionCandidate = CLOCK_CORRECTION_SPEC_MAX; // First value out of the specs

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  const double sampleIsReliable = clockCorrectionEngineUpdate(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  const double expectedClockCorrection = clockCorrection;
  const unsigned int expectedClockCorrectionBucket = clockCorrectionBucket - 1;
  TEST_ASSERT_FALSE(sampleIsReliable);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheInnerLimitOfTheSpecsWithEmptyBucket() {
  // Fixture
  const double clockCorrection = 1;
  const unsigned int clockCorrectionBucket = 0;
  const double clockCorrectionCandidate = nextafter(CLOCK_CORRECTION_SPEC_MAX, 1); // First value in the specs

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  const double sampleIsReliable = clockCorrectionEngineUpdate(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  const double expectedClockCorrection = clockCorrectionCandidate;
  const unsigned int expectedClockCorrectionBucket = clockCorrectionBucket;
  TEST_ASSERT_FALSE(sampleIsReliable);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheAcceptableNoiseWithEmptyBucket() {
  // Fixture
  const double clockCorrection = 1;
  const unsigned int clockCorrectionBucket = 0;
  const double clockCorrectionCandidate = clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE; // First value out of acceptable noise

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  const double sampleIsReliable = clockCorrectionEngineUpdate(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  const double expectedClockCorrection = clockCorrectionCandidate;
  const unsigned int expectedClockCorrectionBucket = clockCorrectionBucket;
  TEST_ASSERT_FALSE(sampleIsReliable);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheAcceptableNoiseWithNonEmptyBucket() {
  // Fixture
  const double clockCorrection = 1;
  const unsigned int clockCorrectionBucket = 2;
  const double clockCorrectionCandidate = clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE; // First value out of acceptable noise

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  const double sampleIsReliable = clockCorrectionEngineUpdate(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  const double expectedClockCorrection = clockCorrection;
  const unsigned int expectedClockCorrectionBucket = clockCorrectionBucket - 1;
  TEST_ASSERT_FALSE(sampleIsReliable);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheInnerLimitOfTheAcceptableNoise() {
  // Fixture
  const double clockCorrection = 1.0 + 10e-6; // A value inside the clock specs
  const unsigned int clockCorrectionBucket = 2;
  const double clockCorrectionCandidate = nextafter(clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE, 1); // First value in the acceptable noise

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  const double sampleIsReliable = clockCorrectionEngineUpdate(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  const double expectedClockCorrection = clockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);
  const unsigned int expectedClockCorrectionBucket = clockCorrectionBucket + 1;
  TEST_ASSERT_TRUE(sampleIsReliable);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}
