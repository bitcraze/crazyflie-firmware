#include "clockCorrectionFunctions.h"

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
  double clockCorrection = 12345.6789;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = 0
  };

  // Test
  double result = getClockCorrection(&clockCorrectionStorage);

  // Assert
  double expectedResult = clockCorrection;
  TEST_ASSERT_EQUAL_DOUBLE(expectedResult, result);
}

void testTruncateTimeStampWithBigInputAnd40BitsMask() {
  // Fixture
  uint64_t input = 0xABCDEFABCDEFABCD;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  // Test
  uint64_t result = truncateTimeStamp(input, mask);

  // Assert
  uint64_t expectedResult = 0x000000ABCDEFABCD;
  TEST_ASSERT_EQUAL_UINT64(expectedResult, result);
}

void testTruncateTimeStampWithBigInputAnd32BitsMask() {
  // Fixture
  uint64_t input = 0xABCDEFABCDEFABCD;
  uint64_t mask = 0xFFFFFFFF; // 32 bits

  // Test
  uint64_t result = truncateTimeStamp(input, mask);

  // Assert
  uint64_t expectedResult = 0x00000000CDEFABCD;
  TEST_ASSERT_EQUAL_UINT64(expectedResult, result);
}

void testTruncateTimeStampWithSmallInputAnd40BitsMask() {
  // Fixture
  uint64_t input = 0x0000000000012345;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  // Test
  uint64_t result = truncateTimeStamp(input, mask);

  // Assert
  uint64_t expectedResult = 0x0000000000012345;
  TEST_ASSERT_EQUAL_UINT64(expectedResult, result);
}

void testfillClockCorrectionBucket() {
  // Fixture
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = 0,
    .clockCorrectionBucket = 0
  };

  for (int i = 0; i < 3 * CLOCK_CORRECTION_BUCKET_MAX; i++) {
    // Test
    fillClockCorrectionBucket(&clockCorrectionStorage);

    // Assert
    if (i < CLOCK_CORRECTION_BUCKET_MAX) {
      TEST_ASSERT_EQUAL_UINT(i + 1, clockCorrectionStorage.clockCorrectionBucket);
    } else {
      TEST_ASSERT_EQUAL_UINT(CLOCK_CORRECTION_BUCKET_MAX, clockCorrectionStorage.clockCorrectionBucket);
    }
  }
}

void testEmptyClockCorrectionBucket() {
  // Fixture
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = 0,
    .clockCorrectionBucket = CLOCK_CORRECTION_BUCKET_MAX
  };

  for (int i = CLOCK_CORRECTION_BUCKET_MAX; i > -3 * CLOCK_CORRECTION_BUCKET_MAX; i--) {
    // Test
    bool result = emptyClockCorrectionBucket(&clockCorrectionStorage);

    // Assert
    if (i > 0) {
      TEST_ASSERT_EQUAL_UINT(i - 1, clockCorrectionStorage.clockCorrectionBucket);
      TEST_ASSERT_FALSE(result);
    } else {
      TEST_ASSERT_EQUAL_UINT(0, clockCorrectionStorage.clockCorrectionBucket);
      TEST_ASSERT_TRUE(result);
    }
  }
}

void testCalculateClockCorrectionWithValidInputDataWithoutWrapAround() {
  // Fixture
  double clockCorrection = 1.0057;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits
  uint64_t difference_in_cl_x = 10000;

  uint64_t old_t_in_cl_x = 1000;
  uint64_t new_t_in_cl_x = old_t_in_cl_x + difference_in_cl_x; // Does not wrap around
  uint64_t old_t_in_cl_reference = 56789;
  uint64_t new_t_in_cl_reference = old_t_in_cl_reference + clockCorrection * difference_in_cl_x; // Does not wrap around

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  double expectedClockCorrection = clockCorrection;
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testCalculateClockCorrectionWithValidInputDataWithWrapAround() {
  // Fixture
  double clockCorrection = 1.0057;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits
  uint64_t difference_in_cl_x = 10000;

  uint64_t old_t_in_cl_x = mask - difference_in_cl_x / 2;
  uint64_t new_t_in_cl_x = (old_t_in_cl_x + difference_in_cl_x) & mask; // Wraps around
  uint64_t old_t_in_cl_reference = 56789;
  uint64_t new_t_in_cl_reference = old_t_in_cl_reference + clockCorrection * difference_in_cl_x; // Does not wrap around

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  double expectedClockCorrection = clockCorrection;
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testCalculateClockCorrectionWithInvalidInputData() {
  // Fixture
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  uint64_t old_t_in_cl_x = 1000;
  uint64_t new_t_in_cl_x = 1000;
  uint64_t old_t_in_cl_reference = 56789;
  uint64_t new_t_in_cl_reference = 56789;

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  double expectedClockCorrection = -1;
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheSpecs() {
  // Fixture
  double clockCorrection = 5;
  unsigned int clockCorrectionBucket = 3;
  double clockCorrectionCandidate = CLOCK_CORRECTION_SPEC_MAX; // First value out of the specs

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  double expectedClockCorrection = clockCorrection;
  unsigned int expectedClockCorrectionBucket = clockCorrectionBucket;
  TEST_ASSERT_FALSE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheInnerLimitOfTheSpecsWithEmptyBucket() {
  // Fixture
  double clockCorrection = 1;
  unsigned int clockCorrectionBucket = 0;
  double clockCorrectionCandidate = nextafter(CLOCK_CORRECTION_SPEC_MAX, 1); // First value in the specs

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  double expectedClockCorrection = clockCorrectionCandidate;
  unsigned int expectedClockCorrectionBucket = clockCorrectionBucket + 1;
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheAcceptableNoiseWithEmptyBucket() {
  // Fixture
  double clockCorrection = 1;
  unsigned int clockCorrectionBucket = 0;
  double clockCorrectionCandidate = clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE; // First value out of acceptable noise

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  double expectedClockCorrection = clockCorrectionCandidate;
  unsigned int expectedClockCorrectionBucket = clockCorrectionBucket + 1;
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheAcceptableNoiseWithNonEmptyBucket() {
  // Fixture
  double clockCorrection = 1;
  unsigned int clockCorrectionBucket = 2;
  double clockCorrectionCandidate = clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE; // First value out of acceptable noise

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  double expectedClockCorrection = clockCorrection;
  unsigned int expectedClockCorrectionBucket = clockCorrectionBucket - 1;
  TEST_ASSERT_FALSE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheInnerLimitOfTheAcceptableNoise() {
  // Fixture

  double clockCorrection = 1.0 + 10e-6; // A value inside the clock specs
  unsigned int clockCorrectionBucket = 2;
  double clockCorrectionCandidate = nextafter(clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE, 1); // First value in the acceptable noise

  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = clockCorrection,
    .clockCorrectionBucket = clockCorrectionBucket
  };

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  double expectedClockCorrection = clockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);
  unsigned int expectedClockCorrectionBucket = clockCorrectionBucket + 1;
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}
