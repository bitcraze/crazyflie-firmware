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
  double expectedResult = 12345.6789;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = expectedResult,
    .clockCorrectionBucket = 0
  };

  // Test
  double result = getClockCorrection(&clockCorrectionStorage);

  // Assert
  TEST_ASSERT_EQUAL_DOUBLE(expectedResult, result);
}

void testTruncateTimeStampFromDW1000WithBigInputAnd40BitsMask() {
  // Fixture
  uint64_t input = 0xABCDEFABCDEFABCD;
  uint64_t expectedResult = 0x000000ABCDEFABCD;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  // Test
  uint64_t result = truncateTimeStamp(input, mask);

  // Assert
  TEST_ASSERT_EQUAL_UINT64(expectedResult, result);
}

void testTruncateTimeStampFromDW1000WithBigInputAnd30BitsMask() {
  // Fixture
  uint64_t input = 0xABCDEFABCDEFABCD;
  uint64_t expectedResult = 0x000000000DEFABCD;
  uint64_t mask = 0x3FFFFFFF; // 30 bits

  // Test
  uint64_t result = truncateTimeStamp(input, mask);

  // Assert
  TEST_ASSERT_EQUAL_UINT64(expectedResult, result);
}

void testTruncateTimeStampFromDW1000WithSmallInputAnd40BitsMask() {
  // Fixture
  uint64_t input = 0x0000000000012345;
  uint64_t expectedResult = 0x0000000000012345;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  // Test
  uint64_t result = truncateTimeStamp(input, mask);

  // Assert
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

void testCalculateClockCorrectionWithValidInputData() {
  // Fixture
  double expectedClockCorrection = 1.005;
  uint64_t old_t_in_cl_x = 1000;
  uint64_t new_t_in_cl_x = 2000;
  uint64_t old_t_in_cl_reference = old_t_in_cl_x * expectedClockCorrection;
  uint64_t new_t_in_cl_reference = new_t_in_cl_x * expectedClockCorrection;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testCalculateClockCorrectionWithInvalidInputData() {
  // Fixture
  double expectedClockCorrection = -1;
  double clockCorrection = 1.005;
  uint64_t old_t_in_cl_x = 1000;
  uint64_t new_t_in_cl_x = 1000;
  uint64_t old_t_in_cl_reference = old_t_in_cl_x * clockCorrection;
  uint64_t new_t_in_cl_reference = new_t_in_cl_x * clockCorrection;
  uint64_t mask = 0xFFFFFFFFFF; // 40 bits

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x, mask);

  // Assert
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheSpecs() {
  // Fixture
  double expectedClockCorrection = 5;
  unsigned int expectedClockCorrectionBucket = 3;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = expectedClockCorrection,
    .clockCorrectionBucket = expectedClockCorrectionBucket
  };
  double clockCorrectionCandidate = CLOCK_CORRECTION_SPEC_MAX; // First value out of the specs

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  TEST_ASSERT_FALSE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheInnerLimitOfTheSpecsWithEmptyBucket() {
  // Fixture
  double expectedClockCorrection = nextafter(CLOCK_CORRECTION_SPEC_MAX, 1); // First value in the specs
  unsigned int expectedClockCorrectionBucket = 1;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = 1,
    .clockCorrectionBucket = 0
  };
  double clockCorrectionCandidate = expectedClockCorrection;

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheAcceptableNoiseWithEmptyBucket() {
  // Fixture
  unsigned int expectedClockCorrectionBucket = 1;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = 1,
    .clockCorrectionBucket = 0
  };
  double expectedClockCorrection = clockCorrectionStorage.clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE; // First value out of acceptable noise
  double clockCorrectionCandidate = expectedClockCorrection;

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheOuterLimitOfTheAcceptableNoiseWithNonEmptyBucket() {
  // Fixture
  double expectedClockCorrection = 1;
  unsigned int expectedClockCorrectionBucket = 1;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = expectedClockCorrection,
    .clockCorrectionBucket = 2
  };
  double clockCorrectionCandidate = clockCorrectionStorage.clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE; // First value out of acceptable noise

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  TEST_ASSERT_FALSE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}

void testUpdateClockCorrectionWithSampleInTheInnerLimitOfTheAcceptableNoise() {
  // Fixture
  unsigned int expectedClockCorrectionBucket = 3;
  clockCorrectionStorage_t clockCorrectionStorage = {
    .clockCorrection = 1.0 + 10e-6, // A value inside the clock specs
    .clockCorrectionBucket = 2
  };
  double clockCorrectionCandidate = nextafter(clockCorrectionStorage.clockCorrection + CLOCK_CORRECTION_ACCEPTED_NOISE, 1); // First value in the acceptable noise
  double expectedClockCorrection = clockCorrectionStorage.clockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);

  // Test
  double result = updateClockCorrection(&clockCorrectionStorage, clockCorrectionCandidate);

  // Assert
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, clockCorrectionStorage.clockCorrection);
  TEST_ASSERT_EQUAL_UINT(expectedClockCorrectionBucket, clockCorrectionStorage.clockCorrectionBucket);
}
