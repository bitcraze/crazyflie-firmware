#include "clockCorrectionFunctions.h"

#include "unity.h"

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

void testTruncateTimeStampFromDW1000WithBigInput() {
  // Fixture
  uint64_t input = 0xABCDEFABCDEFABCD;
  uint64_t expectedResult = 0x000000ABCDEFABCD;

  // Test
  uint64_t result = truncateTimeStampFromDW1000(input);

  // Assert
  TEST_ASSERT_EQUAL_UINT64(expectedResult, result);
}

void testTruncateTimeStampFromDW1000WithSmallInput() {
  // Fixture
  uint64_t input = 0x0000000000012345;
  uint64_t expectedResult = 0x0000000000012345;

  // Test
  uint64_t result = truncateTimeStampFromDW1000(input);

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

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x);

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

  // Test
  double result = calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x);

  // Assert
  TEST_ASSERT_EQUAL_DOUBLE(expectedClockCorrection, result);
}
