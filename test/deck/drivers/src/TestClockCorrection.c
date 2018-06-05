#include "clockCorrectionFunctions.h"
#include "clockCorrectionStorage.h"

#include "unity.h"

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
