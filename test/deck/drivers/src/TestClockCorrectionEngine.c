#include "clockCorrectionEngine.h"
#include "clockCorrectionFunctions.h"

#include "unity.h"

void setUp(void) {
}

void tearDown(void) {
}

void testCalculateClockCorrectionWithWrapAround() {
  // Fixture
  double clockCorrection = 1.0057;
  uint64_t mask = 0xFFFFFFFF; // 32 bits
  uint64_t difference_in_cl_x = 10000;

  uint64_t old_t_in_cl_x = mask - difference_in_cl_x / 2;
  uint64_t new_t_in_cl_x = (old_t_in_cl_x + difference_in_cl_x) & mask; // Wraps around
  uint64_t old_t_in_cl_reference = 2345;
  uint64_t new_t_in_cl_reference = old_t_in_cl_reference + clockCorrection * difference_in_cl_x; // Does not wrap around

  // Test
  clockCorrectionEngine.init(32); // The number of bits of the mask
  double result1 = clockCorrectionEngine.calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x);

  // This second part of the test veryfies that, when the engine is configured with a higher number of bit mask than what the timestamps have, the result is wrong if wrap around happens
  clockCorrectionEngine.init(33); // The number of bits of the mask, plus one
  double result2 = clockCorrectionEngine.calculateClockCorrection(new_t_in_cl_reference, old_t_in_cl_reference, new_t_in_cl_x, old_t_in_cl_x);

  // Assert
  TEST_ASSERT_EQUAL_DOUBLE(clockCorrection, result1);
  TEST_ASSERT_NOT_EQUAL(clockCorrection, result2);
}
