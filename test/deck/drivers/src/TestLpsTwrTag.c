#include "unity.h"
#include "lpsTwrTag.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"

// TODO krri really want extern or move to .h file?
extern uwbAlgorithm_t uwbTwrTagAlgorithm;


void testEventReceiveFailedShouldReturnZero() {
  // Fixture
  dwDevice_t dev;
  uwbTwrTagAlgorithm.init(&dev);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventReceiveFailed);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


void testEventReceiveTimeoutShouldReturnZero() {
  // Fixture
  dwDevice_t dev;
  uwbTwrTagAlgorithm.init(&dev);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventReceiveTimeout);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}
