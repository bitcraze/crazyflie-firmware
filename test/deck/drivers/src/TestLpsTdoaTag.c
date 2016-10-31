// File under test lpsTwrTag.h
#include "lpsTdoaTag.h"

#include "unity.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"


static dwDevice_t dev;
static lpsAlgoOptions_t options;


void setUp(void) {
  uwbTdoaTagAlgorithm.init(&dev, &options);
}


void testEventReceiveUnhandledEventShouldAssertFailure() {
  // Fixture
  assertFail_Expect("", "", 0);
  assertFail_IgnoreArg_exp();
  assertFail_IgnoreArg_file();
  assertFail_IgnoreArg_line();

  uwbEvent_t unknownEvent = 100;

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, unknownEvent);

  // Assert
  // Mock automatically validated after test
}
