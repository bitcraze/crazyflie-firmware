#include "cfassert.h"
#include "unity.h"

// Implementation of assert used in unit tests, it triggers an assert in unity that will fail the test.
void assertFail(char *exp, char *file, int line) {
  char buf[150];
  sprintf(buf, "%s in File: \"%s\", line %d\n", exp, file, line);
  TEST_ASSERT_MESSAGE(false, buf);
}
