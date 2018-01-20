#include "unity.h"
#include "eprintf.h"

#include <stdarg.h>
#include <string.h>

static int putc_mock(int c);
static void verify_stdio(char* format, ...);
static char actual[100];

void setUp(void) {
  memset(actual, 0, sizeof(actual));
}

void tearDown(void) {}

void testThatTextIsPrinted() {
  // Fixture
  char* expected = "Some text\n";

  // Test
  eprintf(putc_mock, "Some text\n");

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
}

void testThatTheNumberOfCharactersIsReturned() {
  // Fixture
  int expected = 10;

  // Test
  int actual = eprintf(putc_mock, "Some text\n");

  // Assert
  TEST_ASSERT_EQUAL_INT(expected, actual);
}

void testThatStringIsPrinted() {
  // Fixture
  // Test
  // Assert
  verify_stdio("Some %s text", "extra");
}

void testThatIntIsPrinted() {
  // Fixture
  int val = -456;

  // Test
  // Assert
  verify_stdio("Some %i text", val);
}

void testThatIntIsPrinted2() {
  // Fixture
  int val = 123;

  // Test
  // Assert
  verify_stdio("Some %d text", val);
}

void testThatUnsignedIntIsPrinted() {
  // Fixture
  unsigned int val = 10;

  // Test
  // Assert
  verify_stdio("Some %u text", val);
}

void testThatLongUnsignedIntIsPrinted() {
  // Fixture
  long unsigned int val = 1234567890;

  // Test
  // Assert
  verify_stdio("Some %lu text", val);
}

void testThatHexIsPrinted() {
  // Fixture
  int val = 0xABCD;

  // Note: Not behaving as sprintf() that generates "Some abcd text"
  char* expected = "Some ABCD text";

  // Test
  eprintf(putc_mock, "Some %x text", val);

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
}

void testThatHexIsPrinted2() {
  // Fixture
  int val = 0xABCD;

  // Test
  // Assert
  verify_stdio("Some %X text", val);
}

void testThatHexWithWidthIsPrinted() {
  // Fixture
  int val = 0xab;

  // Test
  // Assert
  verify_stdio("Some %4X text", val);
}

void testThatHexWithZeroPaddedWidthIsPrinted() {
  // Fixture
  int val = 0xab;

  // Test
  // Assert
  verify_stdio("Some %08X text", val);
}

void testThatDoubleWithPrecisionIsPrinted() {
  // Fixture
  double val = -1234.1234;

  // Test
  // Assert
  verify_stdio("Some %.2f text", val);
}

void testThatMultipleParamsArePrinted() {
  // Fixture

  // Test
  // Assert
  verify_stdio("Bla %i %d text %s %03x %5x", 10, 20, "hello", 4, 5);
}

//////////////////////////////

static int putc_mock(int c) {
  uint32_t i = 0;
  for (; actual[i] != 0 && i < sizeof(actual); i++) {}
  actual[i] = (char)c;
  return 1;
}

static void verify_stdio(char* format, ...) {
  va_list ap;

  // Fixture
  char* expected[100];

  // Trust sprintf() to be correct
  va_start(ap, format);
  vsprintf((char*)expected, format, ap);
  va_end(ap);

  // Test
  va_start(ap, format);
  evprintf(putc_mock, format, ap);
  va_end(ap);

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
}
