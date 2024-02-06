/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * TestEprintf.c - Unit tests for printing of formatted strings
 */

#include "unity.h"
#include "eprintf.h"

#include <stdarg.h>
#include <string.h>

static int putcMock(int c);
static void verifyStdio(char* format, ...);
static void verify(char* expected, char* format, ...);
static char actual[100];

static void reset() {
  memset(actual, 0, sizeof(actual));
}

void setUp(void) {
  reset();
}

void tearDown(void) {}

void testThatTextIsPrinted() {
  // Fixture
  char* expected = "Some text\n";

  // Test
  eprintf(putcMock, "Some text\n");

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
}

void testThatTheNumberOfCharactersIsReturned() {
  // Fixture
  int expected = 10;

  // Test
  int actual = eprintf(putcMock, "Some text\n");

  // Assert
  TEST_ASSERT_EQUAL_INT(expected, actual);
}

void testThatStringIsPrinted() {
  // Fixture
  // Test
  // Assert
  verifyStdio("Some %s text", "extra");
}

void testThatIntIsPrinted() {
  // Fixture
  int val = -456;

  // Test
  // Assert
  verifyStdio("Some %i text", val);
}

void testThatIntIsPrinted2() {
  // Fixture
  int val = 123;

  // Test
  // Assert
  verifyStdio("Some %d text", val);
}

void testThatLongIntIsPrinted() {
  // Fixture
  long int val = -456123434;

  // Test
  // Assert
  verifyStdio("Some %ld text", val);
}

void testThatLongLongIntIsPrinted() {
  // Fixture
  long long int val = -4561234345345;

  // Test
  // Assert
  verifyStdio("Some %lld text", val);
}

void testThatUnsignedIntIsPrinted() {
  // Fixture
  unsigned int val = 10;

  // Test
  // Assert
  verifyStdio("Some %u text", val);
}

void testThatLongUnsignedIntIsPrinted() {
  // Fixture
  long unsigned int val = 1234567890;

  // Test
  // Assert
  verifyStdio("Some %lu text", val);
}

void testThatLongLongUnsignedIntIsPrinted() {
  // Fixture
  long long unsigned int val = 9223372036854775807;

  // Test
  // Assert
  verifyStdio("Some %llu text", val);
}

void testThatHexIsPrinted() {
  // Fixture
  int val = 0xABCD;

  // Note: Not behaving as sprintf() that generates "Some abcd text"
  char* expected = "Some ABCD text";

  // Test
  eprintf(putcMock, "Some %x text", val);

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
}

void testThatHexIsPrinted2() {
  // Fixture
  unsigned int val = 0xABCD;

  // Test
  // Assert
  verifyStdio("Some %X text", val);
}

void testThatLongHexIsPrinted() {
  // Fixture
  long int val = 0xABCD;

  // Test
  // Assert
  verifyStdio("Some %lX text", val);
}


void testThatLongLongHexIsPrinted() {
  // Fixture
  long long int val = 0xABCD98768FEDC098;

  // Test
  // Assert
  verifyStdio("Some %llX text", val);
}

void testThatHexWithWidthIsPrinted() {
  // Fixture
  int val = 0xab;

  // Test
  // Assert
  verifyStdio("Some %4X text", val);
}

void testThatDoubleIsPrintedWithRoundingErrors() {
  // Fixture
  double val = -1234.12;
  char* expected = "Implementaion has rounding errors, -1234.119995";

  // Test
  // Assert
  verify(expected, "Implementaion has rounding errors, %f", val);
}

void testThatHexWithZeroPaddedWidthIsPrinted() {
  // Fixture
  int val = 0xab;

  // Test
  // Assert
  verifyStdio("Some %08X text", val);
}

void testThatDoubleWithPrecisionIsPrinted() {
  // Fixture
  double val = -1234.1234;

  // Test
  // Assert
  verifyStdio("Some %.2f text", val);
}

void testThatCharIsPrinted() {
  // Fixture
  char val1 = 'z';
  char val2 = 'c';

  // Test
  // Assert
  verifyStdio("Bitcra%ce ro%cks!", val1, val2);
}

void testThatMultipleParamsArePrinted() {
  // Fixture

  // Test
  // Assert
  verifyStdio("Bla %i %d text %s %03x %5x", 10, 20, "hello", 4, 5);
}

void testThatAllIntTypesArePrinted() {
  // Fixture
  // Test
  // Assert
  verify("255", "%u", (uint8_t)255);
  verify("65535", "%u", (uint16_t)65535);
  // second cast needed here because sizeof(unsigned long) != sizeof(uint32_t)
  // on arm64
  verify("4294967295", "%lu", (unsigned long)(uint32_t)4294967295);
  verify("18446744073709551615", "%llu", (uint64_t)18446744073709551615u);

  verify("127", "%i", (int8_t)127);
  verify("32767", "%i", (int16_t)32767);
  // second cast needed here because sizeof(long) != sizeof(int32_t) on arm64
  verify("2147483647", "%li", (long)(int32_t)2147483647);
  verify("9223372036854775807", "%lli", (int64_t)9223372036854775807);

  verify("FF", "%X", (uint8_t)0xFF);
  verify("FFFF", "%X", (uint16_t)0xFFFF);
  // second cast needed here because sizeof(unsigned long) != sizeof(uint32_t) on arm64
  verify("FFFFFFFF", "%lX", (unsigned long)(uint32_t)0xFFFFFFFF);
  verify("FFFFFFFFFFFFFFFF", "%llX", (uint64_t)0xFFFFFFFFFFFFFFFF);
}

void testThatPercentIsPrinted() {
  // Fixture
  // Test
  // Assert
  verify("This is 100% correct", "This is %d%% correct", 100);
}

//////////////////////////////

static int putcMock(int c) {
  uint32_t i = 0;
  for (; actual[i] != 0 && i < sizeof(actual); i++) {}
  actual[i] = (char)c;
  return 1;
}

static void verifyStdio(char* format, ...) {
  // Fixture
  reset();
  va_list ap;
  char* expected[100];

  // Trust sprintf() to be correct
  va_start(ap, format);
  int expected_len = vsprintf((char*)expected, format, ap);
  va_end(ap);

  // Test
  va_start(ap, format);
  int actual_len = evprintf(putcMock, format, ap);
  va_end(ap);

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
  TEST_ASSERT_EQUAL_INT(expected_len, actual_len);
}

static void verify(char* expected, char* format, ...) {
  reset();
  va_list ap;

  // Fixture
  // Trust sprintf() to be correct
  // Test
  va_start(ap, format);
  evprintf(putcMock, format, ap);
  va_end(ap);

  // Assert
  TEST_ASSERT_EQUAL_STRING(expected, actual);
}
