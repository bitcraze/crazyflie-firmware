// File under test platform.c
#include "platform.h"

#include "unity.h"
#include <string.h>


static char actualDeviceType[100];

void setUp(void) {
  strcpy(actualDeviceType, "abcdefghijklmn");
}

void tearDown(void) {
  // Empty
}

void testThatDeviceTypeStringIsReturned() {
  // Fixture
  const char* deviceTypeString = "0;CF20";

  // Test
  int actual = platformParseDeviceTypeString(deviceTypeString, actualDeviceType);

  // Assert
  TEST_ASSERT_EQUAL_STRING("CF20", actualDeviceType);
  TEST_ASSERT_EQUAL(0, actual);
}

void testThatDeviceTypeStringIsReturnedWithTrailingSemiColonAndKeyValues() {
  // Fixture
  const char* deviceTypeString = "0;CF21;R=C";

  // Test
  int actual = platformParseDeviceTypeString(deviceTypeString, actualDeviceType);

  // Assert
  TEST_ASSERT_EQUAL_STRING("CF21", actualDeviceType);
  TEST_ASSERT_EQUAL(0, actual);
}

void testThatDeviceTypeStringIsReturnedWhenTypeIdentifierIsShorterThan4Chars() {
  // Fixture
  const char* deviceTypeString = "0;AB;R=C";

  // Test
  int actual = platformParseDeviceTypeString(deviceTypeString, actualDeviceType);

  // Assert
  TEST_ASSERT_EQUAL_STRING("AB", actualDeviceType);
  TEST_ASSERT_EQUAL(0, actual);
}

void testThatDeviceTypeIsNotReturnedWhenTypeIdentifierIsTooLong() {
  // Fixture
  const char* deviceTypeString = "0;CF21XXX;R=C";

  // Test
  int actual = platformParseDeviceTypeString(deviceTypeString, actualDeviceType);

  // Assert
  TEST_ASSERT_NOT_EQUAL(0, actual);
}

void testThatDeviceTypeIsNotReturnedIfVersionIfNot0() {
  // Fixture
  const char* deviceTypeString = "1;CF21";

  // Test
  int actual = platformParseDeviceTypeString(deviceTypeString, actualDeviceType);

  // Assert
  TEST_ASSERT_NOT_EQUAL(0, actual);
}

void testThatDeviceTypeIsNotReturnedIfSecondCharIsNotSemicolon() {
  // Fixture
  const char* deviceTypeString = "0+CF21";

  // Test
  int actual = platformParseDeviceTypeString(deviceTypeString, actualDeviceType);

  // Assert
  TEST_ASSERT_NOT_EQUAL(0, actual);
}
