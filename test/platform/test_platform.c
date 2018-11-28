// File under test platform.c
#include "platform.h"

#include "unity.h"
#include <string.h>


static char actualDeviceType[100];


// Mock implementation
static char* deviceTypeStringToReturn = "";
void platformGetDeviceTypeString(char* deviceTypeString) {
  strcpy(deviceTypeString, deviceTypeStringToReturn);
}

#define CONFIG_COUNT 3
static platformConfig_t fixtureConfig[CONFIG_COUNT];

void setUp(void) {
  strcpy(actualDeviceType, "abcdefghijklmn");
  deviceTypeStringToReturn = "";
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

void testThatFirstMatchingPlatformConfigIsReturned() {
  // Fixture
  deviceTypeStringToReturn = "0;ASD";

  // Test
  int actual = platformInitConfiguration(fixtureConfig, CONFIG_COUNT);

  // Assert
  const char* actualName = platformConfigGetDeviceTypeName();

  TEST_ASSERT_EQUAL(0, actual);
  TEST_ASSERT_EQUAL_STRING("Second", actualName);
}

void testThatErrorIsReturnedWhenDeviceIsNotInConfig() {
  // Fixture
  deviceTypeStringToReturn = "0;WRNG";

  // Test
  int actual = platformInitConfiguration(fixtureConfig, CONFIG_COUNT);

  // Assert
  TEST_ASSERT_EQUAL(1, actual);
}

void testThatItIsNotSearchingOutsideListOfPlatformConfigs() {
  // Fixture
  deviceTypeStringToReturn = "0;ZXC";

  // Test
  int actual = platformInitConfiguration(fixtureConfig, CONFIG_COUNT - 1);

  // Assert
  TEST_ASSERT_EQUAL(1, actual);
}



// Dummy implementations -------------------

const platformConfig_t* platformGetListOfConfigurations(int* nrOfConfigs){
  return 0;
}

void platformInitHardware(){}

// Fixtures -------------------------


static platformConfig_t fixtureConfig[CONFIG_COUNT] = {
  {
    .deviceType = "QWE",
    .deviceTypeName = "First",
  },
  {
    .deviceType = "ASD",
    .deviceTypeName = "Second",
  },
  {
    .deviceType = "ZXC",
    .deviceTypeName = "Third",
  }
};
