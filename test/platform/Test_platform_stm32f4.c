// File under test platform_stm32f4.c

// @MODULE "platform_stm32f4.c"
#include "platform.h" // @NO_MODULE

#include "unity.h"
#include <string.h>


#define PLATFORM_INFO_OTP_NR_OF_BLOCKS 16
#define PLATFORM_INFO_OTP_BLOCK_LEN 32
static char mockOtpBlocks[PLATFORM_INFO_OTP_NR_OF_BLOCKS][PLATFORM_INFO_OTP_BLOCK_LEN];

static void fixtureInitOtpBlocks();
static void fixtureSetOtpBlockString(const int blockNr, const char* content);
static void fixtureClearOtpBlock(const int blockNr);

void setUp(void) {
  fixtureInitOtpBlocks();
}

void tearDown(void) {
  // Empty
}

void testThatFirstBlockIsReturnedIfSet() {
  // Fixture
  fixtureSetOtpBlockString(0, "0;CF21");

  char actual[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];

  // Test
  platformGetDeviceTypeString(actual);

  // Assert
  TEST_ASSERT_EQUAL_STRING("0;CF21", actual);
}

void testThatLAterBlockIsReturnedIfEarlierBlockAreCleared() {
  // Fixture
  fixtureClearOtpBlock(0);
  fixtureClearOtpBlock(1);
  fixtureClearOtpBlock(2);
  fixtureSetOtpBlockString(3, "0;CF21");

  char actual[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];

  // Test
  platformGetDeviceTypeString(actual);

  // Assert
  TEST_ASSERT_EQUAL_STRING("0;CF21", actual);
}

void testThatDefaultPlatformStringIsCF20IfNoInfoIsSet() {
  // Fixture
  char actual[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];

  // Test
  platformGetDeviceTypeString(actual);

  // Assert
  TEST_ASSERT_EQUAL_STRING("0;CF20", actual);
}

void testThatDefaultPlatformStringIsCF20IfAllBlocksAreCleared() {
  // Fixture
  memset(mockOtpBlocks, 0x00, sizeof(mockOtpBlocks));

  char actual[PLATFORM_DEVICE_TYPE_STRING_MAX_LEN];

  // Test
  platformGetDeviceTypeString(actual);

  // Assert
  TEST_ASSERT_EQUAL_STRING("0;CF20", actual);
}

void testThatNoMoreThanTheBlockSizeIsCopiedIfTheBlockIsNotNullTerminated() {
  // Fixture
  // Set the block to a string that is longer than the block size of 32
  fixtureSetOtpBlockString(0, "0123456789012345678901234567890123456789");

  char actual[100];

  // Test
  platformGetDeviceTypeString(actual);

  // Assert
  TEST_ASSERT_EQUAL_STRING("01234567890123456789012345678901", actual);
}



// Replaces function in file under test
char* getAddressOfOtpMemoryBlock(const int blockNr) {
  return mockOtpBlocks[blockNr];
}


// Fixtures

static void fixtureInitOtpBlocks() {
  memset(mockOtpBlocks, 0xff, sizeof(mockOtpBlocks));
}

static void fixtureSetOtpBlockString(const int blockNr, const char* content) {
  strcpy(mockOtpBlocks[blockNr], content);
}

static void fixtureClearOtpBlock(const int blockNr) {
  memset(mockOtpBlocks[blockNr], 0, PLATFORM_INFO_OTP_BLOCK_LEN);
}
