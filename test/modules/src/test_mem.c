// File under test mem.c
#include "mem.h"

#include "unity.h"


// Memory handler ------------------------------------

static uint32_t handleMemGetSize(const uint8_t internal_id) { return 17; }
static bool handleMemRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

static const MemoryHandlerDef_t memoryDef = {
  .type = MEM_TYPE_APP,
  .getSize = handleMemGetSize,
  .read = handleMemRead,
  .write = handleMemWrite,
};

static const uint32_t READ_ADDRESS = 47;
static const uint32_t READ_LEN = 11;
static const uint32_t WRITE_ADDRESS = 87;
static const uint32_t WRITE_LEN = 65;


void setUp(void) {
  memReset();
  memInit();
}

void tearDown(void) {
  // Empty
}

// The tester is id 0 by default
const uint16_t memTesterId = 0;

void testThatTheTestHandlerTypeIsRegistered() {
  // Fixture
  // Test
  MemoryType_t actual = memGetType(memTesterId);

  // Assert
  TEST_ASSERT_EQUAL(MEM_TYPE_TESTER, actual);
}

void testTheNumberOfRegisteredHandlersForTheTestHandler() {
  // Fixture
  // Only the test handler is registered by default
  uint16_t expected = 1;

  // Test
  uint16_t actual = memGetNrOfMems();

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testThatTheTestHandlerReturnsExpectedData() {
  // Fixture
  uint8_t buffer[3];

  // Test
  memRead(memTesterId, 17, 3, buffer);

  // Assert
  TEST_ASSERT_EQUAL(17, buffer[0]);
  TEST_ASSERT_EQUAL(18, buffer[1]);
  TEST_ASSERT_EQUAL(19, buffer[2]);
}

void testThatAHandlerCanBeRegistered() {
  // Fixture
  uint16_t index = 1;

  // Test
  memoryRegisterHandler(&memoryDef);

  // Assert
  // Two mems expected, the default test handler and the new one
  TEST_ASSERT_EQUAL(2, memGetNrOfMems());
  TEST_ASSERT_EQUAL(17, memGetSize(index));
  TEST_ASSERT_EQUAL(MEM_TYPE_APP, memGetType(index));
}

void testRead() {
  // Fixture
  uint16_t index = 1;
  memoryRegisterHandler(&memoryDef);
  uint8_t buffer[10];

  // Test
  bool actual = memRead(index, READ_ADDRESS, READ_LEN, buffer);

  // Assert
  // Verified in read function
  TEST_ASSERT_TRUE(actual);
}

void testWrite() {
  // Fixture
  uint16_t index = 1;
  memoryRegisterHandler(&memoryDef);
  uint8_t buffer[10];

  // Test
  bool actual = memWrite(index, WRITE_ADDRESS, WRITE_LEN, buffer);

  // Assert
  // Verified in write function
  TEST_ASSERT_TRUE(actual);
}

// --------------------------------

static bool handleMemRead(const uint8_t internal_id, const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  TEST_ASSERT_EQUAL(READ_ADDRESS, memAddr);
  TEST_ASSERT_EQUAL(READ_LEN, readLen);
  return true;
}

static bool handleMemWrite(const uint8_t internal_id, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  TEST_ASSERT_EQUAL(WRITE_ADDRESS, memAddr);
  TEST_ASSERT_EQUAL(WRITE_LEN, writeLen);
  return true;
}