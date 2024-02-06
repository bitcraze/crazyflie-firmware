// File under test mem.c
#include "mem.h"

#include "unity.h"


// Memory handler ------------------------------------

static uint32_t handleMemGetSize(void) { return 17; }
static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);

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


// One Wire mem handler -------------------------------

static bool handleOwMemGetSerialNr(const uint8_t selectedMem, uint8_t* serialNr);
static bool handleOwMemRead(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData);
static bool handleOwMemWrite(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData);

static MemoryOwHandlerDef_t memoryOwDef = {
  .nrOfMems = 10,
  .size = 47,
  .getSerialNr = handleOwMemGetSerialNr,
  .read = handleOwMemRead,
  .write = handleOwMemWrite,
};


static const uint8_t READ_OW_ID = 55;
static const uint32_t READ_OW_ADDRESS = 123;
static const uint8_t READ_OW_LEN = 222;
static const uint8_t WRITE_OW_ID = 66;
static const uint32_t WRITE_OW_ADDRESS = 345;
static const uint8_t WRITE_OW_LEN = 111;

static const uint8_t SN_OW_ID = 88;



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

void testThatAOwHandlerCanBeRegistered() {
  // Fixture
  // Test
  memoryRegisterOwHandler(&memoryOwDef);

  // Assert
  TEST_ASSERT_EQUAL(10, memGetNrOfOwMems());
  TEST_ASSERT_EQUAL(47, memGetOwSize());
}

void testOwRead() {
  // Fixture
  memoryRegisterOwHandler(&memoryOwDef);
  uint8_t buffer[10];

  // Test
  bool actual = memReadOw(READ_OW_ID, READ_OW_ADDRESS, READ_OW_LEN, buffer);

  // Assert
  // Verified in read function
  TEST_ASSERT_TRUE(actual);
}

void testOwWrite() {
  // Fixture
  memoryRegisterOwHandler(&memoryOwDef);
  uint8_t buffer[10];

  // Test
  bool actual = memWriteOw(WRITE_OW_ID, WRITE_OW_ADDRESS, WRITE_OW_LEN, buffer);

  // Assert
  // Verified in write function
  TEST_ASSERT_TRUE(actual);
}

void testOwSerialNr() {
  // Fixture
  memoryRegisterOwHandler(&memoryOwDef);
  uint8_t buffer[10];

  // Test
  bool actual = memGetOwSerialNr(SN_OW_ID, buffer);

  // Assert
  // Verified in write function
  TEST_ASSERT_TRUE(actual);
}

// --------------------------------

static bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer) {
  TEST_ASSERT_EQUAL(READ_ADDRESS, memAddr);
  TEST_ASSERT_EQUAL(READ_LEN, readLen);
  return true;
}

static bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer) {
  TEST_ASSERT_EQUAL(WRITE_ADDRESS, memAddr);
  TEST_ASSERT_EQUAL(WRITE_LEN, writeLen);
  return true;
}

static bool handleOwMemGetSerialNr(const uint8_t selectedMem, uint8_t* serialNr) {
  TEST_ASSERT_EQUAL(SN_OW_ID, selectedMem);
  return true;
}

static bool handleOwMemRead(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t readLen, uint8_t* startOfData) {
  TEST_ASSERT_EQUAL(READ_OW_ID, selectedMem);
  TEST_ASSERT_EQUAL(READ_OW_ADDRESS, memAddr);
  TEST_ASSERT_EQUAL(READ_OW_LEN, readLen);
  return true;
}

static bool handleOwMemWrite(const uint8_t selectedMem, const uint32_t memAddr, const uint8_t writeLen, const uint8_t* startOfData) {
  TEST_ASSERT_EQUAL(WRITE_OW_ID, selectedMem);
  TEST_ASSERT_EQUAL(WRITE_OW_ADDRESS, memAddr);
  TEST_ASSERT_EQUAL(WRITE_OW_LEN, writeLen);
  return true;
}
