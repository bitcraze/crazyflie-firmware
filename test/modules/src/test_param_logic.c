// File under test param_logic.c
#include "param_logic.h"

#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <errno.h>

#include "unity.h"

#include "mock_crtp.h"
#include "mock_storage.h"
#include "crc32.h"

// linker symbols mock
int _sdata;
int _edata;
int _sidata;
int _stext;
int _etext;

struct param_s *_param_start;
struct param_s *_param_stop;


// Fixture for parameters
static uint8_t myUint8 = 0;
static uint16_t myUint16 = 0;
static uint32_t myUint32 = 0;
static int8_t myInt8 = 0;
static int16_t myInt16 = 0;
static int32_t myInt32 = 0;
static int32_t myPersistent = 0;
static float myPersistentFloat = 0;
static int8_t myShortPersistent = 0;
static float myFloat = 0.0f;

// Storage fetch mock
static int32_t* fetchMockBuffer = 0;
static size_t fetchMockBufferLength = 0;
static char* fetchMockExpectedKey = "";


PARAM_GROUP_START(myGroup)
PARAM_ADD(PARAM_UINT8, myUint8, &myUint8)
PARAM_ADD(PARAM_UINT16, myUint16, &myUint16)
PARAM_ADD(PARAM_UINT32, myUint32, &myUint32)
PARAM_ADD(PARAM_INT8, myInt8, &myInt8)
PARAM_ADD(PARAM_INT16, myInt16, &myInt16)
PARAM_ADD(PARAM_INT32, myInt32, &myInt32)
PARAM_ADD(PARAM_FLOAT, myFloat, &myFloat)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, myPersistent, &myPersistent)
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, myPersistentFloat, &myPersistentFloat)
PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, myShortPersistent, &myShortPersistent)
PARAM_GROUP_STOP(myGroup)

CRTPPacket replyPk;

static int crtpReply(CRTPPacket* p, int cmock_num_calls)
{
  memcpy(&replyPk, p, sizeof(CRTPPacket));

  return 0;
}

void setUp(void) {
  _param_start = __params_myGroup;
  _param_stop = _param_start + (sizeof(__params_myGroup) / sizeof(struct param_s));

  // Set up storage fetch mock
  fetchMockBuffer = 0;
  fetchMockBufferLength = 0;
  fetchMockExpectedKey = "";

  paramLogicInit();
}

void tearDown(void) {
  // Empty
}

void testSetUint8(void) {
  // Fixture
  uint8_t expected = UINT8_MAX - 1;

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);
  paramVarId_t varid = paramGetVarId("myGroup", "myUint8");

  // Test
  paramSetInt(varid, expected);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myUint8);
}

void testSetUint16(void) {
  // Fixture
  uint16_t expected = UINT16_MAX - 1;

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);
  paramVarId_t varid = paramGetVarId("myGroup", "myUint16");

  // Test
  paramSetInt(varid, expected);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myUint16);
}

void testSetUint32(void) {
  // Fixture
  uint32_t expected = UINT32_MAX - 1;

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);
  paramVarId_t varid = paramGetVarId("myGroup", "myUint32");

  // Test
  paramSetInt(varid, expected);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myUint32);
}

void testSetInt8(void) {
  // Fixture
  uint8_t expected = INT8_MAX - 1;

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);
  paramVarId_t varid = paramGetVarId("myGroup", "myInt8");

  // Test
  paramSetInt(varid, expected);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myInt8);
}

void testSetInt16(void) {
  // Fixture
  uint16_t expected =UINT16_MAX - 1;

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);
  paramVarId_t varid = paramGetVarId("myGroup", "myInt16");

  // Test
  paramSetInt(varid, expected);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myInt16);
}

void testSetInt32(void) {
  // Fixture
  uint32_t expected = INT32_MAX - 1;

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);
  paramVarId_t varid = paramGetVarId("myGroup", "myInt32");

  // Test
  paramSetInt(varid, expected);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myInt32);
}


void testGetUint8(void) {
  // Fixture
  uint8_t expected = UINT8_MAX;

  myUint8 = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myUint8");

  // Test
  const uint8_t actual = paramGetInt(varid);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, actual);
}

void testGetUint16(void) {
  // Fixture
  uint16_t expected = UINT16_MAX;

  myUint16 = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myUint16");

  // Test
  const uint16_t actual = paramGetInt(varid);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}

void testGetUint32(void) {
  // Fixture
  uint32_t expected = UINT32_MAX;

  myUint32 = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myUint32");

  // Test
  const uint32_t actual = paramGetInt(varid);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testGetInt8(void) {
  // Fixture
  int8_t expected = INT8_MIN;

  myInt8 = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myInt8");

  // Test
  const int8_t actual = paramGetInt(varid);

  // Assert
  TEST_ASSERT_EQUAL_INT8(expected, actual);
}

void testGetInt16(void) {
  // Fixture
  int16_t expected = INT16_MIN;

  myInt16 = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myInt16");

  // Test
  const int16_t actual = paramGetInt(varid);

  // Assert
  TEST_ASSERT_EQUAL_INT16(expected, actual);
}

void testGetInt32(void) {
  // Fixture
  int32_t expected = INT32_MIN;

  myInt32 = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myInt32");

  // Test
  const int32_t actual = paramGetInt(varid);

  // Assert
  TEST_ASSERT_EQUAL_INT32(expected, actual);
}

void testGetFloat(void) {
  // Fixture
  float expected = 47.11;
  myFloat = expected;
  paramVarId_t varid = paramGetVarId("myGroup", "myFloat");

  // Test
  const float actual = paramGetFloat(varid);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void testPersistentSetGetFloat(void) {
  // Fixture
  float expected = 10.88f;
  float actual;

  paramVarId_t varid = paramGetVarId("myGroup", "myPersistentFloat");

  crtpIsConnected_IgnoreAndReturn(0);
  crtpSendPacket_StubWithCallback(crtpReply);

  // Test
  paramSetFloat(varid, expected);
  actual = paramGetFloat(varid);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

void testReadProcessUint8(void) {
  // Fixture
  CRTPPacket testPk;
  myUint8 = UINT8_MAX;
  // Id 0 which is first param i list, myUint8
  testPk.data[0] = 0;
  testPk.data[1] = 0;

  crtpSendPacketBlock_StubWithCallback(crtpReply);

  // Test
  paramReadProcess(&testPk);

  // Assert
  testPk.data[2] = 0;
  testPk.data[3] = UINT8_MAX;
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&testPk.data[0], &replyPk.data[0], 3);
}

void testWriteProcessUint8(void) {
  // Fixture
  CRTPPacket testPk;
  uint8_t expected = UINT8_MAX - 1;
  // Id 0 which is first param i list, myUint8
  testPk.data[0] = 0;
  testPk.data[1] = 0;
  testPk.data[2] = expected;

  crtpSendPacketBlock_StubWithCallback(crtpReply);

  // Test
  paramWriteProcess(&testPk);

  // Assert
  TEST_ASSERT_EQUAL_UINT8(expected, myUint8);
}

static size_t storageFetchMockFunc(const char *key, void* buffer, size_t length)
{
  TEST_ASSERT_EQUAL_STRING(fetchMockExpectedKey, key);
  size_t cpyLen = length;
  if (fetchMockBufferLength < length) {
    cpyLen = fetchMockBufferLength;
  }

  memcpy(buffer, fetchMockBuffer, cpyLen);

  return cpyLen;
}

void testPersistentGetStateWithStoredParameter(void) {
  // Fixture
  CRTPPacket testPk;

  paramVarId_t varid = paramGetVarId("myGroup", "myPersistent");

  testPk.data[0] = 0;
  testPk.data[1] = varid.id & 0xffu;
  testPk.data[2] = (varid.id >> 8) & 0xffu;

  crtpSendPacketBlock_StubWithCallback(crtpReply);

  uint32_t storageBuffer = 0x01020304;
  fetchMockBuffer = &storageBuffer;
  fetchMockBufferLength = 4;
  fetchMockExpectedKey = "prm/myGroup.myPersistent";
  storageFetch_StubWithCallback(storageFetchMockFunc);

  // Test
  paramPersistentGetState(&testPk);

  // Assert
  testPk.size = 12;
  testPk.data[3] = PARAM_PERSISTENT_STORED;

  // Default value should be 0. Can not be tested in unit test.
  testPk.data[4] = 0;
  testPk.data[5] = 0;
  testPk.data[6] = 0;
  testPk.data[7] = 0;

  testPk.data[8] = 0x04;
  testPk.data[9] = 0x03;
  testPk.data[10] = 0x02;
  testPk.data[11] = 0x01;

  TEST_ASSERT_EQUAL_UINT8(testPk.size, replyPk.size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&testPk.data[0], &replyPk.data[0], replyPk.size);
}

void testPersistentGetStateWithShortStoredParameter(void) {
  // Fixture
  CRTPPacket testPk;

  paramVarId_t varid = paramGetVarId("myGroup", "myShortPersistent");

  testPk.data[0] = 0;
  testPk.data[1] = varid.id & 0xffu;
  testPk.data[2] = (varid.id >> 8) & 0xffu;

  crtpSendPacketBlock_StubWithCallback(crtpReply);

  uint32_t storageBuffer = 0x17;
  fetchMockBuffer = &storageBuffer;
  fetchMockBufferLength = 1;
  fetchMockExpectedKey = "prm/myGroup.myShortPersistent";
  storageFetch_StubWithCallback(storageFetchMockFunc);

  // Test
  paramPersistentGetState(&testPk);

  // Assert
  testPk.size = 6;

  testPk.data[3] = PARAM_PERSISTENT_STORED;

  // Default value should be 0. Can not be tested in unit test.
  testPk.data[4] = 0;

  testPk.data[5] = 0x17;
  TEST_ASSERT_EQUAL_UINT8(testPk.size, replyPk.size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&testPk.data[0], &replyPk.data[0], replyPk.size);
}

void testPersistentGetStateWithoutStoredParameter(void) {
  // Fixture
  CRTPPacket testPk;

  paramVarId_t varid = paramGetVarId("myGroup", "myPersistent");

  testPk.data[0] = 0;
  testPk.data[1] = varid.id & 0xffu;
  testPk.data[2] = (varid.id >> 8) & 0xffu;

  crtpSendPacketBlock_StubWithCallback(crtpReply);

  fetchMockBufferLength = 0;
  fetchMockExpectedKey = "prm/myGroup.myPersistent";
  storageFetch_StubWithCallback(storageFetchMockFunc);

  // Test
  paramPersistentGetState(&testPk);

  // Assert
  testPk.size = 8;

  testPk.data[3] = PARAM_PERSISTENT_NOT_STORED;

  // Default value should be 0. Can not be tested in unit test.
  testPk.data[4] = 0;
  testPk.data[5] = 0;
  testPk.data[6] = 0;
  testPk.data[7] = 0;
  TEST_ASSERT_EQUAL_UINT8(testPk.size, replyPk.size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&testPk.data[0], &replyPk.data[0], replyPk.size);
}


void testPersistentGetStateWithNonExistingParameter(void) {
  // Fixture
  CRTPPacket testPk;

  testPk.data[0] = 0;
  testPk.data[1] = 0x47;
  testPk.data[2] = 0x11;

  crtpSendPacketBlock_StubWithCallback(crtpReply);

  // Test
  paramPersistentGetState(&testPk);

  // Assert
  testPk.size = 4;

  testPk.data[3] = ENOENT;

  TEST_ASSERT_EQUAL_UINT8(testPk.size, replyPk.size);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(&testPk.data[0], &replyPk.data[0], replyPk.size);
}
