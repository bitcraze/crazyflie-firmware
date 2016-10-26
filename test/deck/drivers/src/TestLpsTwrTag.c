// File under test lpsTwrTag.h
#include "lpsTwrTag.h"

#include <string.h>
#include "unity.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"

// TODO krri really want extern or move to .h file?
extern uwbAlgorithm_t uwbTwrTagAlgorithm;

// TODO krri defined in lpsTwrTag.c. Move to where they are accessable?
#define POLL 0x01   // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

#define TYPE 0
#define SEQ 1

#define TAG_ADDRESS 8


static dwDevice_t dev;
static void mockCallsForInitiateRanging(uint8_t expCurrSeq, uint8_t expAnchor);
static void dwGetData_ExpectAndCopyData(const packet_t* rxPacket, unsigned int expDataLength);

void setUp(void) {
  uwbTwrTagAlgorithm.init(&dev);
}

void testEventReceiveUnhandledEventShouldAssertFailure() {
  // Fixture
  assertFail_Expect("", "", 0);
  assertFail_IgnoreArg_exp();
  assertFail_IgnoreArg_file();
  assertFail_IgnoreArg_line();

  uwbEvent_t unknownEvent = 100;

  // Test
  uwbTwrTagAlgorithm.onEvent(&dev, unknownEvent);

  // Assert
  // Mock automatically validated after test
}

void testEventReceiveFailedShouldReturnZero() {
  // Fixture

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventReceiveFailed);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventReceiveTimeoutShouldReturnZero() {
  // Fixture

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventReceiveTimeout);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventTimeoutShouldInitiateRanging() {
  // Fixture
  uint8_t expectedSeqNr = 1;
  uint8_t expectedAnchor = 2;
  mockCallsForInitiateRanging(expectedSeqNr, expectedAnchor);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventTimeout);

  // Assert
  const uint32_t expected = MAX_TIMEOUT;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventPacketReceivedWithZeroDataLengthShouldReturnZero() {
  // Fixture
  dwGetDataLength_ExpectAndReturn(&dev, 0);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventPacketReceivedWithWrongDestinationAddressShouldReturnMAX_TIMEOUT() {
  // Fixture
  packet_t rxPacket = {.destAddress = {47, 11, 47, 11, 47, 11, 47, 11}};
  const int dataLength = sizeof(rxPacket);

  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&rxPacket, dataLength);

  dwNewReceive_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwStartReceive_Expect(&dev);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = MAX_TIMEOUT;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


///////////////////////////////////////////////////////////////////////////////

static uint8_t expectedCurrSeq;
static const uint8_t expectedTagAddress[] = {TAG_ADDRESS, 0, 0, 0, 0, 0, 0xcf, 0xbc};
static uint8_t expectedBaseAddress[] = {0, 0, 0, 0, 0, 0, 0xcf, 0xbc};

void dwSetDataMockCallback(dwDevice_t* actualDev, uint8_t* actualData, unsigned int actualN, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_INT(0, cmock_num_calls);
  TEST_ASSERT_EQUAL_PTR(&dev, actualDev);
  TEST_ASSERT_EQUAL_INT(MAC802154_HEADER_LENGTH + 2, actualN);

  packet_t *txPacket = (packet_t*)actualData;
  TEST_ASSERT_EQUAL_UINT8(POLL, txPacket->payload[TYPE]);
  TEST_ASSERT_EQUAL_UINT8(expectedCurrSeq, txPacket->payload[SEQ]);
  TEST_ASSERT_EQUAL_MEMORY(expectedTagAddress, txPacket->sourceAddress, sizeof(expectedTagAddress));
  TEST_ASSERT_EQUAL_MEMORY(expectedBaseAddress, txPacket->destAddress, sizeof(expectedBaseAddress));
}

static void mockCallsForInitiateRanging(uint8_t expCurrSeq, uint8_t expAnchor) {
  dwIdle_Expect(&dev);
  dwNewTransmit_Expect(&dev);
  dwSetDefaults_Expect(&dev);

  dwSetData_StubWithCallback(dwSetDataMockCallback);
  expectedCurrSeq = expCurrSeq;
  expectedBaseAddress[0] = expAnchor;

  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);
}

// dwGetData mock /////////////////////////////////////////////////////////////

static unsigned int dwGetDataExpectedDataLength;
static packet_t dwGetDataRxPacket;

static void dwGetDataMockCallback(dwDevice_t* actualDev, uint8_t* data, unsigned int actualDataLength, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_INT(0, cmock_num_calls);
  TEST_ASSERT_EQUAL_PTR(&dev, actualDev);
  TEST_ASSERT_EQUAL_UINT(dwGetDataExpectedDataLength, actualDataLength);

  memcpy(data, &dwGetDataRxPacket, sizeof(actualDataLength));
}

static void dwGetData_ExpectAndCopyData(const packet_t* rxPacket, unsigned int expDataLength) {
  dwGetDataExpectedDataLength = expDataLength;
  memcpy(&dwGetDataRxPacket, rxPacket, sizeof(dwGetDataRxPacket));

  dwGetData_StubWithCallback(dwGetDataMockCallback);
}