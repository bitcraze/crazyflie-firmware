// File under test lpsTwrTag.h
#include "lpsTwrTag.h"

#include <string.h>
#include "unity.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"

// TODO krri defined in lpsTwrTag.c. Move to where they are accessable?
#define POLL 0x01   // Poll is initiated by the tag
#define ANSWER 0x02
#define FINAL 0x03
#define REPORT 0x04 // Report contains all measurement from the anchor

#define TYPE 0
#define SEQ 1

#define ANTENNA_OFFSET 154.6   // In meter

#define TAG_ADDRESS 8
static const uint8_t expectedTagAddress[] = {TAG_ADDRESS, 0, 0, 0, 0, 0, 0xcf, 0xbc};

static dwDevice_t dev;
static lpsAlgoOptions_t options;
static void mockCallsForInitiateRanging(uint8_t expCurrSeq, uint8_t expAnchor);
static void dwGetData_ExpectAndCopyData(dwDevice_t* expDev, const packet_t* rxPacket, unsigned int expDataLength);
static void dwGetTransmitTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time);
static void dwGetReceiveTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time);

static void setAddress(uint8_t* data, uint8_t addr);
static void setTime(uint8_t* data, const dwTime_t* time);

#define ANTENNA_DELAY (ANTENNA_OFFSET*499.2e6*128)/299792458.0 // In radio tick
static lpsAlgoOptions_t defaultOptions = {
  .tagAddress = TAG_ADDRESS,
  .anchors = {1,2,3,4,5,6},
  .antennaDelay = ANTENNA_DELAY,
  .rangingFailedThreshold = 6
};

static int dwGetDataMockCallIndex = 0;
static int dwGetTransmitTimestampMockCallIndex = 0;
static int dwGetReceiveTimestampMockCallIndex = 0;

static const double C = 299792458.0;       // Speed of light
static const double tsfreq = 499.2e6 * 128;  // Timestamp counter frequency


void setUp(void) {
  dwGetDataMockCallIndex = 0;
  dwGetTransmitTimestampMockCallIndex = 0;
  dwGetReceiveTimestampMockCallIndex = 0;

  memcpy(&options, &defaultOptions, sizeof(options));
  uwbTwrTagAlgorithm.init(&dev, &options);
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
  dwGetData_ExpectAndCopyData(&dev, &rxPacket, dataLength);

  dwNewReceive_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwStartReceive_Expect(&dev);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = MAX_TIMEOUT;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventPacketReceivedWithTypeAnswerAndWrongSeqNrShouldReturn0() {
  // Fixture
  packet_t rxPacket;
  setAddress(rxPacket.destAddress, TAG_ADDRESS);

  rxPacket.payload[TYPE] = ANSWER;

  uint8_t wrongSeqNr = 17; // After init curr_seq = 0
  rxPacket.payload[SEQ] = wrongSeqNr;

  const int dataLength = sizeof(rxPacket);
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, &rxPacket, dataLength);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventPacketReceivedWithTypeReportAndWrongSeqNrShouldReturn0() {
  // Fixture
  packet_t rxPacket;
  setAddress(rxPacket.destAddress, TAG_ADDRESS);

  rxPacket.payload[TYPE] = REPORT;

  uint8_t wrongSeqNr = 17; // After init curr_seq = 0
  rxPacket.payload[SEQ] = wrongSeqNr;

  const int dataLength = sizeof(rxPacket);
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, &rxPacket, dataLength);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testNormalMessageSequenceShouldGenerateDistance() {
  // Fixture
  const int dataLength = sizeof(packet_t);
  const uint8_t expectedSeqNr = 1;
  const uint8_t expectedAnchor = 1;

  float expectedDistance = 5.0;
  const uint32_t distInCycles = expectedDistance * tsfreq / C;

  dwTime_t pollDepartureTagTime = {.full = 123456};
  dwTime_t pollArrivalAnchorTime = {.full = pollDepartureTagTime.full + distInCycles + ANTENNA_DELAY / 2};
  dwTime_t answerDepartureAnchorTime = {.full = pollArrivalAnchorTime.full + 100000};
  dwTime_t answerArrivalTagTime = {.full = answerDepartureAnchorTime.full + distInCycles + ANTENNA_DELAY / 2};
  dwTime_t finalDepartureTagTime = {.full = answerArrivalTagTime.full + 200000};
  dwTime_t finalArrivalAnchorTime = {.full = finalDepartureTagTime.full + distInCycles + ANTENNA_DELAY / 2};


  packet_t expectedTxPacket1;
  memset(&expectedTxPacket1, 0, sizeof(packet_t));
  MAC80215_PACKET_INIT(expectedTxPacket1, MAC802154_TYPE_DATA);
  expectedTxPacket1.pan = 0xbccf;
  expectedTxPacket1.payload[SEQ] = expectedSeqNr;
  expectedTxPacket1.payload[TYPE] = POLL;
  setAddress(expectedTxPacket1.sourceAddress, TAG_ADDRESS);
  setAddress(expectedTxPacket1.destAddress, expectedAnchor + 1); // TODO krri rework addresses

  packet_t rxPacket1;
  setAddress(rxPacket1.sourceAddress, expectedAnchor + 1); // TODO krri rework addresses
  setAddress(rxPacket1.destAddress, TAG_ADDRESS);
  rxPacket1.payload[TYPE] = ANSWER;
  rxPacket1.payload[SEQ] = expectedSeqNr;

  packet_t expectedTxPacket2;
  memset(&expectedTxPacket2, 0, sizeof(packet_t));
  MAC80215_PACKET_INIT(expectedTxPacket2, MAC802154_TYPE_DATA);
  expectedTxPacket2.pan = 0xbccf;
  expectedTxPacket2.payload[SEQ] = expectedSeqNr;
  expectedTxPacket2.payload[TYPE] = FINAL;
  setAddress(expectedTxPacket2.sourceAddress, TAG_ADDRESS);
  setAddress(expectedTxPacket2.destAddress, expectedAnchor + 1); // TODO krri rework addresses

  packet_t rxPacket2;
  setAddress(rxPacket2.sourceAddress, expectedAnchor + 1); // TODO krri rework addresses
  setAddress(rxPacket2.destAddress, TAG_ADDRESS);
  rxPacket2.payload[TYPE] = REPORT;
  rxPacket2.payload[SEQ] = expectedSeqNr;
  lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket2.payload+2);
  setTime(report->pollRx, &pollArrivalAnchorTime);
  setTime(report->answerTx, &answerDepartureAnchorTime);
  setTime(report->finalRx, &finalArrivalAnchorTime);

  // eventTimeout
  dwIdle_Expect(&dev);
  dwNewTransmit_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwSetData_ExpectWithArray(&dev, 1, (uint8_t*)&expectedTxPacket1, sizeof(packet_t), MAC802154_HEADER_LENGTH + 2);
  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);

  // eventPacketSent (POLL)
  dwGetTransmitTimestamp_ExpectAndCopyData(&dev, &pollDepartureTagTime);

  // eventPacketReceived (ANSWER)
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, &rxPacket1, dataLength);
  dwGetReceiveTimestamp_ExpectAndCopyData(&dev, &answerArrivalTagTime);
  dwNewTransmit_Expect(&dev);
  dwSetData_ExpectWithArray(&dev, 1, (uint8_t*)&expectedTxPacket2, sizeof(packet_t), MAC802154_HEADER_LENGTH + 2);
  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);

  // eventPacketSent (FINAL)
  dwGetTransmitTimestamp_ExpectAndCopyData(&dev, &finalDepartureTagTime);

  // eventPacketReceived (REPORT)
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, &rxPacket2, dataLength);

  // Test
  uint32_t actual1 = uwbTwrTagAlgorithm.onEvent(&dev, eventTimeout);
  uint32_t actual2 = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketSent);
  uint32_t actual3 = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uint32_t actual4 = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketSent);
  uint32_t actual5 = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual1);
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual2);
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual3);
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual4);
  TEST_ASSERT_EQUAL_UINT32(0, actual5);

  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDistance, options.distance[expectedAnchor]);
}

// TODO krri verify asl
// TODO krri verify rangingState and failedRanding after timeouts

///////////////////////////////////////////////////////////////////////////////


static uint8_t dwSetDataExpectedCurrSeq;
static uint8_t dwSetDataExpectedBaseAddress[] = {0, 0, 0, 0, 0, 0, 0xcf, 0xbc};

void dwSetDataMockCallback(dwDevice_t* actualDev, uint8_t* actualData, unsigned int actualN, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_INT(0, cmock_num_calls);
  TEST_ASSERT_EQUAL_PTR(&dev, actualDev);
  TEST_ASSERT_EQUAL_INT(MAC802154_HEADER_LENGTH + 2, actualN);

  packet_t *txPacket = (packet_t*)actualData;
  TEST_ASSERT_EQUAL_UINT8(POLL, txPacket->payload[TYPE]);
  TEST_ASSERT_EQUAL_UINT8(dwSetDataExpectedCurrSeq, txPacket->payload[SEQ]);
  TEST_ASSERT_EQUAL_MEMORY(expectedTagAddress, txPacket->sourceAddress, sizeof(expectedTagAddress));
  TEST_ASSERT_EQUAL_MEMORY(dwSetDataExpectedBaseAddress, txPacket->destAddress, sizeof(dwSetDataExpectedBaseAddress));
}

static void mockCallsForInitiateRanging(uint8_t expCurrSeq, uint8_t expAnchor) {
  dwIdle_Expect(&dev);
  dwNewTransmit_Expect(&dev);
  dwSetDefaults_Expect(&dev);

  dwSetData_StubWithCallback(dwSetDataMockCallback);
  dwSetDataExpectedCurrSeq = expCurrSeq;
  dwSetDataExpectedBaseAddress[0] = expAnchor;

  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);
}

// dwGetData mock /////////////////////////////////////////////////////////////

#define DW_GET_DATA_MAX_CALLS 2
static struct {
  dwDevice_t* expectedDev;
  unsigned int expectedDataLength;
  packet_t packet;
} dwGetDataContexts[DW_GET_DATA_MAX_CALLS];


static void dwGetDataMockCallback(dwDevice_t* actualDev, uint8_t* data, unsigned int actualDataLength, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_PTR(dwGetDataContexts[cmock_num_calls].expectedDev, actualDev);
  TEST_ASSERT_EQUAL_UINT(dwGetDataContexts[cmock_num_calls].expectedDataLength, actualDataLength);

  memcpy(data, &dwGetDataContexts[cmock_num_calls].packet, actualDataLength);
}

static void dwGetData_ExpectAndCopyData(dwDevice_t* expDev, const packet_t* rxPacket, unsigned int expDataLength) {
  TEST_ASSERT_TRUE(dwGetDataMockCallIndex < DW_GET_DATA_MAX_CALLS);

  dwGetDataContexts[dwGetDataMockCallIndex].expectedDev = expDev;
  dwGetDataContexts[dwGetDataMockCallIndex].expectedDataLength = expDataLength;
  memcpy(&dwGetDataContexts[dwGetDataMockCallIndex].packet, rxPacket, sizeof(packet_t));

  dwGetData_StubWithCallback(dwGetDataMockCallback);

  dwGetDataMockCallIndex++;
}


// dwGetTransmitTimestamp mock ////////////////////////////////////////////////
#define DW_GET_TRANSMIT_TIMESTAMP_MAX_CALLS 2
static struct {
  dwDevice_t* expectedDev;
  dwTime_t time;
} dwGetTransmitTimestampContexts[DW_GET_TRANSMIT_TIMESTAMP_MAX_CALLS];

static void dwGetTransmitTimestampMockCallback(dwDevice_t* actualDev, dwTime_t* time, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_PTR(dwGetTransmitTimestampContexts[cmock_num_calls].expectedDev, actualDev);
  memcpy(time, &dwGetTransmitTimestampContexts[cmock_num_calls].time, sizeof(dwTime_t));
}

static void dwGetTransmitTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time) {
  TEST_ASSERT_TRUE(dwGetTransmitTimestampMockCallIndex < DW_GET_TRANSMIT_TIMESTAMP_MAX_CALLS);

  dwGetTransmitTimestampContexts[dwGetTransmitTimestampMockCallIndex].expectedDev = expDev;
  memcpy(&dwGetTransmitTimestampContexts[dwGetTransmitTimestampMockCallIndex].time, time, sizeof(dwTime_t));

  dwGetTransmitTimestamp_StubWithCallback(dwGetTransmitTimestampMockCallback);

  dwGetTransmitTimestampMockCallIndex++;
}


// dwGetReceiveTimestamp mock /////////////////////////////////////////////////
#define DW_GET_RECEIVE_TIMESTAMP_MAX_CALLS 2
static struct {
  dwDevice_t* expectedDev;
  dwTime_t time;
} dwGetReceiveTimestampContexts[DW_GET_RECEIVE_TIMESTAMP_MAX_CALLS];

static void dwGetReceiveTimestampMockCallback(dwDevice_t* actualDev, dwTime_t* time, int cmock_num_calls) {
  TEST_ASSERT_EQUAL_PTR(dwGetReceiveTimestampContexts[cmock_num_calls].expectedDev, actualDev);
  memcpy(time, &dwGetReceiveTimestampContexts[cmock_num_calls].time, sizeof(dwTime_t));
}

static void dwGetReceiveTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time) {
  TEST_ASSERT_TRUE(dwGetReceiveTimestampMockCallIndex < DW_GET_RECEIVE_TIMESTAMP_MAX_CALLS);

  dwGetReceiveTimestampContexts[dwGetReceiveTimestampMockCallIndex].expectedDev = expDev;
  memcpy(&dwGetReceiveTimestampContexts[dwGetReceiveTimestampMockCallIndex].time, time, sizeof(dwTime_t));

  dwGetReceiveTimestamp_StubWithCallback(dwGetReceiveTimestampMockCallback);

  dwGetReceiveTimestampMockCallIndex++;
}

///////////////////////////////////////////////////////////////////////////////

static void setAddress(uint8_t* data, uint8_t addr) {
  data[0] = addr;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0xcf;
  data[7] = 0xbc;
}

static void setTime(uint8_t* data, const dwTime_t* time) {
  memcpy(data, time, sizeof(dwTime_t));
}
