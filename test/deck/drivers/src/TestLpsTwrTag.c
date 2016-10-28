// File under test lpsTwrTag.h
#include "lpsTwrTag.h"

#include <string.h>
#include "unity.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"


static dwDevice_t dev;
static lpsAlgoOptions_t options;

static void dwGetData_ExpectAndCopyData(dwDevice_t* expDev, const packet_t* rxPacket, unsigned int expDataLength);
static void dwGetTransmitTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time);
static void dwGetReceiveTimestamp_ExpectAndCopyData(dwDevice_t* expDev, const dwTime_t* time);

static void setTime(uint8_t* data, const dwTime_t* time);
static void populatePacket(packet_t* packet, uint8_t seqNr, uint8_t type, locoAddress_t sourceAddress, locoAddress_t destinationAddress);

static void mockEventTimeoutHandling(const packet_t* expectedTxPacket);
static void mockEventPacketSendHandling(dwTime_t* departureTime);
static void mockEventPacketReceivedAnswerHandling(int dataLength, const packet_t* rxPacket, const dwTime_t* answerArrivalTagTime, const packet_t* expectedTxPacket);
static void mockEventPacketReceivedReportHandling(int dataLength, const packet_t* rxPacket);

static lpsAlgoOptions_t defaultOptions = {
  .tagAddress = 0xbccf000000000008,
  .anchorAddress = {
    0xbccf000000000001,
    0xbccf000000000002,
    0xbccf000000000003,
    0xbccf000000000004,
    0xbccf000000000005,
    0xbccf000000000006
  },
  .antennaDelay = 30000,
  .rangingFailedThreshold = 6,
  .anchorPositionOk = false
};

static int dwGetDataMockCallIndex = 0;
static int dwGetTransmitTimestampMockCallIndex = 0;
static int dwGetReceiveTimestampMockCallIndex = 0;


void setUp(void) {
  dwGetDataMockCallIndex = 0;
  dwGetTransmitTimestampMockCallIndex = 0;
  dwGetReceiveTimestampMockCallIndex = 0;

  memcpy(&options, &defaultOptions, sizeof(options));
  uwbTwrTagAlgorithm.init(&dev, &options);
}

void testNormalMessageSequenceShouldGenerateDistance() {
  // Fixture
  const int dataLength = sizeof(packet_t);
  const uint8_t expectedSeqNr = 1;
  const uint8_t expectedAnchor = 1;

  float expectedDistance = 5.0;
  const uint32_t distInTicks = expectedDistance * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  dwTime_t pollDepartureTagTime = {.full = 123456};
  dwTime_t pollArrivalAnchorTime = {.full = pollDepartureTagTime.full + distInTicks + defaultOptions.antennaDelay / 2};
  dwTime_t answerDepartureAnchorTime = {.full = pollArrivalAnchorTime.full + 100000};
  dwTime_t answerArrivalTagTime = {.full = answerDepartureAnchorTime.full + distInTicks + defaultOptions.antennaDelay / 2};
  dwTime_t finalDepartureTagTime = {.full = answerArrivalTagTime.full + 200000};
  dwTime_t finalArrivalAnchorTime = {.full = finalDepartureTagTime.full + distInTicks + defaultOptions.antennaDelay / 2};

  // eventTimeout
  packet_t expectedTxPacket1;
  populatePacket(&expectedTxPacket1, expectedSeqNr, LPS_TWR_POLL, defaultOptions.tagAddress, defaultOptions.anchorAddress[expectedAnchor]);
  mockEventTimeoutHandling(&expectedTxPacket1);

  // eventPacketSent (POLL)
  mockEventPacketSendHandling(&pollDepartureTagTime);

  // eventPacketReceived (ANSWER)
  packet_t rxPacket1;
  populatePacket(&rxPacket1, expectedSeqNr, LPS_TWR_ANSWER, defaultOptions.anchorAddress[expectedAnchor], defaultOptions.tagAddress);
  packet_t expectedTxPacket2;
  populatePacket(&expectedTxPacket2, expectedSeqNr, LPS_TWR_FINAL, defaultOptions.tagAddress, defaultOptions.anchorAddress[expectedAnchor]);
  mockEventPacketReceivedAnswerHandling(dataLength, &rxPacket1, &answerArrivalTagTime, &expectedTxPacket2);

  // eventPacketSent (FINAL)
  mockEventPacketSendHandling(&finalDepartureTagTime);

  // eventPacketReceived (REPORT)
  packet_t rxPacket2;
  populatePacket(&rxPacket2, expectedSeqNr, LPS_TWR_REPORT, defaultOptions.anchorAddress[expectedAnchor], defaultOptions.tagAddress);
  lpsTwrTagReportPayload_t *report = (lpsTwrTagReportPayload_t *)(rxPacket2.payload + 2);
  setTime(report->pollRx, &pollArrivalAnchorTime);
  setTime(report->answerTx, &answerDepartureAnchorTime);
  setTime(report->finalRx, &finalArrivalAnchorTime);
  mockEventPacketReceivedReportHandling(dataLength, &rxPacket2);

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

  float actualDistance = options.distance[expectedAnchor];
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDistance, actualDistance);
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

void testEventReceiveFailedShouldBeIgnored() {
  // Fixture

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventReceiveFailed);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventReceiveTimeoutShouldBeIgnored() {
  // Fixture

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventReceiveTimeout);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventPacketReceivedWithZeroDataLengthShouldBeIgnored() {
  // Fixture
  dwGetDataLength_ExpectAndReturn(&dev, 0);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testEventPacketReceivedWithWrongDestinationAddressShouldPrepareForReceptionOfNewPacket() {
  // Fixture
  packet_t rxPacket = {.destAddress = 0x4711471147114711};
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
  rxPacket.destAddress = defaultOptions.tagAddress;
  rxPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;

  uint8_t wrongSeqNr = 17; // After init curr_seq = 0
  rxPacket.payload[LPS_TWR_SEQ] = wrongSeqNr;

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
  rxPacket.destAddress = defaultOptions.tagAddress;
  rxPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;

  uint8_t wrongSeqNr = 17; // After init curr_seq = 0
  rxPacket.payload[LPS_TWR_SEQ] = wrongSeqNr;

  const int dataLength = sizeof(rxPacket);
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, &rxPacket, dataLength);

  // Test
  uint32_t actual = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  const uint32_t expected = 0;
  TEST_ASSERT_EQUAL_UINT32(expected, actual);
}


// TODO krri verify seqNr are increased
// TODO krri verify we use all anchors
// TODO krri verify asl
// TODO krri verify rangingState and failedRanding after timeouts


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

static void setTime(uint8_t* data, const dwTime_t* time) {
  memcpy(data, time, sizeof(dwTime_t));
}

static void populatePacket(packet_t* packet, uint8_t seqNr, uint8_t type, locoAddress_t sourceAddress, locoAddress_t destinationAddress) {
  memset(packet, 0, sizeof(packet_t));

  MAC80215_PACKET_INIT((*packet), MAC802154_TYPE_DATA);
  packet->pan = 0xbccf;
  packet->payload[LPS_TWR_SEQ] = seqNr;
  packet->payload[LPS_TWR_TYPE] = type;
  packet->sourceAddress = sourceAddress;
  packet->destAddress = destinationAddress;
}

static void mockEventTimeoutHandling(const packet_t* expectedTxPacket) {
  dwIdle_Expect(&dev);
  dwNewTransmit_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwSetData_ExpectWithArray(&dev, 1, (uint8_t*)expectedTxPacket, sizeof(packet_t), MAC802154_HEADER_LENGTH + 2);
  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);
}

static void mockEventPacketSendHandling(dwTime_t* departureTime) {
  dwGetTransmitTimestamp_ExpectAndCopyData(&dev, departureTime);
}

static void mockEventPacketReceivedAnswerHandling(int dataLength, const packet_t* rxPacket, const dwTime_t* answerArrivalTagTime, const packet_t* expectedTxPacket) {
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, rxPacket, dataLength);
  dwGetReceiveTimestamp_ExpectAndCopyData(&dev, answerArrivalTagTime);
  dwNewTransmit_Expect(&dev);
  dwSetData_ExpectWithArray(&dev, 1, (uint8_t*)expectedTxPacket, sizeof(packet_t), MAC802154_HEADER_LENGTH + 2);
  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);
}

static void mockEventPacketReceivedReportHandling(int dataLength, const packet_t* rxPacket) {
  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, rxPacket, dataLength);
}
