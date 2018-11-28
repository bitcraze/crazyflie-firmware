// File under test lpsTwrTag.h
#include "lpsTwrTag.h"

#include <string.h>
#include "unity.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"
#include "mock_locodeck.h"
#include "mock_configblock.h"

#include "crtp_localization_serviceMocks.h"
#include "dw1000Mocks.h"
#include "physicalConstants.h"

// #include "mock_arm_math.h"

// The mocking FW can not handle the cf_math.h/arm_math.h file, it crashes while parsing it. We have to use manual mocks instead.
// Temporarily fix to make tests pass, add test code for the kalman part of rxcallback()
#include "cf_math.h"
void arm_std_f32( float32_t * pSrc, uint32_t blockSize, float32_t * pResult) { *pResult = 0.0; }
void arm_mean_f32( float32_t * pSrc, uint32_t blockSize, float32_t * pResult) { *pResult = 0.0; }

#include "mock_estimator_kalman.h"

#include "freertosMocks.h"

static dwDevice_t dev;
static lpsTwrAlgoOptions_t options;

static void setTime(uint8_t* data, const dwTime_t* time);
static void populatePacket(packet_t* packet, uint8_t seqNr, uint8_t type, locoAddress_t sourceAddress, locoAddress_t destinationAddress);
static void populateLppPacket(packet_t* packet, char *data, int length, locoAddress_t sourceAddress, locoAddress_t destinationAddress);

static void mockEventTimeoutHandling(const packet_t* expectedTxPacket);
static void mockEventPacketSendHandling(dwTime_t* departureTime);
static void mockEventPacketReceivedAnswerHandling(int dataLength, const packet_t* rxPacket, const dwTime_t* answerArrivalTagTime, const packet_t* expectedTxPacket);
static void mockEventPacketReceivedReportHandling(int dataLength, const packet_t* rxPacket);
static void mockSendLppShortHandling(const packet_t* expectedTxPacket, int datalength);

static bool lpsGetLppShortCallbackForLppShortPacketSent(lpsLppShortPacket_t* shortPacket, int cmock_num_calls);

static lpsTwrAlgoOptions_t defaultOptions = {
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
  .combinedAnchorPositionOk = false
};

static char * lppShortPacketData = "hello";
static int lppShortPacketLength = 5;
static int lppShortPacketDest = 3;

void setUp(void) {
  dwGetData_resetMock();
  dwGetTransmitTimestamp_resetMock();
  dwGetReceiveTimestamp_resetMock();

  dwSetReceiveWaitTimeout_Expect(&dev, TWR_RECEIVE_TIMEOUT);
  dwCommitConfiguration_Expect(&dev);

  memcpy(&options, &defaultOptions, sizeof(options));
  uwbTwrTagSetOptions(&options);

  locoDeckSetRangingState_Expect(0);

  uwbTwrTagAlgorithm.init(&dev);

  locoDeckGetRangingState_IgnoreAndReturn(0);
  locoDeckSetRangingState_Ignore();
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
  lpsGetLppShort_IgnoreAndReturn(false);

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

  float actualDistance = lpsTwrTagGetDistance(expectedAnchor);
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

void testThatLppShortPacketIsSentWhenAvailable() {
  // Fixture
  lpsGetLppShort_StubWithCallback(lpsGetLppShortCallbackForLppShortPacketSent);

  packet_t expectedTxPacket;
  populateLppPacket(&expectedTxPacket, lppShortPacketData, lppShortPacketLength, defaultOptions.tagAddress, defaultOptions.anchorAddress[lppShortPacketDest]);

  mockSendLppShortHandling(&expectedTxPacket, lppShortPacketLength);

  dwTime_t txTime = {.full = 0};
  mockEventPacketSendHandling(&txTime);

  // Test
  uint32_t actual1 = uwbTwrTagAlgorithm.onEvent(&dev, eventTimeout);
  uint32_t actual2 = uwbTwrTagAlgorithm.onEvent(&dev, eventPacketSent);

  // Assert
  const uint32_t expected1 = MAX_TIMEOUT;
  const uint32_t expected2 = 0;
  TEST_ASSERT_EQUAL_UINT32(expected1, actual1);
  TEST_ASSERT_EQUAL_UINT32(expected2, actual2);
}

void testThatInitiallyNoRangingAreReportedToBeOk() {
  // Test
  // Nothing there, there has been no rangings

  // Assert
  TEST_ASSERT_FALSE(uwbTwrTagAlgorithm.isRangingOk());
}

void testThatWhenARangingHasHappenRangingIsReportedToBeOk() {
  // Fixture, setup a simple ranging
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
  lpsGetLppShort_IgnoreAndReturn(false);

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
  uwbTwrTagAlgorithm.onEvent(&dev, eventTimeout);
  uwbTwrTagAlgorithm.onEvent(&dev, eventPacketSent);
  uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTwrTagAlgorithm.onEvent(&dev, eventPacketSent);
  uwbTwrTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_TRUE(uwbTwrTagAlgorithm.isRangingOk());
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

static void populateLppPacket(packet_t* packet, char *data, int length, locoAddress_t sourceAddress, locoAddress_t destinationAddress) {
  memset(packet, 0, sizeof(packet_t));

  MAC80215_PACKET_INIT((*packet), MAC802154_TYPE_DATA);
  packet->pan = 0xbccf;
  memcpy(&packet->payload[LPS_TWR_SEND_LPP_PAYLOAD], data, length);
  packet->payload[LPS_TWR_TYPE] = LPS_TWR_LPP_SHORT;
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

static void mockSendLppShortHandling(const packet_t* expectedTxPacket, int length) {
  dwIdle_Expect(&dev);
  dwNewTransmit_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwSetData_ExpectWithArray(&dev, 1, (uint8_t*)expectedTxPacket, sizeof(packet_t), MAC802154_HEADER_LENGTH + 1 + length);
  dwWaitForResponse_Expect(&dev, false);
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

static bool lpsGetLppShortCallbackForLppShortPacketSent(lpsLppShortPacket_t* shortPacket, int cmock_num_calls) {
  memcpy(shortPacket->data, lppShortPacketData, lppShortPacketLength);
  shortPacket->dest = lppShortPacketDest;
  shortPacket->length = lppShortPacketLength;

  return true;
}
