// File under test lpsTwrTag.h
#include "lpsTdoa2Tag.h"

#include <string.h>
#include <stdlib.h>
#include "unity.h"

#include "mock_libdw1000.h"
#include "mock_cfassert.h"
#include "mock_estimator_kalman.h"
#include "mock_locodeck.h"

#include "dw1000Mocks.h"
#include "freertosMocks.h"
#include "physicalConstants.h"

// The local clock uses 40 bits
#define TIMER_TAG_MAX_VALUE 0x000000FFFFFFFFFFul

// Timestamps received in messages from anchors are 32 bits
#define TIMER_ANCHOR_MAX_VALUE 0x00000000FFFFFFFFul

static dwDevice_t dev;
static lpsTdoa2AlgoOptions_t options = {
  .anchorAddress = {
    0xbccf000000000000,
    0xbccf000000000001,
    0xbccf000000000002,
    0xbccf000000000003,
    0xbccf000000000004,
    0xbccf000000000005,
    0xbccf000000000006,
    0xbccf000000000007,
  },
  .anchorPosition = {
    {.x = 0.99, .y = 1.49, .z = 1.80},
    {.x = 0.99, .y = 3.29, .z = 1.80},
    {.x = 4.67, .y = 2.54, .z = 1.80},
    {.x = 0.59, .y = 2.27, .z = 0.20},
    {.x = 4.70, .y = 3.38, .z = 0.20},
    {.x = 4.70, .y = 1.14, .z = 0.20},
    {.x = 4.70, .y = 3.38, .z = 0.20},
    {.x = 4.70, .y = 1.14, .z = 0.20},
  },
};

static const uint64_t NS = 0;
static const uint32_t dataLengthNoLpp = MAC802154_HEADER_LENGTH + sizeof(rangePacket2_t);

static void mockMessageFromAnchor(uint8_t anchorIndex, uint64_t rxTime, uint8_t sequenceNrs[], uint64_t timestamps[], uint64_t distances[]);
static void mockMessageFromAnchorNotComingBackToReceive(uint8_t anchorIndex, uint64_t rxTime, uint8_t sequenceNrs[], uint64_t timestamps[], uint64_t distances[]);
static void mockMessageFromAnchorWithLppData(uint8_t anchorIndex, uint64_t rxTime, uint8_t sequenceNrs[], uint64_t timestamps[], uint64_t distances[], uint32_t lppDataSize, uint8_t* lppData);
static void mockRadioSetToReceiveMode();

static void ignoreKalmanEstimatorValidation();
static void mockKalmanEstimator(uint8_t anchor1, uint8_t anchor2, double distanceDiff);
static void mockKalmanEstimator_validate();
static void mockKalmanEstimator_resetMock();
static void populateLppPacket(packet_t* packet, char *data, int length, locoAddress_t sourceAddress, locoAddress_t destinationAddress);
static void mockSendLppShortHandling(const packet_t* expectedTxPacket, int length);

static uint64_t drift(float factor, uint64_t time);

static bool lpsGetLppShort_ignoreAndReturnFalse = true;
static bool lpsGetLppShortCallbackForLppShortPacketSent(lpsLppShortPacket_t* shortPacket, int cmock_num_calls);


// Stock test case for verifying clock offsets and clock wrap
static void verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(uint64_t tO, uint64_t a0O, uint64_t a1O);
// Stock test case for verifying clock drift
void verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(float driftTag, float driftA1);


const uint64_t time1m = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
const uint64_t time2m = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
const uint64_t time2_5m = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

const uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
const uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
const uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

static char * lppShortPacketData = "hello";
static int lppShortPacketLength = 5;
static int lppShortPacketDest = 3;
static int lppShortPacketSource = 0xff;

static int lpsGetLppShort_numberOfCall;

// End stock test case

#define SLOT_TIME 0.002
#define FRAME_TIME (SLOT_TIME * 8)
#define ANCHOR_TIME(frame, slot) ((1.0 + frame * FRAME_TIME + slot * SLOT_TIME) * LOCODECK_TS_FREQ)

const uint64_t iTxTime0_0 = ANCHOR_TIME(0, 0);
const uint64_t iTxTime0_1 = ANCHOR_TIME(0, 1);
const uint64_t iTxTime0_2 = ANCHOR_TIME(0, 2);
const uint64_t iTxTime0_3 = ANCHOR_TIME(0, 3);
const uint64_t iTxTime0_4 = ANCHOR_TIME(0, 4);
const uint64_t iTxTime0_5 = ANCHOR_TIME(0, 5);

const uint64_t iTxTime1_0 = ANCHOR_TIME(1, 0);
const uint64_t iTxTime1_1 = ANCHOR_TIME(1, 1);
const uint64_t iTxTime1_2 = ANCHOR_TIME(1, 2);
const uint64_t iTxTime1_3 = ANCHOR_TIME(1, 3);
const uint64_t iTxTime1_4 = ANCHOR_TIME(1, 4);
const uint64_t iTxTime1_5 = ANCHOR_TIME(1, 5);

const uint64_t iTxTime2_0 = ANCHOR_TIME(2, 0);
const uint64_t iTxTime2_1 = ANCHOR_TIME(2, 1);
const uint64_t iTxTime2_2 = ANCHOR_TIME(2, 2);
const uint64_t iTxTime2_3 = ANCHOR_TIME(2, 3);
const uint64_t iTxTime2_4 = ANCHOR_TIME(2, 4);
const uint64_t iTxTime2_5 = ANCHOR_TIME(2, 5);

const uint64_t iTxTime3_0 = ANCHOR_TIME(3, 0);
const uint64_t iTxTime3_1 = ANCHOR_TIME(3, 1);
const uint64_t iTxTime3_2 = ANCHOR_TIME(3, 2);
const uint64_t iTxTime3_3 = ANCHOR_TIME(3, 3);
const uint64_t iTxTime3_4 = ANCHOR_TIME(3, 4);
const uint64_t iTxTime3_5 = ANCHOR_TIME(3, 5);

void setUp(void) {
  lpsTdoa2TagSetOptions(&options);

  dwGetData_resetMock();
  dwGetReceiveTimestamp_resetMock();

  mockKalmanEstimator_resetMock();

  options.combinedAnchorPositionOk = true;

  dwSetReceiveWaitTimeout_Expect(&dev, TDOA2_RECEIVE_TIMEOUT);
  dwCommitConfiguration_Expect(&dev);

  locoDeckSetRangingState_Expect(0);
  uwbTdoa2TagAlgorithm.init(&dev);

  lpsGetLppShort_StubWithCallback(lpsGetLppShortCallbackForLppShortPacketSent);
  lpsGetLppShort_ignoreAndReturnFalse = true;

  locoDeckSetRangingState_Ignore();
}

void tearDown(void)
{
  mockKalmanEstimator_validate();
}


void testEventReceiveUnhandledEventShouldAssertFailure() {
  // Fixture
  assertFail_Expect("", "", 0);
  assertFail_IgnoreArg_exp();
  assertFail_IgnoreArg_file();
  assertFail_IgnoreArg_line();

  uwbEvent_t unknownEvent = 100;

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, unknownEvent);

  // Assert
  // Mock automatically validated after test
}

void testDifferenceOfDistanceWithNoClockDriftButOffset1() {
  // Fixture
  uint64_t tO = 3 * LOCODECK_TS_FREQ;
  uint64_t a0O = 1 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingLocalClock1() {
  // Fixture
  // Local clock start offset set to wrap local clock after first message from A0
  uint64_t tO = TIMER_TAG_MAX_VALUE - (iTxTime0_0 + timeA0ToTag) - 1;
  uint64_t a0O = 4 * LOCODECK_TS_FREQ;
  uint64_t a1O = 5 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingLocalClock2() {
  // Fixture
  // Local clock start offset set to wrap local clock after first message from A1
  uint64_t tO = TIMER_TAG_MAX_VALUE - (iTxTime0_1 + timeA1ToTag) - 1;
  uint64_t a0O = 4 * LOCODECK_TS_FREQ;
  uint64_t a1O = 5 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingLocalClock3() {
  // Fixture
  // Local clock start offset set to wrap local clock after second message from A0
  uint64_t tO = TIMER_TAG_MAX_VALUE - (iTxTime1_0 + timeA0ToTag) - 1;
  uint64_t a0O = 4 * LOCODECK_TS_FREQ;
  uint64_t a1O = 5 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingAnchor0Clock1() {
  // Fixture
  uint64_t tO = 7 * LOCODECK_TS_FREQ;
  // A0 clock offset set to wrap after first message from A0
  uint64_t a0O = TIMER_ANCHOR_MAX_VALUE - iTxTime0_0 - 1;
  uint64_t a1O = 5 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingAnchor0Clock2() {
  // Fixture
  uint64_t tO = 7 * LOCODECK_TS_FREQ;
  // A0 clock offset set to wrap after first message from A1
  uint64_t a0O = TIMER_ANCHOR_MAX_VALUE - (iTxTime0_1 + timeA0ToA1) - 1;
  uint64_t a1O = 5 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingAnchor0Clock3() {
  // Fixture
  uint64_t tO = 7 * LOCODECK_TS_FREQ;
  // A0 clock offset set to wrap after second message from A0
  uint64_t a0O = TIMER_ANCHOR_MAX_VALUE - iTxTime1_0 - 1;
  uint64_t a1O = 5 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingAnchor1Clock1() {
  // Fixture
  uint64_t tO = 7 * LOCODECK_TS_FREQ;
  uint64_t a0O = 3 * LOCODECK_TS_FREQ;
  // A1 clock offset set to wrap after first message from A0
  uint64_t a1O = TIMER_ANCHOR_MAX_VALUE - (iTxTime0_0 + timeA0ToA1) - 1;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingAnchor1Clock2() {
  // Fixture
  uint64_t tO = 7 * LOCODECK_TS_FREQ;
  uint64_t a0O = 3 * LOCODECK_TS_FREQ;
  // A1 clock offset set to wrap after first message from A1
  uint64_t a1O = TIMER_ANCHOR_MAX_VALUE - iTxTime0_1 - 1;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingAnchor1Clock3() {
  // Fixture
  uint64_t tO = 7 * LOCODECK_TS_FREQ;
  uint64_t a0O = 3 * LOCODECK_TS_FREQ;
  // A1 clock offset set to wrap after second message from A0
  uint64_t a1O = TIMER_ANCHOR_MAX_VALUE - (iTxTime1_0 + timeA0ToA1) - 1;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesNoDrift() {
  // Fixture
  float driftTag = 1.0;
  float driftA1 = 1.0;

  // Test
  verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(driftTag, driftA1);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift1() {
  // Fixture
  float driftTag = 1.000010;
  float driftA1 = 1.0;

  // Test
  verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(driftTag, driftA1);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift2() {
  // Fixture
  float driftTag = 1.0;
  float driftA1 = 0.999995;

  // Test
  verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(driftTag, driftA1);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift3() {
  // Fixture
  float driftTag = 1.000010;
  float driftA1 = 0.999995;

  // Test
  verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(driftTag, driftA1);

  // Assert
  // Nothing here, verification in mocks
}

void testMissingTimestampInhibitsClockDriftCalculationInFirstIteration() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;
  uint64_t timeA0ToA5 = time1m;

  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag,
    (uint8_t[]) {10,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){NS,                      NS, NS, NS, NS, iTxTime0_5,              NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){iTxTime1_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  21,                      0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime1_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime2_0 + timeA0ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  21,                      0, 0},
    (uint64_t[]){iTxTime2_0,              NS, NS, NS, NS, iTxTime1_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime2_5 + timeA5ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  22,                      0, 0},
    (uint64_t[]){iTxTime2_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime2_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  mockKalmanEstimator(5, 0, -expectedDiff);
  mockKalmanEstimator(0, 5, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testMissingPacketAnchorToAnchorInhibitsDiffCalculation() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;
  uint64_t timeA0ToA5 = time1m;

  uint64_t missing = 0;

  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag,
    (uint8_t[]) {10,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){NS,                      NS, NS, NS, NS, iTxTime0_5,              NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){iTxTime1_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  21,                      0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime1_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  // The timestamp is missing for A5
  mockMessageFromAnchor(0, iTxTime2_0 + timeA0ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  21,                      0, 0},
    (uint64_t[]){iTxTime2_0,              NS, NS, NS, NS, missing                , NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime2_5 + timeA5ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  22,                      0, 0},
    (uint64_t[]){iTxTime2_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime2_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  // Not called
  // mockKalmanEstimator(5, 0, -expectedDiff);
  mockKalmanEstimator(0, 5, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testMissingAnchorToAnchorDistanceInhibitsDiffCalculation() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;
  uint64_t timeA0ToA5 = time1m;

  uint64_t missing = 0;

  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag,
    (uint8_t[]) {10,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){NS,                      NS, NS, NS, NS, iTxTime0_5,              NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){iTxTime1_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  21,                      0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime1_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  // The distance is missing for A5
  mockMessageFromAnchor(0, iTxTime2_0 + timeA0ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  21,                      0, 0},
    (uint64_t[]){iTxTime2_0,              NS, NS, NS, NS, iTxTime1_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, missing,                 NS, NS});

  mockMessageFromAnchor(5, iTxTime2_5 + timeA5ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  22,                      0, 0},
    (uint64_t[]){iTxTime2_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime2_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  // Not called
  // mockKalmanEstimator(5, 0, -expectedDiff);
  mockKalmanEstimator(0, 5, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testMissingPacketPacketAnchorToAnchorInhibitsDiffCalculation() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;
  uint64_t timeA0ToA5 = time1m;

  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag,
    (uint8_t[]) {10,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){NS,                      NS, NS, NS, NS, iTxTime0_5,              NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){iTxTime1_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  21,                      0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime1_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  // The packet from A5 to A0 is missing
  // The sequence number, timestamp and distance is same as in previous packet from A0
  mockMessageFromAnchor(0, iTxTime2_0 + timeA0ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  20,                      0, 0},
    (uint64_t[]){iTxTime2_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime2_5 + timeA5ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  22,                      0, 0},
    (uint64_t[]){iTxTime2_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime2_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  // Not called
  // mockKalmanEstimator(5, 0, -expectedDiff);
  mockKalmanEstimator(0, 5, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testMissingPacketPacketAnchorToAnchorInhibitsDiffCalculationWhenSequenceNrWraps() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;
  uint64_t timeA0ToA5 = time1m;

  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag,
    (uint8_t[]) {10,                      0,  0,  0,  0,  255,                     0,  0},
    (uint64_t[]){NS,                      NS, NS, NS, NS, iTxTime0_5,              NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  255,                     0,  0},
    (uint64_t[]){iTxTime1_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  0,                       0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime1_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  // The packet from A5 to A0 is missing
  // The sequence number, timestamp and distance is same as in previous packet from A0
  mockMessageFromAnchor(0, iTxTime2_0 + timeA0ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  255,                     0, 0},
    (uint64_t[]){iTxTime2_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime2_5 + timeA5ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  1,                       0, 0},
    (uint64_t[]){iTxTime2_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime2_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  // Not called
  // mockKalmanEstimator(5, 0, -expectedDiff);
  mockKalmanEstimator(0, 5, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testMissingPacketAnchorToTagInhibitsDiffCalculation() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;
  uint64_t timeA0ToA5 = time1m;

  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag,
    (uint8_t[]) {10,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){NS,                      NS, NS, NS, NS, iTxTime0_5,              NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  20,                      0,  0},
    (uint64_t[]){iTxTime1_0,              NS, NS, NS, NS, iTxTime0_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag,
    (uint8_t[]) {11,                      0,  0,  0,  0,  21,                      0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime1_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  // Missing packet
  //  mockMessageFromAnchor(0, iTxTime2_0 + timeA0ToTag,
  //    (uint8_t[]) {12,                      0,  0,  0,  0,  21,                      0, 0},
  //    (uint64_t[]){iTxTime2_0,              NS, NS, NS, NS, iTxTime1_5 + timeA0ToA5, NS, NS},
  //    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});

  mockMessageFromAnchor(5, iTxTime2_5 + timeA5ToTag,
    (uint8_t[]) {12,                      0,  0,  0,  0,  22,                      0, 0},
    (uint64_t[]){iTxTime2_0 + timeA0ToA5, NS, NS, NS, NS, iTxTime2_5,              NS, NS},
    (uint64_t[]){timeA0ToA5,              NS, NS, NS, NS, NS,                      NS, NS});

  mockMessageFromAnchor(0, iTxTime3_0 + timeA0ToTag,
    (uint8_t[]) {13,                      0,  0,  0,  0,  22,                      0, 0},
    (uint64_t[]){iTxTime3_0,              NS, NS, NS, NS, iTxTime2_5 + timeA0ToA5, NS, NS},
    (uint64_t[]){NS,                      NS, NS, NS, NS, timeA0ToA5,              NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  // Not called due to the packet loss
  //  mockKalmanEstimator(5, 0, -expectedDiff);

  // Not called since packet from previous anchor was lost
  //  mockKalmanEstimator(0, 5, expectedDiff);

  // Not called since previous packet from same anchor was lost
  //  mockKalmanEstimator(5, 0, -expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  //  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testPacketReceivedEventShouldSetTheRadioInReceiveMode() {
  // Fixture
  // mockRadioSetToReceiveMode() called as part of mockMessageFromAnchor()
  mockMessageFromAnchor(0, NS,
    (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS});

  ignoreKalmanEstimatorValidation();

  // Test
  uint32_t actual = uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual);
}

void testEventTimeoutShouldSetTheRadioInReceiveMode() {
  // Fixture
  mockRadioSetToReceiveMode();

  // Test
  uint32_t actual = uwbTdoa2TagAlgorithm.onEvent(&dev, eventTimeout);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual);
}

void testEventReceiveTimeoutShouldSetTheRadioInReceiveMode() {
  // Fixture
  mockRadioSetToReceiveMode();

  // Test
  uint32_t actual = uwbTdoa2TagAlgorithm.onEvent(&dev, eventReceiveTimeout);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual);
}

void testThatLppShortPacketIsNotSentToWrongAnchorWhenAvailable() {
  // Fixture
  // mockRadioSetToReceiveMode() called as part of mockMessageFromAnchor()
  mockMessageFromAnchor(lppShortPacketDest+1, NS,
    (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS});
  mockMessageFromAnchor(lppShortPacketDest+1, NS,
    (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS});

  ignoreKalmanEstimatorValidation();

  packet_t expectedTxPacket;
  populateLppPacket(&expectedTxPacket, lppShortPacketData, lppShortPacketLength, 0xbccf000000000000 | lppShortPacketDest, 0xbccf000000000000 | lppShortPacketSource);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testThatLppShortPacketIsSentToGoodAnchorWhenAvailable() {
  // Fixture
  // mockRadioSetToReceiveMode() called as part of mockMessageFromAnchor()
  mockMessageFromAnchor(lppShortPacketDest, NS,
    (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS});
  mockMessageFromAnchorNotComingBackToReceive(lppShortPacketDest, NS,
    (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS});

  ignoreKalmanEstimatorValidation();

  packet_t expectedTxPacket;
  populateLppPacket(&expectedTxPacket, lppShortPacketData, lppShortPacketLength, 0xbccf000000000000 | lppShortPacketSource, 0xbccf000000000000 | lppShortPacketDest);

  mockSendLppShortHandling(&expectedTxPacket, lppShortPacketLength);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testThatLppShortPacketIsDiscardedIfAnchorNotPresentForTooLong() {
  // Fixture
  // mockRadioSetToReceiveMode() called as part of mockMessageFromAnchor()
  for (int i=0; i<TDOA2_LPP_PACKET_SEND_TIMEOUT+1; i++) {
    mockMessageFromAnchor(lppShortPacketDest+1, NS,
      (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
      (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
      (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS});
  }

  ignoreKalmanEstimatorValidation();

  packet_t expectedTxPacket;
  populateLppPacket(&expectedTxPacket, lppShortPacketData, lppShortPacketLength, 0xbccf000000000000 | lppShortPacketDest, 0xbccf000000000000 | lppShortPacketSource);

  lpsGetLppShort_numberOfCall =  0;

  // Test
  for (int i=0; i<TDOA2_LPP_PACKET_SEND_TIMEOUT+1; i++) {
    uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  }

  // Assert
  TEST_ASSERT_EQUAL_INT(2, lpsGetLppShort_numberOfCall);
}

void testDifferenceOfDistancePushedInKalmanIfAnchorsPositionIsValid() {
  // Fixture
  uint64_t tO = 3 * LOCODECK_TS_FREQ;
  uint64_t a0O = 1 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;

  options.combinedAnchorPositionOk = false;
  options.anchorPosition[0].timestamp = 1;
  options.anchorPosition[1].timestamp = 1;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceNotPushedInKalmanIfAnchorsPositionIsInValid() {
  // Fixture
  uint64_t tO = 3 * LOCODECK_TS_FREQ;
  uint64_t a0O = 1 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;

  options.combinedAnchorPositionOk = false;
  options.anchorPosition[0].timestamp = 1;
  options.anchorPosition[1].timestamp = 0;

  // test
  // tO:  offset for tag clock
  // a0O: offset for anchor 0 clock
  // a1O: offset for anchor 1 clock

  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA1ToTag = time2_5m;

  // Time between anchors in universal time. Including anchor delay.
  uint64_t timeA0ToA1 = time1m + 150.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  mockMessageFromAnchor(1, iTxTime0_1 + timeA1ToTag + tO,
    (uint8_t[]) {10,                            20,               0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS,                            iTxTime0_1 + a1O, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS,                            NS,               NS, NS, NS, NS, NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag + tO,
    (uint8_t[]) {11,                            20,               0,  0,  0,  0,  0,  0},
    (uint64_t[]){iTxTime1_0 + a0O,              NS,               NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS,                            NS,               NS, NS, NS, NS, NS, NS});

  mockMessageFromAnchor(1, iTxTime1_1 + timeA1ToTag + tO,
    (uint8_t[]) {11,                            21,               0,  0,  0,  0,  0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA1 + a1O, iTxTime1_1 + a1O, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){timeA0ToA1,                    NS,               NS, NS, NS, NS, NS, NS});

  // The measurement should not be pushed in the kalman filter

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testLppPacketIsHandled() {
  // Fixture
  float expectedX = 1.23;
  float expectedY = 2.34;
  float expectedZ = 3.45;

  struct lppShortAnchorPos_s position;
  position.x = expectedX;
  position.y = expectedY;
  position.z = expectedZ;

  uint8_t lppDataSize = sizeof(struct lppShortAnchorPos_s);
  uint8_t anchorId = 4;

  mockMessageFromAnchorWithLppData(anchorId, NS,
    (uint8_t[]) {0, 0, 0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS, NS, NS, NS, NS, NS, NS, NS},
    lppDataSize, (uint8_t*)(&position));

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expectedX, options.anchorPosition[anchorId].x);
  TEST_ASSERT_EQUAL_FLOAT(expectedY, options.anchorPosition[anchorId].y);
  TEST_ASSERT_EQUAL_FLOAT(expectedZ, options.anchorPosition[anchorId].z);
}

void testThatInitiallyNoRangingAreReportedToBeOk() {
  // Test
  // Nothing there, there has been no rangings

  // Assert
  TEST_ASSERT_FALSE(uwbTdoa2TagAlgorithm.isRangingOk());
}

void testThatWhenARangingHasHappenRangingIsReportedToBeOk() {
  // Fixture, runs a simple ranging
  uint64_t tO = 3 * LOCODECK_TS_FREQ;
  uint64_t a0O = 1 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  TEST_ASSERT_TRUE(uwbTdoa2TagAlgorithm.isRangingOk());
}

////////////////////////////////////////////

static dwTime_t ts(uint64_t time) {
  dwTime_t a = {.full = time & 0x00FFFFFFFFFFul};
  return a;
}

static void mockMessageFromAnchorWithLppData(uint8_t anchorIndex, uint64_t rxTime, uint8_t sequenceNrs[], uint64_t timestamps[], uint64_t distances[], uint32_t lppDataSize, uint8_t* lppData) {
  packet_t packet;
  MAC80215_PACKET_INIT(packet, MAC802154_TYPE_DATA);

  packet.sourceAddress = options.anchorAddress[anchorIndex];

  rangePacket2_t* payload = (rangePacket2_t*)&packet.payload;
  payload->type = 0x22;
  for (int i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
    payload->sequenceNrs[i] = sequenceNrs[i];
    payload->timestamps[i] = (uint32_t)(timestamps[i] & 0xffffffff);
    payload->distances[i] = (uint16_t)(distances[i] & 0xffff);
  }

  uint32_t lppTotalSize = 0;
  if (lppDataSize > 0) {
    packet.payload[LPS_TDOA2_LPP_HEADER] = LPP_HEADER_SHORT_PACKET;
    packet.payload[LPS_TDOA2_LPP_TYPE] = LPP_SHORT_ANCHORPOS;
    memcpy(&packet.payload[LPS_TDOA2_LPP_PAYLOAD], lppData, lppDataSize);
    lppTotalSize = lppDataSize + 2;
  }

  uint32_t totalDataLength = dataLengthNoLpp + lppTotalSize;
  dwGetDataLength_ExpectAndReturn(&dev, totalDataLength);
  dwGetData_ExpectAndCopyData(&dev, &packet, totalDataLength);

  dwTime_t rxTimeStr = ts(rxTime);
  dwGetReceiveTimestamp_ExpectAndCopyData(&dev, &rxTimeStr);

  mockRadioSetToReceiveMode();
}

static void mockMessageFromAnchor(uint8_t anchorIndex, uint64_t rxTime, uint8_t sequenceNrs[], uint64_t timestamps[], uint64_t distances[]) {
  mockMessageFromAnchorWithLppData(anchorIndex, rxTime, sequenceNrs, timestamps, distances, 0, 0);
}

static void mockMessageFromAnchorNotComingBackToReceive(uint8_t anchorIndex, uint64_t rxTime, uint8_t sequenceNrs[], uint64_t timestamps[], uint64_t distances[]) {
  packet_t packet;
  MAC80215_PACKET_INIT(packet, MAC802154_TYPE_DATA);

  packet.sourceAddress = 0xbccf000000000000 | anchorIndex;

  rangePacket2_t* payload = (rangePacket2_t*)&packet.payload;
  payload->type = 0x22;
  for (int i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
    payload->sequenceNrs[i] = sequenceNrs[i];
    payload->timestamps[i] = (uint32_t)(timestamps[i] & 0xffffffff);
    payload->distances[i] = (uint16_t)(distances[i] & 0xffff);
  }

  dwGetDataLength_ExpectAndReturn(&dev, dataLengthNoLpp);
  dwGetData_ExpectAndCopyData(&dev, &packet, dataLengthNoLpp);

  dwTime_t rxTimeStr = ts(rxTime);
  dwGetReceiveTimestamp_ExpectAndCopyData(&dev, &rxTimeStr);
}

static uint64_t drift(float factor, uint64_t time) {
  return (uint64_t)((double)time * (double)factor);
}

static void mockRadioSetToReceiveMode() {
  dwNewReceive_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwStartReceive_Expect(&dev);
}

static void ignoreKalmanEstimatorValidation() {
  estimatorKalmanEnqueueTDOA_IgnoreAndReturn(true);
}

#define STATE_ESTIMATOR_MAX_NR_OF_CALLS 10

static tdoaMeasurement_t stateEstimatorExpectations[STATE_ESTIMATOR_MAX_NR_OF_CALLS];
static int stateEstimatorIndex = 0;
static int stateEstimatorNrOfCalls = 0;

static bool estimatorKalmanEnqueueTDOAMockCallback(tdoaMeasurement_t* actual, int cmock_num_calls) {
  char message[100];
  sprintf(message, "Failed in call %i to kalmanEstimatorEnqueueTDOA()", cmock_num_calls);

  tdoaMeasurement_t* expected = &stateEstimatorExpectations[cmock_num_calls];
  // What is a reasonable accepted error here? 2 cm is needed to make the clock drift cases pass (expected: -0.500000 actual: -0.487943).
  // Rounding error based on clock resolution is around 3 mm
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.02, expected->distanceDiff, actual->distanceDiff, message);

  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[0].x, actual->anchorPosition[0].x, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[0].y, actual->anchorPosition[0].y, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[0].z, actual->anchorPosition[0].z, message);

  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[1].x, actual->anchorPosition[1].x, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[1].y, actual->anchorPosition[1].y, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[1].z, actual->anchorPosition[1].z, message);

  stateEstimatorNrOfCalls = cmock_num_calls + 1;

  return true;
}

static void mockKalmanEstimator(uint8_t anchor1, uint8_t anchor2, double distanceDiff) {
    TEST_ASSERT_TRUE(stateEstimatorIndex < STATE_ESTIMATOR_MAX_NR_OF_CALLS);

    estimatorKalmanEnqueueTDOA_StubWithCallback(estimatorKalmanEnqueueTDOAMockCallback);

    tdoaMeasurement_t* measurement = &stateEstimatorExpectations[stateEstimatorIndex];

    measurement->distanceDiff = distanceDiff;

    measurement->anchorPosition[0].x = options.anchorPosition[anchor1].x;
    measurement->anchorPosition[0].y = options.anchorPosition[anchor1].y;
    measurement->anchorPosition[0].z = options.anchorPosition[anchor1].z;

    measurement->anchorPosition[1].x = options.anchorPosition[anchor2].x;
    measurement->anchorPosition[1].y = options.anchorPosition[anchor2].y;
    measurement->anchorPosition[1].z = options.anchorPosition[anchor2].z;

    stateEstimatorIndex++;
}

static void mockKalmanEstimator_validate() {
    TEST_ASSERT_EQUAL_INT(stateEstimatorIndex, stateEstimatorNrOfCalls);
}

static void mockKalmanEstimator_resetMock() {
    stateEstimatorIndex = 0;
    stateEstimatorNrOfCalls = 0;
}




void verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(uint64_t tO, uint64_t a0O, uint64_t a1O) {
  // tO:  offset for tag clock
  // a0O: offset for anchor 0 clock
  // a1O: offset for anchor 1 clock

  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA1ToTag = time2_5m;

  // Time between anchors in universal time. Including anchor delay.
  uint64_t timeA0ToA1 = time1m + 150.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  mockMessageFromAnchor(1, iTxTime0_1 + timeA1ToTag + tO,
    (uint8_t[]) {10,                            20,               0,  0,  0,  0,  0,  0},
    (uint64_t[]){NS,                            iTxTime0_1 + a1O, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS,                            NS,               NS, NS, NS, NS, NS, NS});

  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag + tO,
    (uint8_t[]) {11,                            20,               0,  0,  0,  0,  0,  0},
    (uint64_t[]){iTxTime1_0 + a0O,              NS,               NS, NS, NS, NS, NS, NS},
    (uint64_t[]){NS,                            NS,               NS, NS, NS, NS, NS, NS});

  mockMessageFromAnchor(1, iTxTime1_1 + timeA1ToTag + tO,
    (uint8_t[]) {11,                            21,               0,  0,  0,  0,  0,  0},
    (uint64_t[]){iTxTime1_0 + timeA0ToA1 + a1O, iTxTime1_1 + a1O, NS, NS, NS, NS, NS, NS},
    (uint64_t[]){timeA0ToA1,                    NS,               NS, NS, NS, NS, NS, NS});

  // Only the last message will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 1, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}


void verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(float driftTag, float driftA5) {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A5 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToTag = time2m;
  uint64_t timeA5ToTag = time2_5m;

  // Distance between anchors including antenna delay
  uint64_t timeA0ToA5 = time1m + 150.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t a5O = 138 * LOCODECK_TS_FREQ;

  mockMessageFromAnchor(5, drift(driftTag, iTxTime0_5 + timeA5ToTag + tO),
    (uint8_t[]) {10,                                            0,  0,  0,  0,  20,                               0,  0},
    (uint64_t[]){NS,                                            NS, NS, NS, NS, drift(driftA5, iTxTime0_5 + a5O), NS, NS},
    (uint64_t[]){NS,                                            NS, NS, NS, NS, NS,                               NS, NS});

  mockMessageFromAnchor(0, drift(driftTag, iTxTime1_0 + timeA0ToTag + tO),
    (uint8_t[]) {11,                                            0,  0,  0,  0,  20,                               0,  0},
    (uint64_t[]){iTxTime1_0 + a0O,                              NS, NS, NS, NS, NS,                               NS, NS},
    (uint64_t[]){NS,                                            NS, NS, NS, NS, NS,                               NS, NS});

  mockMessageFromAnchor(5, drift(driftTag, iTxTime1_5 + timeA5ToTag + tO),
    (uint8_t[]) {11,                                            0,  0,  0,  0,  21,                               0,  0},
    (uint64_t[]){drift(driftA5, iTxTime1_0 + timeA0ToA5 + a5O), NS, NS, NS, NS, drift(driftA5, iTxTime1_5 + a5O), NS, NS},
    (uint64_t[]){timeA0ToA5,                                    NS, NS, NS, NS, NS,                               NS, NS});

  mockMessageFromAnchor(0, drift(driftTag, iTxTime2_0 + timeA0ToTag + tO),
    (uint8_t[]) {12,                                            0,  0,  0,  0,  21,                               0, 0},
    (uint64_t[]){iTxTime2_0 + a0O,                              NS, NS, NS, NS, iTxTime1_5 + timeA0ToA5 + a0O,    NS, NS},
    (uint64_t[]){NS,                                            NS, NS, NS, NS, timeA0ToA5,                       NS, NS});

  mockMessageFromAnchor(5, drift(driftTag, iTxTime2_5 + timeA5ToTag + tO),
    (uint8_t[]) {12,                                            0,  0,  0,  0,  22,                               0, 0},
    (uint64_t[]){drift(driftA5, iTxTime2_0 + timeA0ToA5 + a5O), NS, NS, NS, NS, drift(driftA5, iTxTime2_5 + a5O), NS, NS},
    (uint64_t[]){timeA0ToA5,                                    NS, NS, NS, NS, NS,                               NS, NS});


  // Only the three last messages will create calls to the estimator. The two first are discarded due to missing data.
  mockKalmanEstimator(0, 5, expectedDiff);
  mockKalmanEstimator(5, 0, -expectedDiff);
  mockKalmanEstimator(0, 5, expectedDiff);

  // Test
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoa2TagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

static void populateLppPacket(packet_t* packet, char *data, int length, locoAddress_t sourceAddress, locoAddress_t destinationAddress) {
  lpsGetLppShort_ignoreAndReturnFalse = false;

  memset(packet, 0, sizeof(packet_t));

  MAC80215_PACKET_INIT((*packet), MAC802154_TYPE_DATA);
  packet->pan = 0xbccf;
  memcpy(&packet->payload[LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX], data, length);
  packet->payload[LPS_TDOA2_TYPE_INDEX] = LPP_HEADER_SHORT_PACKET;
  packet->sourceAddress = sourceAddress;
  packet->destAddress = destinationAddress;
}

static bool lpsGetLppShortCallbackForLppShortPacketSent(lpsLppShortPacket_t* shortPacket, int cmock_num_calls) {
  lpsGetLppShort_numberOfCall++;

  if (lpsGetLppShort_ignoreAndReturnFalse) {
    return false;
  } else {
    memcpy(shortPacket->data, lppShortPacketData, lppShortPacketLength);
    shortPacket->dest = lppShortPacketDest;
    shortPacket->length = lppShortPacketLength;

    return true;
  }
}

static void mockSendLppShortHandling(const packet_t* expectedTxPacket, int length) {
  dwIdle_Expect(&dev);
  dwNewTransmit_Expect(&dev);
  dwSetDefaults_Expect(&dev);
  dwSetData_ExpectWithArray(&dev, 1, (uint8_t*)expectedTxPacket, sizeof(packet_t), MAC802154_HEADER_LENGTH + 1 + length);
  dwWaitForResponse_Expect(&dev, true);
  dwStartTransmit_Expect(&dev);
}
