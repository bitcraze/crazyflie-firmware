// @IGNORE_IF_NOT PLATFORM_CF2
// @IGNORE_IF_NOT ESTIMATOR_TYPE_kalman

// File under test lpsTwrTag.h
#include "lpsTdoaTag.h"

#include <string.h>
#include "unity.h"

#include "mock_libdw1000.h"
#include "mock_cfassert.h"
#include "mock_estimator_kalman.h"

#include "dw1000Mocks.h"

#define TIMER_MAX_VALUE 0x00FFFFFFFFFFul

static dwDevice_t dev;
static lpsAlgoOptions_t options = {
  .anchorPosition = {
    {.x = 0.99, .y = 1.49, .z = 1.80},
    {.x = 0.99, .y = 3.29, .z = 1.80},
    {.x = 4.67, .y = 2.54, .z = 1.80},
    {.x = 0.59, .y = 2.27, .z = 0.20},
    {.x = 4.70, .y = 3.38, .z = 0.20},
    {.x = 4.70, .y = 1.14, .z = 0.20},
  },
};

static const uint64_t NS = 0;
static const int dataLength = sizeof(packet_t);

static void mockMessageFromAnchor(uint8_t anchorIndex, uint64_t rxTime, uint64_t t0, uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t t7);
static void mockRadioSetToReceiveMode();

static void ignoreKalmanEstimatorValidation();
static void mockKalmanEstimator(uint8_t anchor1, uint8_t anchor2, double distanceDiff);
static void mockKalmanEstimator_validate();
static void mockKalmanEstimator_resetMock();

static uint64_t drift(float factor, uint64_t time);


// Stock test case for verifying clock offsets and clock wrap
static void verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(uint64_t tO, uint64_t a0O, uint64_t a1O);
// Stock test case for verifying clock drift
void verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(float driftTag, float driftA1);


const uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
const uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
const uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
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


void setUp(void) {
  dwGetData_resetMock();
  dwGetReceiveTimestamp_resetMock();

  mockKalmanEstimator_resetMock();

  uwbTdoaTagAlgorithm.init(&dev, &options);
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
  uwbTdoaTagAlgorithm.onEvent(&dev, unknownEvent);

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

void testDifferenceOfDistanceWithNoClockDriftButOffsetFirstMeasurementNotDiscardedThoughInternalStateIsNotStable() {
  // Fixture
  uint64_t tO = 3 * LOCODECK_TS_FREQ;
  uint64_t a0O = 2 * LOCODECK_TS_FREQ;
  uint64_t a1O = 1 * LOCODECK_TS_FREQ;

  // test
  verifyDifferenceOfDistanceWithNoClockDriftButConfigurableClockOffset(tO, a0O, a1O);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrappingLocalClock1() {
  // Fixture
  // Local clock start offset set to wrap local clock after first message from A0
  uint64_t tO = TIMER_MAX_VALUE - (iTxTime0_0 + timeA0ToTag) - 1;
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
  uint64_t tO = TIMER_MAX_VALUE - (iTxTime0_1 + timeA1ToTag) - 1;
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
  uint64_t tO = TIMER_MAX_VALUE - (iTxTime1_0 + timeA0ToTag) - 1;
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
  uint64_t a0O = TIMER_MAX_VALUE - iTxTime0_0 - 1;
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
  uint64_t a0O = TIMER_MAX_VALUE - (iTxTime0_1 + timeA0ToA1) - 1;
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
  uint64_t a0O = TIMER_MAX_VALUE - iTxTime1_0 - 1;
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
  uint64_t a1O = TIMER_MAX_VALUE - (iTxTime0_0 + timeA0ToA1) - 1;

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
  uint64_t a1O = TIMER_MAX_VALUE - iTxTime0_1 - 1;

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
  uint64_t a1O = TIMER_MAX_VALUE - (iTxTime1_0 + timeA0ToA1) - 1;

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

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageA5toA0() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  float expectedDiff = 0.5;
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t anO = 138 * LOCODECK_TS_FREQ;

  // Clock drifts
  float aD = 1.000010;
  float tD = 0.999995;

  // Message from A5 -> A0 lost
  //                    A       Arrival Time                       A0                                           A1  A2  A3  A4  A5                           A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0_0 + timeA0ToTag + tO), iTxTime0_0 + a0O                        , NS, NS, NS, NS, NS                         , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime0_5 + timeAnToTag + tO), drift(aD, iTxTime0_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime0_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime1_0 + timeA0ToTag + tO), iTxTime1_0 + a0O                        , NS, NS, NS, NS, iTxTime0_5 + timeA0ToAn + a0O, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1_5 + timeAnToTag + tO), drift(aD, iTxTime1_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime1_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2_0 + timeA0ToTag + tO), iTxTime2_0 + a0O                        , NS, NS, NS, NS, 0                          , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime2_5 + timeAnToTag + tO), drift(aD, iTxTime2_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime2_5 + anO)  , NS, NS);

  // Only message 4 will lead to a call to the estimator
  mockKalmanEstimator(0, anchor, expectedDiff);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageA0toA5() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t anO = 138 * LOCODECK_TS_FREQ;

  // Clock drifts
  float aD = 1.000010;
  float tD = 0.999995;

  // Message from A5 -> A0 lost
  //                    A       Arrival Time                              A0                                        A1  A2  A3  A4  A5                             A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0_0 + timeA0ToTag + tO), iTxTime0_0 + a0O                        , NS, NS, NS, NS, NS                           , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime0_5 + timeAnToTag + tO), drift(aD, iTxTime0_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime0_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime1_0 + timeA0ToTag + tO), iTxTime1_0 + a0O                        , NS, NS, NS, NS, iTxTime0_5 + timeA0ToAn + a0O, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1_5 + timeAnToTag + tO), 0                                       , NS, NS, NS, NS, drift(aD, iTxTime1_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2_0 + timeA0ToTag + tO), iTxTime2_0 + a0O                        , NS, NS, NS, NS, iTxTime1_5 + timeA0ToAn + a0O, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime2_5 + timeAnToTag + tO), drift(aD, iTxTime2_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime2_5 + anO)  , NS, NS);

  // There are no calls to the estimator

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageNr3() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t anO = 138 * LOCODECK_TS_FREQ;

  // Clock drifts
  float aD = 1.000010;
  float tD = 0.999995;

  //                    A       Arrival Time                              A0                                        A1  A2  A3  A4  A5                             A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0_0 + timeA0ToTag + tO), iTxTime0_0 + a0O                        , NS, NS, NS, NS, NS                           , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime0_5 + timeAnToTag + tO), drift(aD, iTxTime0_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime0_5 + anO)  , NS, NS);
  //mockMessageFromAnchor(0     , drift(tD, iTxTime1_0 + timeA0ToTag + tO), iTxTime1_0 + a0O                        , NS, NS, NS, NS, iTxTime0_5 + timeA0ToAn + a0O, NS, NS); message lost
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1_5 + timeAnToTag + tO), drift(aD, iTxTime1_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime1_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2_0 + timeA0ToTag + tO), iTxTime2_0 + a0O                        , NS, NS, NS, NS, iTxTime1_5 + timeA0ToAn + a0O, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime2_5 + timeAnToTag + tO), drift(aD, iTxTime2_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime2_5 + anO)  , NS, NS);

  // The missing packet will lead to no calls to the estimator

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  //uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageNr4() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t anO = 138 * LOCODECK_TS_FREQ;

  // Clock drifts
  float aD = 1.000010;
  float tD = 0.999995;

  //                    A       Arrival Time                              A0                                        A1  A2  A3  A4  A5                             A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0_0 + timeA0ToTag + tO), iTxTime0_0 + a0O                        , NS, NS, NS, NS, NS                           , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime0_5 + timeAnToTag + tO), drift(aD, iTxTime0_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime0_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime1_0 + timeA0ToTag + tO), iTxTime1_0 + a0O                        , NS, NS, NS, NS, iTxTime0_5 + timeA0ToAn + a0O, NS, NS);
  //mockMessageFromAnchor(anchor, drift(tD, iTxTime1_5 + timeAnToTag + tO), drift(aD, iTxTime1_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime1_5 + anO)  , NS, NS); message lost
  mockMessageFromAnchor(0     , drift(tD, iTxTime2_0 + timeA0ToTag + tO), iTxTime2_0 + a0O                        , NS, NS, NS, NS, iTxTime1_5 + timeA0ToAn + a0O, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime2_5 + timeAnToTag + tO), drift(aD, iTxTime2_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime2_5 + anO)  , NS, NS);

  // The missing packet will lead to no calls to the estimator

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  //uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageNr5() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  float expectedDiff = 0.5;
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t anO = 138 * LOCODECK_TS_FREQ;

  // Clock drifts
  float aD = 1.000010;
  float tD = 0.999995;

  //                    A       Arrival Time                       A0                                           A1  A2  A3  A4  A5                           A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0_0 + timeA0ToTag + tO), iTxTime0_0 + a0O                        , NS, NS, NS, NS, NS                         , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime0_5 + timeAnToTag + tO), drift(aD, iTxTime0_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime0_5 + anO)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime1_0 + timeA0ToTag + tO), iTxTime1_0 + a0O                        , NS, NS, NS, NS, iTxTime0_5 + timeA0ToAn + a0O, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1_5 + timeAnToTag + tO), drift(aD, iTxTime1_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime1_5 + anO)  , NS, NS);
  //mockMessageFromAnchor(0     , drift(tD, iTxTime2_0 + timeA0ToTag + tO), iTxTime2_0 + a0O                        , NS, NS, NS, NS, iTxTime1_5 + timeA0ToAn + a0O, NS, NS); message lost
  mockMessageFromAnchor(anchor, drift(tD, iTxTime2_5 + timeAnToTag + tO), drift(aD, iTxTime2_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(aD, iTxTime2_5 + anO)  , NS, NS);

  // Only message 4 will lead to a call to the estimator
  mockKalmanEstimator(0, anchor, expectedDiff);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  //uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}


void testNotUsingAnchor0() {
  // Fixture
  // Two anchors (A2 and A5), separated by 1.0m
  // Distance from A2 to tag is 2.0m
  // Distance from A5 to tag is 2.5m

  // Ideal times in universal clock (A0)
  uint64_t timeA2ToA5 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA2ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA5ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  // Intentionally less than 40 bit clock wrap around, approx 17s
  uint64_t a2O = 1 * LOCODECK_TS_FREQ;
  uint64_t a5O = 2 * LOCODECK_TS_FREQ;
  uint64_t tO = 3 * LOCODECK_TS_FREQ;

  //                    A  Arrival Time                   A0  A1  A2                             A3  A4  A5                              A6  A7
  mockMessageFromAnchor(2, iTxTime0_2 + timeA2ToTag + tO, NS, NS, iTxTime0_2 + a2O             , NS, NS, NS                            , NS, NS);
  mockMessageFromAnchor(5, iTxTime0_5 + timeA5ToTag + tO, NS, NS, iTxTime0_2 + timeA2ToA5 + a5O, NS, NS, iTxTime0_5 + a5O              , NS, NS);
  mockMessageFromAnchor(2, iTxTime1_2 + timeA2ToTag + tO, NS, NS, iTxTime1_2 + a2O             , NS, NS, iTxTime0_5 + timeA2ToA5 + a2O , NS, NS);
  mockMessageFromAnchor(5, iTxTime1_5 + timeA5ToTag + tO, NS, NS, iTxTime1_2 + timeA2ToA5 + a5O, NS, NS, iTxTime1_5 + a5O              , NS, NS);

  //                  A1 A2 Exp diff   Exp measurement diff
  // mockKalmanEstimator(2, 5, 2.5 - 2.0);  Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(2, 5, 2.5 - 2.0);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}


void testPacketReceivedEventShouldSetTheRadioInReceiveMode() {
  // Fixture
  // mockRadioSetToReceiveMode() called as part of mockMessageFromAnchor()
  mockMessageFromAnchor(0, NS, NS, NS, NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uint32_t actual = uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual);
}


void testEventTimeoutShouldSetTheRadioInReceiveMode() {
  // Fixture
  mockRadioSetToReceiveMode();

  // Test
  uint32_t actual = uwbTdoaTagAlgorithm.onEvent(&dev, eventTimeout);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual);
}

void testEventReceiveTimeoutShouldSetTheRadioInReceiveMode() {
  // Fixture
  mockRadioSetToReceiveMode();

  // Test
  uint32_t actual = uwbTdoaTagAlgorithm.onEvent(&dev, eventReceiveTimeout);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(MAX_TIMEOUT, actual);
}

////////////////////////////////////////////

static dwTime_t ts(uint64_t time) {
  dwTime_t a = {.full = time & 0x00FFFFFFFFFFul};
  return a;
}

static void setTimestampInPayload(uint64_t time, uint8_t* dest) {
  dwTime_t timestamp = {.full = time};
  memcpy(dest, timestamp.raw, 5);
}

static void mockMessageFromAnchor(uint8_t anchorIndex, uint64_t rxTime, uint64_t t0, uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t t7) {
  packet_t packet;
  MAC80215_PACKET_INIT(packet, MAC802154_TYPE_DATA);

  packet.sourceAddress = 0xbccf000000000000 | anchorIndex;

  rangePacket_t* payload = (rangePacket_t*)&packet.payload;
  payload->type = 0x21;
  setTimestampInPayload(t0, payload->timestamps[0]);
  setTimestampInPayload(t1, payload->timestamps[1]);
  setTimestampInPayload(t2, payload->timestamps[2]);
  setTimestampInPayload(t3, payload->timestamps[3]);
  setTimestampInPayload(t4, payload->timestamps[4]);
  setTimestampInPayload(t5, payload->timestamps[5]);
  setTimestampInPayload(t6, payload->timestamps[6]);
  setTimestampInPayload(t7, payload->timestamps[7]);

  dwGetDataLength_ExpectAndReturn(&dev, dataLength);
  dwGetData_ExpectAndCopyData(&dev, &packet, dataLength);

  dwTime_t rxTimeStr = ts(rxTime);
  dwGetReceiveTimestamp_ExpectAndCopyData(&dev, &rxTimeStr);

  mockRadioSetToReceiveMode();
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
  stateEstimatorEnqueueTDOA_IgnoreAndReturn(true);
}

#define STATE_ESTIMATOR_MAX_NR_OF_CALLS 10

static tdoaMeasurement_t stateEstimatorExpectations[STATE_ESTIMATOR_MAX_NR_OF_CALLS];
static int stateEstimatorIndex = 0;
static int stateEstimatorNrOfCalls = 0;

static bool stateEstimatorEnqueueTDOAMockCallback(tdoaMeasurement_t* actual, int cmock_num_calls) {
  char message[100];
  sprintf(message, "Failed in call %i to stateEstimatorEnqueueTDOA()", cmock_num_calls);

  tdoaMeasurement_t* expected = &stateEstimatorExpectations[cmock_num_calls];
  // TODO krri What is a reasonable accepted error here? 2 cm is needed to make the clock drift cases pass (expected: -0.500000 actual: -0.487943).
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

    stateEstimatorEnqueueTDOA_StubWithCallback(stateEstimatorEnqueueTDOAMockCallback);

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
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m
  float expectedDiff = 0.5;

  //                    A  Arrival Time                   A0                             A1                              A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0_0 + timeA0ToTag + tO, iTxTime0_0 + a0O             , NS                            , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime0_1 + timeA1ToTag + tO, iTxTime0_0 + timeA0ToA1 + a1O, iTxTime0_1 + a1O              , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime1_0 + timeA0ToTag + tO, iTxTime1_0 + a0O             , iTxTime0_1 + timeA0ToA1 + a0O , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1_1 + timeA1ToTag + tO, iTxTime1_0 + timeA0ToA1 + a1O, iTxTime1_1 + a1O              , NS, NS, NS, NS, NS, NS);

  //                  A1 A2 Exp diff      Exp measurement diff
  // mockKalmanEstimator(0, 1, 2.5 - 2.0); Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(0, 1, expectedDiff);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}


void verifyDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift(float driftTag, float driftA1) {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  float expectedDiff = 0.5;
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  // Clock start offset including any antenna delay
  uint64_t tO = 17 * LOCODECK_TS_FREQ;
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t anO = 138 * LOCODECK_TS_FREQ;

  //                    A       Arrival Time                                    A0                                             A1  A2  A3  A4  A5                                A6  A7
  mockMessageFromAnchor(0     , drift(driftTag, iTxTime0_0 + timeA0ToTag + tO), iTxTime0_0 + a0O                             , NS, NS, NS, NS, NS                              , NS, NS);
  mockMessageFromAnchor(anchor, drift(driftTag, iTxTime0_5 + timeAnToTag + tO), drift(driftA1, iTxTime0_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(driftA1, iTxTime0_5 + anO), NS, NS);
  mockMessageFromAnchor(0     , drift(driftTag, iTxTime1_0 + timeA0ToTag + tO), iTxTime1_0 + a0O                             , NS, NS, NS, NS, iTxTime0_5 + timeA0ToAn + a0O   , NS, NS);
  mockMessageFromAnchor(anchor, drift(driftTag, iTxTime1_5 + timeAnToTag + tO), drift(driftA1, iTxTime1_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(driftA1, iTxTime1_5 + anO), NS, NS);
  mockMessageFromAnchor(0     , drift(driftTag, iTxTime2_0 + timeA0ToTag + tO), iTxTime2_0 + a0O                             , NS, NS, NS, NS, iTxTime1_5 + timeA0ToAn + a0O   , NS, NS);
  mockMessageFromAnchor(anchor, drift(driftTag, iTxTime2_5 + timeAnToTag + tO), drift(driftA1, iTxTime2_0 + timeA0ToAn + anO), NS, NS, NS, NS, drift(driftA1, iTxTime2_5 + anO), NS, NS);

  // Only the three last messages will create calls to the estimator. The three first are discarded due to bad data.
  mockKalmanEstimator(0, anchor, expectedDiff);
  mockKalmanEstimator(anchor, 0, -expectedDiff);
  mockKalmanEstimator(0, anchor, expectedDiff);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}