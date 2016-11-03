// File under test lpsTwrTag.h
#include "lpsTdoaTag.h"

#include <string.h>
#include "unity.h"
#include "mock_libdw1000.h"
#include "mock_cfassert.h"
#include "dw1000Mocks.h"

static dwDevice_t dev;
static lpsAlgoOptions_t options;

static const uint64_t NS = 0;
static const int dataLength = sizeof(packet_t);

static void mockMessageFromAnchor(int anchorIndex, uint64_t rxTime, uint64_t t0, uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t t7);
static uint64_t drift(float factor, uint64_t time);

void setUp(void) {
  dwGetData_resetMock();
  dwGetReceiveTimestamp_resetMock();

  uwbTdoaTagAlgorithm.init(&dev, &options);
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


void testDifferenceOfDistanceWithNoClockDrift() {
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m
  float expectedDiff = 0.5;

  // Ideal times in universal clock
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t a1O = 138 * LOCODECK_TS_FREQ;
  uint64_t tO = 17 * LOCODECK_TS_FREQ;

  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA0ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, uwbTdoaDistDiff[1]);
}


void testDifferenceOfDistanceWithTwoAnchors3FramesNoDrift() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  float expectedDiff = 0.5;
  const int anchor = 2;

  // Ideal times in universal clock
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;
  uint64_t iTxTime4 = 1.008 * LOCODECK_TS_FREQ;
  uint64_t iTxTime5 = 1.010 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay
  uint64_t a0O = 60 * LOCODECK_TS_FREQ;
  uint64_t a1O = 138 * LOCODECK_TS_FREQ;
  uint64_t tO = 17 * LOCODECK_TS_FREQ;

  mockMessageFromAnchor(0     , iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS, NS                         , NS, NS, NS, NS, NS);
  mockMessageFromAnchor(anchor, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, NS, iTxTime1 + a1O             , NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0     , iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , NS, iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(anchor, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, NS, iTxTime3 + a1O             , NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0     , iTxTime4 + timeA0ToTag + tO, iTxTime4 + a0O             , NS, iTxTime3 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(anchor, iTxTime5 + timeA1ToTag + tO, iTxTime4 + timeA0ToA1 + a1O, NS, iTxTime5 + a1O             , NS, NS, NS, NS, NS);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual1 = uwbTdoaDistDiff[anchor];
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual2 = uwbTdoaDistDiff[anchor];

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual1);
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual2);
}


void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDrift() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
  float expectedDiff = 0.5;
  const int anchor = 5;

  // Ideal times in universal clock
  uint64_t timeA0ToAn = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeAnToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;
  uint64_t iTxTime4 = 1.008 * LOCODECK_TS_FREQ;
  uint64_t iTxTime5 = 1.010 * LOCODECK_TS_FREQ;

  // Clock drifts
  float aD = 1.000010;
  float tD = 0.999995;

  mockMessageFromAnchor(0     , drift(tD, iTxTime0 + timeA0ToTag), iTxTime0                        , NS, NS, NS, NS, NS                   , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1 + timeAnToTag), drift(aD, iTxTime0 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime1)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2 + timeA0ToTag), iTxTime2                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime3 + timeAnToTag), drift(aD, iTxTime2 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime3)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime4 + timeA0ToTag), iTxTime4                        , NS, NS, NS, NS, iTxTime3 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime5 + timeAnToTag), drift(aD, iTxTime4 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime5)  , NS, NS);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual1 = uwbTdoaDistDiff[anchor];
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual2 = uwbTdoaDistDiff[anchor];

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual1);
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual2);
}


////////////////////////////////////////////

static dwTime_t ts(uint64_t time) {
  dwTime_t a = {.full = time};
  return a;
}

static void setTimestampInPayload(uint64_t time, uint8_t* dest) {
  dwTime_t timestamp = {.full = time};
  memcpy(dest, timestamp.raw, 5);
}

static void mockMessageFromAnchor(int anchorIndex, uint64_t rxTime, uint64_t t0, uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4, uint64_t t5, uint64_t t6, uint64_t t7) {
  packet_t packet;
  MAC80215_PACKET_INIT(packet, MAC802154_TYPE_DATA);

  packet.sourceAddress = anchorIndex;

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
}

static uint64_t drift(float factor, uint64_t time) {
  return (uint64_t)((double)time * (double)factor);
}