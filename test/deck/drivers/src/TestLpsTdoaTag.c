// @IGNORE_IF_NOT PLATFORM_CF2

// File under test lpsTwrTag.h
#include "lpsTdoaTag.h"

#include <string.h>
#include "unity.h"

#include "mock_libdw1000.h"
#include "mock_cfassert.h"
#ifdef ESTIMATOR_TYPE_kalman
#include "mock_estimator_kalman.h"
#endif // ESTIMATOR_TYPE_kalman

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
static void mockKalmanEstimator(uint8_t anchor, double distanceDiff, double timeBetweenMeasurements);
static void mockKalmanEstimator_resetMock();

static uint64_t drift(float factor, uint64_t time);

void setUp(void) {
  dwGetData_resetMock();
  dwGetReceiveTimestamp_resetMock();

  #ifdef ESTIMATOR_TYPE_kalman
  mockKalmanEstimator_resetMock();
  #endif

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

  //                    A  Arrival Time                 A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, uwbTdoaDistDiff[1]);
}


void testKalmanEstimatorWithNoClockDrift() {
  #ifndef ESTIMATOR_TYPE_kalman
  TEST_IGNORE();
  #endif
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  // Ideal times in universal clock (A0)
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay
  // Intentionally less than 40 bit clock wrap around, approx 17s
  uint64_t a0O = 1 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;
  uint64_t tO = 3 * LOCODECK_TS_FREQ;

  //                    A  Arrival Time                 A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  //                  A  Arrival Time in tag clock    Tx time in A0 clock
  // mockKalmanEstimator(1, 2.5 - 2.0, , ((iTxTime1 + timeA1ToTag + tO) - (iTxTime0 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ);  Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime3 + timeA1ToTag + tO) - (iTxTime2 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}


void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrapping() {
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

  // Local clock start offset
  uint64_t localOffset = TIMER_MAX_VALUE - (iTxTime1 + timeA0ToTag + tO) - 1;

  //                    A  Arrival Time                               A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, localOffset + iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, localOffset + iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, localOffset + iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, localOffset + iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, uwbTdoaDistDiff[1]);
}

void testKalmanEstimatorWithNoClockDriftWithTagClockWrapping() {
  #ifndef ESTIMATOR_TYPE_kalman
  TEST_IGNORE();
  #endif
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  // Ideal times in universal clock
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay.
  // Intentionally less than 40 bit clock wrap around, approx 17s
  uint64_t a0O = 3 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;
  // Local clock start offset set to wrap after first message from A0
  uint64_t tO = TIMER_MAX_VALUE - (iTxTime0 + timeA0ToTag) - 1;

  //                    A  Arrival Time                 A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  //                  A  Arrival Time in tag clock    Tx time in A0 clock
  // mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime1 + timeA1ToTag + tO) - (iTxTime0 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ); Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime3 + timeA1ToTag + tO) - (iTxTime2 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testKalmanEstimatorWithNoClockDriftWithTagClockWrapping2() {
  #ifndef ESTIMATOR_TYPE_kalman
  TEST_IGNORE();
  #endif
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  // Ideal times in universal clock
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay.
  // Intentionally less than 40 bit clock wrap around, approx 17s
  uint64_t a0O = 3 * LOCODECK_TS_FREQ;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;
  // Local clock start offset set to wrap after second message from A0
  uint64_t tO = TIMER_MAX_VALUE - (iTxTime2 + timeA0ToTag) - 1;

  //                    A  Arrival Time                 A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  //                  A  Arrival Time in tag clock    Tx time in A0 clock
  // mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime1 + timeA1ToTag + tO) - (iTxTime0 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ); Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime3 + timeA1ToTag + tO) - (iTxTime2 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testKalmanEstimatorWithNoClockDriftWithA0ClockWrapping() {
  #ifndef ESTIMATOR_TYPE_kalman
  TEST_IGNORE();
  #endif
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  // Ideal times in universal clock
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay.
  // Intentionally less than 40 bit clock wrap around, approx 17s
  // A0 clock start offset set to wrap after first message from A0
  uint64_t a0O = TIMER_MAX_VALUE - iTxTime0 - 1;
  uint64_t a1O = 2 * LOCODECK_TS_FREQ;
  uint64_t tO = 3 * LOCODECK_TS_FREQ;

  //                    A  Arrival Time                 A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  //                  A  Arrival Time in tag clock    Tx time in A0 clock
  // mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime1 + timeA1ToTag + tO) - (iTxTime0 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ); Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime3 + timeA1ToTag + tO) - (iTxTime2 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testKalmanEstimatorWithNoClockDriftWithA1ClockWrapping() {
  #ifndef ESTIMATOR_TYPE_kalman
  TEST_IGNORE();
  #endif
  // Fixture
  // Two anchors (A0 and A1), separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from A1 to tag is 2.5m

  // Ideal times in universal clock
  uint64_t timeA0ToA1 = 1.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA0ToTag = 2.0 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;
  uint64_t timeA1ToTag = 2.5 * LOCODECK_TS_FREQ / SPEED_OF_LIGHT;

  uint64_t iTxTime0 = 1.0 * LOCODECK_TS_FREQ;
  uint64_t iTxTime1 = 1.002 * LOCODECK_TS_FREQ;
  uint64_t iTxTime2 = 1.004 * LOCODECK_TS_FREQ;
  uint64_t iTxTime3 = 1.006 * LOCODECK_TS_FREQ;

  // Clock start offset including any antenna delay.
  // Intentionally less than 40 bit clock wrap around, approx 17s
  // A0 clock start offset set to wrap after first message from A0
  uint64_t a0O = 1 * LOCODECK_TS_FREQ;
  uint64_t a1O = TIMER_MAX_VALUE - iTxTime0 + timeA0ToA1 - 1;
  uint64_t tO = 3 * LOCODECK_TS_FREQ;

  //                    A  Arrival Time                 A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  //                  A  Arrival Time in tag clock    Tx time in A0 clock
  // mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime1 + timeA1ToTag + tO) - (iTxTime0 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ); Not called as the packet is interpreted as packet loss and filtered out
  mockKalmanEstimator(1, 2.5 - 2.0, ((iTxTime3 + timeA1ToTag + tO) - (iTxTime2 + timeA0ToTag + tO)) / LOCODECK_TS_FREQ);

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  // Nothing here, verification in mocks
}

void testDifferenceOfDistanceWithNoClockDriftWithTagClockWrapping2() {
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

  // Local clock start offset
  uint64_t localOffset = TIMER_MAX_VALUE - (iTxTime2 + timeA0ToTag + tO) - 1;

  //                    A  Arrival Time                               A0                           A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, localOffset + iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, localOffset + iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O             , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, localOffset + iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, localOffset + iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O             , NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, uwbTdoaDistDiff[1]);
}

void testDifferenceOfDistanceWithNoClockDriftWithA0ClockWrapping() {
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

  // Local clock start offset
  uint64_t a0Offset = TIMER_MAX_VALUE - (iTxTime1 + timeA0ToA1 + a0O) - 1;

  //                    A  Arrival Time                 A0                           A1                                       A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, a0Offset + iTxTime0 + a0O  , NS                                     , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, iTxTime1 + a1O                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, a0Offset + iTxTime2 + a0O  , a0Offset + iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, iTxTime3 + a1O                         , NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, uwbTdoaDistDiff[1]);
}

void testDifferenceOfDistanceWithNoClockDriftWithA1ClockWrapping() {
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

  // Local clock start offset
  uint64_t a1Offset = TIMER_MAX_VALUE - (iTxTime0 + timeA0ToA1 + a1O) - 1;

  //                    A  Arrival Time                 A0                                      A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O                        , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, a1Offset + iTxTime0 + timeA0ToA1 + a1O, a1Offset + iTxTime1 + a1O  , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O                        , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, a1Offset + iTxTime2 + timeA0ToA1 + a1O, a1Offset + iTxTime3 + a1O  , NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, uwbTdoaDistDiff[1]);
}

void testDifferenceOfDistanceWithNoClockDriftWithA1ClockWrapping2() {
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

  // Local clock start offset
  uint64_t a1Offset = TIMER_MAX_VALUE - (iTxTime2 + timeA0ToA1 + a1O) - 1;

  //                    A  Arrival Time                 A0                                      A1                           A2  A3  A4  A5  A6  A7
  mockMessageFromAnchor(0, iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O                        , NS                         , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime1 + timeA1ToTag + tO, a1Offset + iTxTime0 + timeA0ToA1 + a1O, a1Offset + iTxTime1 + a1O  , NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0, iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O                        , iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(1, iTxTime3 + timeA1ToTag + tO, a1Offset + iTxTime2 + timeA0ToA1 + a1O, a1Offset + iTxTime3 + a1O  , NS, NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

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

  //                    A       Arrival Time                 A0                           A1  A2                           A3  A4  A5  A6  A7
  mockMessageFromAnchor(0     , iTxTime0 + timeA0ToTag + tO, iTxTime0 + a0O             , NS, NS                         , NS, NS, NS, NS, NS);
  mockMessageFromAnchor(anchor, iTxTime1 + timeA1ToTag + tO, iTxTime0 + timeA0ToA1 + a1O, NS, iTxTime1 + a1O             , NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0     , iTxTime2 + timeA0ToTag + tO, iTxTime2 + a0O             , NS, iTxTime1 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(anchor, iTxTime3 + timeA1ToTag + tO, iTxTime2 + timeA0ToA1 + a1O, NS, iTxTime3 + a1O             , NS, NS, NS, NS, NS);
  mockMessageFromAnchor(0     , iTxTime4 + timeA0ToTag + tO, iTxTime4 + a0O             , NS, iTxTime3 + timeA0ToA1 + a0O, NS, NS, NS, NS, NS);
  mockMessageFromAnchor(anchor, iTxTime5 + timeA1ToTag + tO, iTxTime4 + timeA0ToA1 + a1O, NS, iTxTime5 + a1O             , NS, NS, NS, NS, NS);

  ignoreKalmanEstimatorValidation();

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

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageA5toA0() {
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

  // Message from A5 -> A0 lost
  //                    A       Arrival Time                       A0                                A1  A2  A3  A4  A5                     A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0 + timeA0ToTag), iTxTime0                        , NS, NS, NS, NS, NS                   , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1 + timeAnToTag), drift(aD, iTxTime0 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime1)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2 + timeA0ToTag), iTxTime2                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime3 + timeAnToTag), drift(aD, iTxTime2 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime3)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime4 + timeA0ToTag), iTxTime4                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime5 + timeAnToTag), drift(aD, iTxTime4 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime5)  , NS, NS);

  ignoreKalmanEstimatorValidation();

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

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageA0toT() {
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

  // Message from A5 -> A0 lost
  //                    A       Arrival Time                       A0                                A1  A2  A3  A4  A5                     A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0 + timeA0ToTag), iTxTime0                        , NS, NS, NS, NS, NS                   , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1 + timeAnToTag), drift(aD, iTxTime0 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime1)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2 + timeA0ToTag), iTxTime2                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime3 + timeAnToTag), drift(aD, iTxTime2 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime3)  , NS, NS);
//  mockMessageFromAnchor(0     , drift(tD, iTxTime4 + timeA0ToTag), iTxTime4                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime5 + timeAnToTag), drift(aD, iTxTime4 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime5)  , NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual1 = uwbTdoaDistDiff[anchor];
  //uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual2 = uwbTdoaDistDiff[anchor];

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual1);
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual2);
}

void testDifferenceOfDistanceWithTwoAnchors3FramesWithClockDriftAndLostMessageA0() {
  // Fixture
  // Two anchors, separated by 1.0m
  // Distance from A0 to tag is 2.0m
  // Distance from other anchor to tag is 2.5m
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

  // Message from A5 -> A0 lost
  //                    A       Arrival Time                       A0                                A1  A2  A3  A4  A5                     A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0 + timeA0ToTag), iTxTime0                        , NS, NS, NS, NS, NS                   , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1 + timeAnToTag), drift(aD, iTxTime0 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime1)  , NS, NS);
  // Lost packet
  // mockMessageFromAnchor(0     , drift(tD, iTxTime2 + timeA0ToTag), iTxTime2                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime3 + timeAnToTag), drift(aD, iTxTime2 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime3)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime4 + timeA0ToTag), iTxTime4                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime5 + timeAnToTag), drift(aD, iTxTime4 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime5)  , NS, NS);

  ignoreKalmanEstimatorValidation();

  // Test
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);

  // At this point we have an accepted distance diff for the anchor. The following packets should not be accepted and update the distance diff so this will be the expectation.
  float expectedDiff = uwbTdoaDistDiff[anchor];

  // Lost packet
  // uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  uwbTdoaTagAlgorithm.onEvent(&dev, eventPacketReceived);
  float actual = uwbTdoaDistDiff[anchor];

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(0.01, expectedDiff, actual);
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

  //                    A       Arrival Time                       A0                                A1  A2  A3  A4  A5                     A6  A7
  mockMessageFromAnchor(0     , drift(tD, iTxTime0 + timeA0ToTag), iTxTime0                        , NS, NS, NS, NS, NS                   , NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime1 + timeAnToTag), drift(aD, iTxTime0 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime1)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime2 + timeA0ToTag), iTxTime2                        , NS, NS, NS, NS, iTxTime1 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime3 + timeAnToTag), drift(aD, iTxTime2 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime3)  , NS, NS);
  mockMessageFromAnchor(0     , drift(tD, iTxTime4 + timeA0ToTag), iTxTime4                        , NS, NS, NS, NS, iTxTime3 + timeA0ToAn, NS, NS);
  mockMessageFromAnchor(anchor, drift(tD, iTxTime5 + timeAnToTag), drift(aD, iTxTime4 + timeA0ToAn), NS, NS, NS, NS, drift(aD, iTxTime5)  , NS, NS);

  ignoreKalmanEstimatorValidation();

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
  #ifdef ESTIMATOR_TYPE_kalman
  stateEstimatorEnqueueTDOA_IgnoreAndReturn(true);
  #endif
}

#ifdef ESTIMATOR_TYPE_kalman
#define STATE_ESTIMATOR_MAX_NR_OF_CALLS 10

static tdoaMeasurement_t stateEstimatorExpectations[STATE_ESTIMATOR_MAX_NR_OF_CALLS];
static int stateEstimatorIndex = 0;

static bool stateEstimatorEnqueueTDOAMockCallback(tdoaMeasurement_t* actual, int cmock_num_calls) {
  char message[100];
  sprintf(message, "Failed in call %i to stateEstimatorEnqueueTDOA()", cmock_num_calls);

  tdoaMeasurement_t* expected = &stateEstimatorExpectations[cmock_num_calls];
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.01, expected->distanceDiff, actual->distanceDiff, message);

  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->timeBetweenMeasurements, actual->timeBetweenMeasurements, message);

  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[0].x, actual->anchorPosition[0].x, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[0].y, actual->anchorPosition[0].y, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[0].z, actual->anchorPosition[0].z, message);

  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[1].x, actual->anchorPosition[1].x, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[1].y, actual->anchorPosition[1].y, message);
  TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.0, expected->anchorPosition[1].z, actual->anchorPosition[1].z, message);

  return true;
}
#endif

static void mockKalmanEstimator(uint8_t anchor, double distanceDiff, double timeBetweenMeasurements) {
  #ifdef ESTIMATOR_TYPE_kalman
    TEST_ASSERT_TRUE(stateEstimatorIndex < STATE_ESTIMATOR_MAX_NR_OF_CALLS);

    stateEstimatorEnqueueTDOA_StubWithCallback(stateEstimatorEnqueueTDOAMockCallback);

    tdoaMeasurement_t* measurement = &stateEstimatorExpectations[stateEstimatorIndex];

    measurement->distanceDiff = distanceDiff;
    measurement->timeBetweenMeasurements = timeBetweenMeasurements;

    // Always comparing to anchor 0 for now
    measurement->anchorPosition[0].x = options.anchorPosition[0].x;
    measurement->anchorPosition[0].y = options.anchorPosition[0].y;
    measurement->anchorPosition[0].z = options.anchorPosition[0].z;

    measurement->anchorPosition[1].x = options.anchorPosition[anchor].x;
    measurement->anchorPosition[1].y = options.anchorPosition[anchor].y;
    measurement->anchorPosition[1].z = options.anchorPosition[anchor].z;

    stateEstimatorIndex++;
  #else
    TEST_FAIL_MESSAGE("The kalman filter is not enabled. Are you sure you want to use the mock?");
  #endif
}

static void mockKalmanEstimator_resetMock() {
  #ifdef ESTIMATOR_TYPE_kalman
    stateEstimatorIndex = 0;
  #else
    TEST_FAIL_MESSAGE("The kalman filter is not enabled. Are you sure you want to use the mock?");
  #endif
}