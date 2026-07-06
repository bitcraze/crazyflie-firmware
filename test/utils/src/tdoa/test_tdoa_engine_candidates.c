// Unit tests for the TDoA candidate logging in tdoaEngine.c
//
// tdoaEngine.c, tdoaStorage.c, tdoaStats.c and statsCnt.c are compiled for
// real. The eventtrigger module is mocked so emitted estTdoaCand events can
// be captured and inspected. The clock correction engine is mocked so that
// packets pass the time-is-good and clock-correction gates.

// File under test
#include "tdoaEngine.h"

// Real dependencies (their sources are compiled because these headers are
// included here - see tools/test/rakefile_helper.rb extract_headers)
#include "tdoaStorage.h"
#include "tdoaStats.h"
#include "statsCnt.h"

#include "unity.h"
#include <string.h>

#include "mock_clockCorrectionEngine.h"
#include "mock_eventtrigger.h"

// Mirrors the estTdoaCand payload defined by EVENTTRIGGER() in tdoaEngine.c
typedef struct {
  uint32_t group;
  uint8_t idA;
  uint8_t idB;
  float distanceDiff;
  uint8_t isSelected;
} __attribute__((packed)) candidatePayload_t;

#define MAX_CAPTURED_EVENTS 64
static candidatePayload_t captured[MAX_CAPTURED_EVENTS];
static int capturedCount;

static tdoaEngineState_t state;

#define TS_FREQ (499.2e6 * 128)
static const uint32_t NOW_MS = 1000;
static const int64_t TX_AN = 1234567;
static const int64_t RX_AN = 7654321;

static void stubSendTdoaToEstimator(tdoaMeasurement_t* m) {
  (void)m;
}

static void captureEventTrigger(const eventtrigger* event, int cmock_num_calls) {
  (void)cmock_num_calls;
  TEST_ASSERT_EQUAL_STRING("estTdoaCand", event->name);
  TEST_ASSERT_EQUAL_UINT8(sizeof(candidatePayload_t), event->payloadSize);
  TEST_ASSERT_TRUE(capturedCount < MAX_CAPTURED_EVENTS);
  memcpy(&captured[capturedCount], event->payload, sizeof(candidatePayload_t));
  capturedCount++;
}

void setUp(void) {
  memset(&state, 0, sizeof(state));
  memset(captured, 0, sizeof(captured));
  capturedCount = 0;

  tdoaEngineInit(&state, NOW_MS, stubSendTdoaToEstimator, TS_FREQ, TdoaEngineMatchingAlgorithmRandom);

  // All packets pass the clock correction gates
  clockCorrectionEngineCalculate_IgnoreAndReturn(1.0);
  clockCorrectionEngineUpdate_IgnoreAndReturn(true);
  clockCorrectionEngineGet_IgnoreAndReturn(1.0);

  eventTrigger_StubWithCallback(captureEventTrigger);
}

// Make remote anchor idB a valid candidate of packets from anchor idA:
// known time-of-flight and a fresh (matching) sequence number.
static void fixtureAddValidCandidate(const uint8_t idA, const uint8_t idB, const uint8_t seqNr) {
  tdoaAnchorContext_t ctx;

  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, idA, NOW_MS, &ctx);
  tdoaStorageSetRemoteRxTime(&ctx, idB, 4711, seqNr);
  tdoaStorageSetRemoteTimeOfFlight(&ctx, idB, 1000);

  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, idB, NOW_MS, &ctx);
  tdoaStorageSetRxTxData(&ctx, 1111, 2222, seqNr);
}

// Process one packet received from anchor idA
static void fixtureProcessPacket(const uint8_t idA) {
  tdoaAnchorContext_t ctx;
  tdoaEngineGetAnchorCtxForPacketProcessing(&state, idA, NOW_MS, &ctx);
  // Non-zero previous rx/tx times are required by the clock correction update
  tdoaStorageSetRxTxData(&ctx, 3333, 4444, 17);
  tdoaEngineProcessPacket(&state, &ctx, TX_AN, RX_AN);
}

void testThatNoCandidatesAreLoggedWhenLoggingIsDisabled(void) {
  // Fixture: a valid candidate exists, but logging is off (default after init)
  fixtureAddValidCandidate(1, 2, 42);

  // Test
  fixtureProcessPacket(1);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, capturedCount);
}
