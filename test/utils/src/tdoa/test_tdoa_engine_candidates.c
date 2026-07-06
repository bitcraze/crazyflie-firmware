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

static int countCandidatesWithIdB(const uint8_t idB) {
  int n = 0;
  for (int i = 0; i < capturedCount; i++) {
    if (captured[i].idB == idB) {
      n++;
    }
  }
  return n;
}

static int countSelected(void) {
  int n = 0;
  for (int i = 0; i < capturedCount; i++) {
    if (captured[i].isSelected) {
      n++;
    }
  }
  return n;
}

void testThatAllValidCandidatesAreLoggedWhenEnabled(void) {
  // Fixture: three valid candidates for packets from anchor 1
  fixtureAddValidCandidate(1, 2, 42);
  fixtureAddValidCandidate(1, 3, 43);
  fixtureAddValidCandidate(1, 4, 44);
  state.candidateLogEnable = 1;

  // Test
  fixtureProcessPacket(1);

  // Assert: no truncation - every valid candidate is emitted exactly once
  TEST_ASSERT_EQUAL_INT(3, capturedCount);
  TEST_ASSERT_EQUAL_INT(1, countCandidatesWithIdB(2));
  TEST_ASSERT_EQUAL_INT(1, countCandidatesWithIdB(3));
  TEST_ASSERT_EQUAL_INT(1, countCandidatesWithIdB(4));
  for (int i = 0; i < capturedCount; i++) {
    TEST_ASSERT_EQUAL_UINT8(1, captured[i].idA);
  }
}

void testThatTheSelectedPairIsAlwaysInTheLog(void) {
  // Fixture
  fixtureAddValidCandidate(1, 2, 42);
  fixtureAddValidCandidate(1, 3, 43);
  fixtureAddValidCandidate(1, 4, 44);
  state.candidateLogEnable = 1;

  // Test: process several packets; the random matcher's rotating offset
  // varies which pair is selected
  for (int packet = 0; packet < 10; packet++) {
    capturedCount = 0;
    fixtureProcessPacket(1);

    // Assert: complete group with exactly one selected pair, every time
    TEST_ASSERT_EQUAL_INT(3, capturedCount);
    TEST_ASSERT_EQUAL_INT(1, countSelected());
  }
}

void testThatLoggingDoesNotCreateAnchorContexts(void) {
  // Fixture: matching disabled so that any storage mutation is attributable
  // to the logging path alone (the live matchers have their own GetCreate
  // behavior, which is stock firmware and out of scope)
  tdoaEngineInit(&state, NOW_MS, stubSendTdoaToEstimator, TS_FREQ, TdoaEngineMatchingAlgorithmNone);
  state.candidateLogEnable = 1;

  fixtureAddValidCandidate(1, 2, 42);

  // Anchor 9 is in the remote list of anchor 1 (valid ToF + seqNr), but has
  // no context of its own in storage
  tdoaAnchorContext_t ctx;
  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, 1, NOW_MS, &ctx);
  tdoaStorageSetRemoteRxTime(&ctx, 9, 4711, 99);
  tdoaStorageSetRemoteTimeOfFlight(&ctx, 9, 1000);

  // Test
  fixtureProcessPacket(1);

  // Assert: only the known candidate is emitted, nothing is flagged selected
  // (no matcher ran), and - crucially - anchor 9 was NOT created in storage
  // as a side effect of logging
  TEST_ASSERT_EQUAL_INT(1, capturedCount);
  TEST_ASSERT_EQUAL_UINT8(2, captured[0].idB);
  TEST_ASSERT_EQUAL_UINT8(0, captured[0].isSelected);
  TEST_ASSERT_FALSE(tdoaStorageIsAnchorInStorage(state.anchorInfoArray, 9));
}
