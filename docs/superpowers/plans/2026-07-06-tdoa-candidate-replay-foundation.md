# TDoA Candidate Capture + Offline-Replay Foundation — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the vibe-coded TDoA candidate-logging branch into a verified capture → readback → replay foundation, per the approved spec `docs/superpowers/specs/2026-07-02-tdoa-candidate-replay-foundation-design.md`.

**Architecture:** Firmware emits one `estTdoaCand` event per valid anchor-pair candidate per packet (complete, side-effect-free, grouped). The uSD readback script machine-checks losslessness. A small Python seam (`tdoa_selection.py` grouping/policies + new `tdoa_replay.py` replay entry point) feeds logged samples through the real `cffirmware` Kalman core. `replay_tdoa.py` becomes a thin CLI consumer.

**Tech Stack:** STM32 firmware (C11), Unity/CMock unit tests via `rake unit`, Python 3 + pytest, SWIG `cffirmware` bindings, bash tooling, `cfcli`.

## Global Constraints

- Work on branch `tdoa-candidate-logging`; commit after every task.
- Scope is the **non-hybrid** TDoA3 build (`configs/tdoa3_lh_groundtruth.conf`). `CONFIG_DECK_LOCO_TDOA3_HYBRID_MODE` is out of scope (spec C2 scope note).
- The live estimator path must not change. Candidate logging must be side-effect-free on live state both disabled and enabled (spec C4); the only permitted effect is event emission.
- Param stays named `tdoaEngine.logCand`; semantics: `0` = off, any non-zero = log **all** valid candidates. No partial/truncated mode exists (spec C1).
- Frozen event payload: `estTdoaCand = (uint32 group, uint8 idA, uint8 idB, float distanceDiff, uint8 isSelected)`.
- Frozen group schema: `{'group': int, 't_ms': float, 'idA': int, 'candidates': [{'idB': int, 'distanceDiff': float, 'isSelected': bool}]}`.
- Firmware unit tests: `rake unit test_tdoa_engine_candidates.c` (single file) or `rake unit` (all). Run from repo root.
- Python tests: `PYTHONPATH=build python3 -m pytest <file> -v` from repo root. Bindings build: `make bindings_python` (only needed for the smoke test in Task 9).
- Hardware logs stay **local**; do not commit any collected log file (spec resolved decision 3).
- Autonomous flight only with the user's explicit go-ahead, each time.

---

### Task 1: EVENTTRIGGER expansion under UNIT_TEST_MODE + candidate-logging test scaffold

`eventtrigger.h` currently defines `EVENTTRIGGER(...)` as *empty* under `UNIT_TEST_MODE`, so `tdoaEngine.c` (which writes `eventTrigger_estTdoaCand_payload.group = ...`) cannot compile in a unit test. Fix the header so the macro expands fully in tests (minus the linker-section attributes), and create the test scaffold that all firmware tasks build on.

**Files:**
- Modify: `src/modules/interface/eventtrigger.h:97-168`
- Create: `test/utils/src/tdoa/test_tdoa_engine_candidates.c`

**Interfaces:**
- Consumes: `tdoaEngineInit`, `tdoaEngineProcessPacket`, `tdoaEngineGetAnchorCtxForPacketProcessing` (tdoaEngine.h); `tdoaStorage*` setters (tdoaStorage.h); CMock mocks `mock_eventtrigger.h`, `mock_clockCorrectionEngine.h`.
- Produces: the test file with capture harness (`captured[]`, `capturedCount`, `captureEventTrigger`) and fixtures (`fixtureAddValidCandidate`, `fixtureProcessPacket`) that Tasks 2–4 add test functions to. A `candidatePayload_t` packed struct mirroring the event payload.

- [ ] **Step 1: Write the test scaffold with one test (logging disabled after init)**

Create `test/utils/src/tdoa/test_tdoa_engine_candidates.c`:

```c
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
```

- [ ] **Step 2: Run the test to verify it fails to compile**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: FAIL — compile error in `tdoaEngine.c` along the lines of `'eventTrigger_estTdoaCand_payload' undeclared` (the empty `EVENTTRIGGER` macro under `UNIT_TEST_MODE` defines neither the payload struct nor the trigger).

- [ ] **Step 3: Make EVENTTRIGGER expand fully under UNIT_TEST_MODE**

In `src/modules/interface/eventtrigger.h`, the macro machinery (lines 97–168) is wrapped in `#ifndef UNIT_TEST_MODE` with an empty `#define EVENTTRIGGER(NAME, ...)` in the `#else` branch. Restructure so the machinery is unconditional and only the linker-section attributes differ:

1. Delete the `#ifndef UNIT_TEST_MODE` at line 97, and the whole `#else // UNIT_TEST_MODE ... #endif // UNIT_TEST_MODE` block at lines 163–168 (keeping the machinery in between).
2. Immediately before the (now unconditional) macro machinery, add:

```c
// In unit test builds there is no .eventtrigger linker section, but the full
// payload/trigger definitions are still needed so that files using
// EVENTTRIGGER payloads compile and can be tested.
#ifndef UNIT_TEST_MODE
#define _EVENTTRIGGER_ATTRIBUTES(NAME) __attribute__((section(".eventtrigger." #NAME), used))
#else
#define _EVENTTRIGGER_ATTRIBUTES(NAME)
#endif
```

3. In `_EVENTTRIGGER_NON_EMPTY` replace
   `static const eventtrigger eventTrigger_##NAME __attribute__((section(".eventtrigger." #NAME), used)) = {`
   with
   `static const eventtrigger eventTrigger_##NAME _EVENTTRIGGER_ATTRIBUTES(NAME) = {`
4. Make the same replacement in `_EVENTTRIGGER_EMPTY`.

- [ ] **Step 4: Run the test to verify it passes**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: PASS (1 test).

- [ ] **Step 5: Run the full firmware unit test suite to verify no regression**

Run: `rake unit`
Expected: all tests pass (same set as before this task, plus the new one).

- [ ] **Step 6: Commit**

```bash
git add src/modules/interface/eventtrigger.h test/utils/src/tdoa/test_tdoa_engine_candidates.c
git commit -m "test: unit-test scaffold for TDoA candidate logging

EVENTTRIGGER now expands fully under UNIT_TEST_MODE (minus linker-section
attributes) so files that fill event payloads can be unit tested."
```

---

### Task 2: C1 — remove per-packet truncation; logCand becomes an enable flag

The draft truncates the emit loop after `logCand` events, which can silently drop the live-selected pair (emit order is storage order; the matcher picks from a rotating offset). Per the spec's resolved decision, truncation is removed entirely: `logCand` non-zero = log **every** valid candidate. The state field is renamed to match the new meaning.

**Files:**
- Modify: `src/utils/interface/tdoa/tdoaEngine.h:38-48` (field rename + comment)
- Modify: `src/utils/src/tdoa/tdoaEngine.c:57-67,78,149-193,300` (rename, loop cap removal, comments)
- Modify: `src/modules/src/tdoaEngineInstance.c:117-129` (param binding + doc rewrite)
- Test: `test/utils/src/tdoa/test_tdoa_engine_candidates.c`

**Interfaces:**
- Consumes: scaffold from Task 1.
- Produces: `tdoaEngineState_t.candidateLogEnable` (uint8_t, 0 = off, non-zero = log all) replacing `candidateLogMaxCount`. Tasks 3–4 set `state.candidateLogEnable = 1` in their tests. Param `tdoaEngine.logCand` binds to this field.

- [ ] **Step 1: Rename `candidateLogMaxCount` → `candidateLogEnable` (mechanical)**

In `src/utils/interface/tdoa/tdoaEngine.h`, replace the field and its comment:

```c
  // Candidate logging. When candidateLogEnable is non-zero, every processed
  // packet emits an "estTdoaCand" event trigger for each valid anchor-pair
  // candidate (before selection collapses them to one). All valid candidates
  // are always emitted so the log is a complete record of what the matcher
  // saw; there is no partial mode. 0 disables logging.
  uint8_t candidateLogEnable;
```

In `src/utils/src/tdoa/tdoaEngine.c` replace all three references (`tdoaEngineInit` at line 78, the parameter check in `logTdoaCandidates`, the gate at line 300) with `candidateLogEnable`. In `src/modules/src/tdoaEngineInstance.c` update the `PARAM_ADD` binding to `&tdoaEngineState.candidateLogEnable`.

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: PASS (rename only, no behavior change).

- [ ] **Step 2: Write the failing tests (no truncation; selected pair always present)**

Add to `test/utils/src/tdoa/test_tdoa_engine_candidates.c`:

```c
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
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: FAIL — `testThatAllValidCandidatesAreLoggedWhenEnabled` sees `capturedCount == 1` (the loop still truncates at `candidateLogEnable = 1` events).

- [ ] **Step 4: Remove the truncation**

In `logTdoaCandidates` in `src/utils/src/tdoa/tdoaEngine.c`:
- Delete the `uint8_t loggedCount = 0;` declaration and the `loggedCount++;` statement.
- Change the loop condition from
  `for (int index = 0; index < remoteCount && loggedCount < engineState->candidateLogEnable; index++) {`
  to
  `for (int index = 0; index < remoteCount; index++) {`

Update the function's comment block (lines 149–154) to:

```c
// Emit an estTdoaCand event for every valid anchor-pair candidate of this
// packet. A candidate (idA, idB) is considered valid using the same criteria
// as the matching algorithms: idA must have a known time-of-flight to idB and
// the cached sequence number must match. All valid candidates are always
// emitted (no truncation) so that the log is a complete record; the pair
// chosen by the live matching algorithm is flagged with isSelected.
```

Also update the `EVENTTRIGGER` doc comment at lines 57–61 (replace "when engineState->candidateLogMaxCount > 0" with "when engineState->candidateLogEnable is non-zero").

- [ ] **Step 5: Run tests to verify they pass**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: PASS (3 tests).

- [ ] **Step 6: Rewrite the param documentation**

In `src/modules/src/tdoaEngineInstance.c`, replace the whole `logCand` doc comment + PARAM_ADD with:

```c
/**
 * @brief Enable logging of all TDoA anchor-pair candidates (0 = off, non-zero = on).
 *
 * When enabled, every processed packet emits one "estTdoaCand" event trigger
 * per valid candidate pair, before the matching algorithm collapses the
 * candidates to a single pair. This records all candidate pairs (normally
 * discarded) to the uSD deck for offline replay and A/B testing of the
 * selection policy. All valid candidates are always logged so the log is a
 * complete record; there is no partial mode. Note that this increases the
 * event rate to the uSD deck considerably. Check for dropped events after a
 * run by comparing the usd.eventsRequested and usd.eventsWritten log
 * variables (tools/usdlog/read_usd_log.sh does this automatically).
 */
PARAM_ADD(PARAM_UINT8, logCand, &tdoaEngineState.candidateLogEnable)
```

Run: `rake unit`
Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/utils/interface/tdoa/tdoaEngine.h src/utils/src/tdoa/tdoaEngine.c src/modules/src/tdoaEngineInstance.c test/utils/src/tdoa/test_tdoa_engine_candidates.c
git commit -m "fix: log all TDoA candidates, no per-packet truncation (spec C1)

Truncation walked the remote list in storage order while the matcher picks
from a rotating offset, so the live-selected pair could silently miss the
cap, corrupting the baseline. logCand is now a plain enable flag."
```

---

### Task 3: C4 — logging must not mutate anchor storage

`logTdoaCandidates` uses `tdoaStorageGetCreateAnchorCtx`, which on a miss *creates* a context slot by evicting the oldest stored anchor. Enabling logging could therefore perturb the pipeline being observed. Switch to the non-creating accessor.

**Files:**
- Modify: `src/utils/src/tdoa/tdoaEngine.c` (`logTdoaCandidates`, one call)
- Test: `test/utils/src/tdoa/test_tdoa_engine_candidates.c`

**Interfaces:**
- Consumes: scaffold from Task 1, `candidateLogEnable` from Task 2, `tdoaStorageGetAnchorCtx` / `tdoaStorageIsAnchorInStorage` (tdoaStorage.h).
- Produces: nothing new for later tasks (behavioral guarantee only).

- [ ] **Step 1: Write the failing test**

Add to `test/utils/src/tdoa/test_tdoa_engine_candidates.c`:

```c
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
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: FAIL — `tdoaStorageIsAnchorInStorage(..., 9)` is true (the GetCreate call in the logging path created it).

- [ ] **Step 3: Switch to the non-creating accessor**

In `logTdoaCandidates` in `src/utils/src/tdoa/tdoaEngine.c`, change:

```c
    tdoaAnchorContext_t otherAnchorCtx;
    if (!tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, &otherAnchorCtx)) {
      continue;
    }
```

to:

```c
    // Non-creating accessor: observing the candidates must never mutate the
    // anchor storage (a GetCreate miss would evict the oldest stored anchor)
    tdoaAnchorContext_t otherAnchorCtx;
    if (!tdoaStorageGetAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, &otherAnchorCtx)) {
      continue;
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add src/utils/src/tdoa/tdoaEngine.c test/utils/src/tdoa/test_tdoa_engine_candidates.c
git commit -m "fix: candidate logging must not mutate anchor storage (spec C4)

GetCreateAnchorCtx evicts the oldest slot on a miss, so enabling logging
could perturb the live pipeline being observed. Use the non-creating
accessor in the logging path."
```

---

### Task 4: C2/C3 — feasible-set parity and exact grouping tests

Characterization tests for the remaining capture-faithfulness criteria. These are expected to pass against the code as it stands after Tasks 2–3; if any fails, that is a real firmware bug — fix it within this task following the same red→green pattern.

**Files:**
- Test: `test/utils/src/tdoa/test_tdoa_engine_candidates.c`

**Interfaces:**
- Consumes: scaffold from Task 1, `candidateLogEnable` from Task 2.
- Produces: nothing new (verified guarantees only).

- [ ] **Step 1: Write the C2 parity tests (validity predicate matches the matcher)**

Add to `test/utils/src/tdoa/test_tdoa_engine_candidates.c`:

```c
void testThatCandidateWithoutTimeOfFlightIsNotLogged(void) {
  // Fixture: anchor 3 has a remote rx time but no time-of-flight, which makes
  // it unselectable for the matcher - it must not appear in the log either
  fixtureAddValidCandidate(1, 2, 42);
  tdoaAnchorContext_t ctx;
  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, 1, NOW_MS, &ctx);
  tdoaStorageSetRemoteRxTime(&ctx, 3, 4711, 43);
  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, 3, NOW_MS, &ctx);
  tdoaStorageSetRxTxData(&ctx, 1111, 2222, 43);
  state.candidateLogEnable = 1;

  // Test
  fixtureProcessPacket(1);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, capturedCount);
  TEST_ASSERT_EQUAL_UINT8(2, captured[0].idB);
}

void testThatCandidateWithStaleSeqNrIsNotLogged(void) {
  // Fixture: remote data for anchor 3 was cached at seqNr 50, but anchor 3
  // has since transmitted seqNr 51 - stale, unselectable, must not be logged
  fixtureAddValidCandidate(1, 2, 42);
  tdoaAnchorContext_t ctx;
  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, 1, NOW_MS, &ctx);
  tdoaStorageSetRemoteRxTime(&ctx, 3, 4711, 50);
  tdoaStorageSetRemoteTimeOfFlight(&ctx, 3, 1000);
  tdoaStorageGetCreateAnchorCtx(state.anchorInfoArray, 3, NOW_MS, &ctx);
  tdoaStorageSetRxTxData(&ctx, 1111, 2222, 51);
  state.candidateLogEnable = 1;

  // Test
  fixtureProcessPacket(1);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, capturedCount);
  TEST_ASSERT_EQUAL_UINT8(2, captured[0].idB);
}

void testThatNoCandidatesAreLoggedWhenClockCorrectionIsInvalid(void) {
  // Fixture: clock correction gate closed (Get returns 0.0); note setUp()
  // already queued IgnoreAndReturn(1.0), so re-init the mock expectations
  mock_clockCorrectionEngine_Init();
  clockCorrectionEngineCalculate_IgnoreAndReturn(1.0);
  clockCorrectionEngineUpdate_IgnoreAndReturn(true);
  clockCorrectionEngineGet_IgnoreAndReturn(0.0);

  fixtureAddValidCandidate(1, 2, 42);
  state.candidateLogEnable = 1;

  // Test
  fixtureProcessPacket(1);

  // Assert: gate matches the matcher's clock-correction condition
  TEST_ASSERT_EQUAL_INT(0, capturedCount);
}
```

- [ ] **Step 2: Write the C3 grouping tests**

Add:

```c
void testThatEventsOfOnePacketShareOneGroupAndGroupsDiffer(void) {
  // Fixture
  fixtureAddValidCandidate(1, 2, 42);
  fixtureAddValidCandidate(1, 3, 43);
  fixtureAddValidCandidate(1, 4, 44);
  state.candidateLogEnable = 1;

  // Test: two packets
  fixtureProcessPacket(1);
  fixtureProcessPacket(1);

  // Assert: 3 + 3 events; first three share one group, last three share
  // another, and the two groups differ
  TEST_ASSERT_EQUAL_INT(6, capturedCount);
  TEST_ASSERT_EQUAL_UINT32(captured[0].group, captured[1].group);
  TEST_ASSERT_EQUAL_UINT32(captured[0].group, captured[2].group);
  TEST_ASSERT_EQUAL_UINT32(captured[3].group, captured[4].group);
  TEST_ASSERT_EQUAL_UINT32(captured[3].group, captured[5].group);
  TEST_ASSERT_NOT_EQUAL(captured[0].group, captured[3].group);
}

void testThatGroupCounterAdvancesForPacketsWithZeroCandidates(void) {
  // Fixture: anchor 1 has candidates, anchor 5 has none
  fixtureAddValidCandidate(1, 2, 42);
  state.candidateLogEnable = 1;

  // Test: packet from 1, packet from 5 (zero candidates), packet from 1
  fixtureProcessPacket(1);
  uint32_t firstGroup = captured[0].group;
  fixtureProcessPacket(5);
  fixtureProcessPacket(1);

  // Assert: the empty packet consumed a group number; groups never merge.
  // Offline, a gap in group numbers is therefore benign (empty packet), and
  // events of different packets can never share a group value.
  TEST_ASSERT_EQUAL_INT(2, capturedCount);
  TEST_ASSERT_EQUAL_UINT32(firstGroup + 2, captured[1].group);
}
```

- [ ] **Step 3: Run the tests**

Run: `rake unit test_tdoa_engine_candidates.c`
Expected: PASS (9 tests). If any of these fails, the validity predicate or grouping in `logTdoaCandidates` genuinely diverges from the spec — fix `tdoaEngine.c` (not the test) and re-run until green.

- [ ] **Step 4: Run the full suite and commit**

Run: `rake unit`
Expected: all pass.

```bash
git add test/utils/src/tdoa/test_tdoa_engine_candidates.c
git commit -m "test: TDoA candidate feasible-set parity and exact grouping (spec C2, C3)"
```

---

### Task 5: R1 firmware — expose uSD drop counters as log variables

The firmware counts `eventsRequested` vs `eventsWritten` (reset at each log start) but only DEBUG_PRINTs them at log stop. Expose both as `usd.*` log variables so losslessness can be checked over the log framework.

**Files:**
- Modify: `src/deck/drivers/src/usddeck.c:1116-1129` (the `usd` LOG_GROUP)

**Interfaces:**
- Consumes: `static usdLogStats_t usdLogStats` (usddeck.c:221).
- Produces: log variables `usd.eventsRequested` and `usd.eventsWritten` (LOG_UINT32) — consumed by Task 6's script and the Task 10 docs.

- [ ] **Step 1: Add the log variables**

In `src/deck/drivers/src/usddeck.c`, inside `LOG_GROUP_START(usd)` (after the `fatWrBps` entry), add:

```c
/**
 * @brief Number of events requested to be written to the uSD card during the current/last log run
 */
LOG_ADD(LOG_UINT32, eventsRequested, &usdLogStats.eventsRequested)
/**
 * @brief Number of events actually written to the uSD card during the current/last log run.
 *
 * If lower than eventsRequested, the ring buffer overflowed and the log has
 * holes; do not trust lossless replay of that log.
 */
LOG_ADD(LOG_UINT32, eventsWritten, &usdLogStats.eventsWritten)
```

- [ ] **Step 2: Verify the firmware builds**

Run: `make cf2_defconfig && make -j$(nproc)`
Expected: build completes, `build/cf2.bin` produced. (There is no unit-test harness for usddeck; the behavioral check happens on hardware in Task 11.)

- [ ] **Step 3: Commit**

```bash
git add src/deck/drivers/src/usddeck.c
git commit -m "feat: expose uSD event drop counters as log variables (spec R1)

usd.eventsRequested / usd.eventsWritten were previously only DEBUG_PRINTed
at log stop; as log variables the drop check can be automated."
```

---

### Task 6: R1 tooling — automatic drop check in read_usd_log.sh

Every readback must verify losslessness: after stopping logging, read the two counters and fail loudly on a mismatch.

**Files:**
- Modify: `tools/usdlog/read_usd_log.sh` (insert after the `param set usd.logging 0` block, before the `mem list` block)

**Interfaces:**
- Consumes: `usd.eventsRequested` / `usd.eventsWritten` log variables from Task 5; `cfcli log print <names> --csv` (streams one CSV line per sample; bounded with `--timeout`).
- Produces: readback script that exits 1 on detected drops. Task 10 documents it; Task 11 exercises it on hardware.

- [ ] **Step 1: Insert the drop-check block**

In `tools/usdlog/read_usd_log.sh`, after:

```bash
echo ">> Stopping logging (usd.logging = 0) so the file becomes readable..."
"$CFCLI" -u "$URI" param set usd.logging 0
```

insert:

```bash
echo ">> Checking for dropped uSD events (usd.eventsRequested vs usd.eventsWritten)..."
# The counters are reset when logging starts and keep their final values after
# stop. `log print` streams one CSV line per sample; bound it with --timeout
# and take the last complete sample.
DROP_LINE="$("$CFCLI" -u "$URI" --timeout 3000 --csv log print usd.eventsRequested,usd.eventsWritten 2>/dev/null | tail -n 1 || true)"
EVT_REQ="$(echo "$DROP_LINE" | awk -F',' '{print $(NF-1)}')"
EVT_WR="$(echo "$DROP_LINE" | awk -F',' '{print $NF}')"

if [[ ! "$EVT_REQ" =~ ^[0-9]+$ || ! "$EVT_WR" =~ ^[0-9]+$ ]]; then
  echo "!! WARNING: could not read usd.eventsRequested/usd.eventsWritten." >&2
  echo "   Firmware without the drop counters? Losslessness NOT verified." >&2
elif [[ "$EVT_REQ" != "$EVT_WR" ]]; then
  echo "!! DROPPED EVENTS: eventsRequested=$EVT_REQ eventsWritten=$EVT_WR ($((EVT_REQ - EVT_WR)) lost)." >&2
  echo "   The log has holes; do NOT trust replay results from it." >&2
  echo "   Mitigations: increase the ring buffer size (line 2 of config.txt)" >&2
  echo "   or switch to config_tdoa_candidates_lean.txt." >&2
  exit 1
else
  echo ">> OK: no dropped events (eventsRequested = eventsWritten = $EVT_REQ)."
fi
```

- [ ] **Step 2: Verify the script parses**

Run: `bash -n tools/usdlog/read_usd_log.sh && echo SYNTAX-OK`
Expected: `SYNTAX-OK`. If `shellcheck` is installed, also run `shellcheck tools/usdlog/read_usd_log.sh` and fix any error-level findings (the existing style may produce info-level notes; leave those).

Note: the exact CSV column order of `cfcli log print` is verified on hardware in Task 11; the parse-failure branch above degrades to a loud warning rather than a false pass.

- [ ] **Step 3: Commit**

```bash
git add tools/usdlog/read_usd_log.sh
git commit -m "feat: automatic uSD drop check on every log readback (spec R1)"
```

---

### Task 7: Seam — frozen group schema tests + baseline-reconstruction verifier

Pin down `build_candidate_groups` and the selection policies with unit tests (no bindings required), and add `verify_baseline_reconstruction` — the F1 measurement-level check that the baseline policy's reconstruction equals the independently logged `estTDOA` stream.

**Files:**
- Modify: `bindings/util/tdoa_selection.py` (add `verify_baseline_reconstruction`)
- Test: `test_python/test_tdoa_selection.py` (new)

**Interfaces:**
- Consumes: `build_candidate_groups(log_data)`, `BaselinePolicy` etc. (existing, `bindings/util/tdoa_selection.py`); `log_data` dicts in `cfusdlog.decode` shape: `{'estTdoaCand': {'timestamp': [...], 'group': [...], 'idA': [...], 'idB': [...], 'distanceDiff': [...], 'isSelected': [...]}, 'estTDOA': {'timestamp': [...], 'idA': [...], 'idB': [...], 'distanceDiff': [...]}}`.
- Produces: `verify_baseline_reconstruction(log_data) -> dict` with keys `n_groups, n_selectionless, n_baseline, n_est_tdoa, n_compared, n_mismatched`. Used by Task 8's CLI and the Task 10/11 checklist.

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_selection.py`:

```python
"""Tests for the candidate-group seam (no cffirmware bindings required)."""

from bindings.util.tdoa_selection import (
    build_candidate_groups,
    make_policy,
    verify_baseline_reconstruction,
)


def _cand_log(events):
    """Build log_data with an estTdoaCand block from a list of tuples:
    (timestamp, group, idA, idB, distanceDiff, isSelected)."""
    keys = ('timestamp', 'group', 'idA', 'idB', 'distanceDiff', 'isSelected')
    return {'estTdoaCand': {k: [e[i] for e in events] for i, k in enumerate(keys)}}


def test_events_are_grouped_by_group_value_in_log_order():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 1),
        (100.0, 7, 1, 3, 0.6, 0),
        (105.0, 9, 4, 5, -0.1, 1),
    ])
    groups = build_candidate_groups(log)
    assert [g['group'] for g in groups] == [7, 9]
    assert [g['idA'] for g in groups] == [1, 4]
    assert [len(g['candidates']) for g in groups] == [2, 1]


def test_group_time_is_min_of_event_timestamps():
    # uSD stamps events at write time; the packet time is the group minimum
    log = _cand_log([
        (103.0, 7, 1, 2, 0.5, 1),
        (101.0, 7, 1, 3, 0.6, 0),
    ])
    groups = build_candidate_groups(log)
    assert groups[0]['t_ms'] == 101.0


def test_group_schema_is_frozen():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 1)])
    group = build_candidate_groups(log)[0]
    assert set(group.keys()) == {'group', 't_ms', 'idA', 'candidates'}
    assert set(group['candidates'][0].keys()) == {'idB', 'distanceDiff', 'isSelected'}
    assert isinstance(group['group'], int)
    assert isinstance(group['t_ms'], float)
    assert isinstance(group['idA'], int)
    assert isinstance(group['candidates'][0]['isSelected'], bool)


def test_missing_or_empty_candidate_block_gives_no_groups():
    assert build_candidate_groups({}) == []
    assert build_candidate_groups({'estTdoaCand': {}}) == []


def test_baseline_policy_picks_the_selected_candidate():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    measurements = make_policy('baseline').select(group)
    assert measurements == [{'idA': 1, 'idB': 3, 'distanceDiff': 0.6}]


def test_baseline_policy_skips_groups_without_selection():
    # Legitimate case (spec C2 scope note): e.g. no matching algorithm ran
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 0)])
    group = build_candidate_groups(log)[0]
    assert make_policy('baseline').select(group) == []


def test_all_policy_returns_every_candidate():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    assert len(make_policy('all').select(group)) == 2


def test_round_robin_rotates_and_resets():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 0),
        (100.0, 7, 1, 3, 0.6, 1),
    ])
    group = build_candidate_groups(log)[0]
    policy = make_policy('round_robin')
    first = policy.select(group)[0]['idB']
    second = policy.select(group)[0]['idB']
    assert {first, second} == {2, 3}
    policy.reset()
    assert policy.select(group)[0]['idB'] == first


def test_verify_baseline_reconstruction_clean_log():
    log = _cand_log([
        (100.0, 7, 1, 2, 0.5, 1),
        (100.0, 7, 1, 3, 0.6, 0),
        (105.0, 8, 4, 5, -0.1, 1),
    ])
    log['estTDOA'] = {
        'timestamp': [100.0, 105.0],
        'idA': [1, 4],
        'idB': [2, 5],
        'distanceDiff': [0.5, -0.1],
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_groups'] == 2
    assert result['n_selectionless'] == 0
    assert result['n_baseline'] == 2
    assert result['n_est_tdoa'] == 2
    assert result['n_compared'] == 2
    assert result['n_mismatched'] == 0


def test_verify_baseline_reconstruction_detects_mismatch():
    log = _cand_log([(100.0, 7, 1, 2, 0.5, 1)])
    log['estTDOA'] = {
        'timestamp': [100.0],
        'idA': [1],
        'idB': [2],
        'distanceDiff': [0.7],  # differs from the flagged candidate
    }
    result = verify_baseline_reconstruction(log)
    assert result['n_mismatched'] == 1
```

- [ ] **Step 2: Run the tests to verify the new ones fail**

Run: `python3 -m pytest test_python/test_tdoa_selection.py -v`
Expected: the grouping/policy tests PASS (existing code), the two `verify_baseline_reconstruction` tests FAIL with `ImportError: cannot import name 'verify_baseline_reconstruction'`. If any grouping/policy test fails instead, the existing seam code has a real bug — fix `tdoa_selection.py`, not the test.

- [ ] **Step 3: Implement `verify_baseline_reconstruction`**

Append to `bindings/util/tdoa_selection.py`:

```python
def verify_baseline_reconstruction(log_data):
    """F1 measurement-level check: baseline reconstruction == logged estTDOA.

    The candidate flagged ``isSelected`` and the ``estTDOA`` event of the same
    packet both originate from the same ``calcDistanceDiff`` double in the
    firmware and are stored as float32, so matching entries must be *exactly*
    equal.

    The comparison is a strict in-order 1:1 zip of the two streams, which is
    only meaningful for drop-free logs -- verify losslessness (the
    usd.eventsRequested / usd.eventsWritten check) first.

    Returns a dict of counts; a faithful, drop-free log has
    ``n_mismatched == 0`` and ``n_baseline == n_est_tdoa == n_compared``.
    """
    groups = build_candidate_groups(log_data)
    policy = BaselinePolicy()
    baseline = []
    n_selectionless = 0
    for group in groups:
        measurements = policy.select(group)
        if measurements:
            baseline.extend(measurements)
        else:
            n_selectionless += 1

    est = log_data.get('estTDOA') or {}
    n_est_tdoa = len(est.get('timestamp', []))
    n_compared = min(len(baseline), n_est_tdoa)
    n_mismatched = 0
    for i in range(n_compared):
        m = baseline[i]
        if (int(est['idA'][i]) != m['idA']
                or int(est['idB'][i]) != m['idB']
                or float(est['distanceDiff'][i]) != m['distanceDiff']):
            n_mismatched += 1

    return {
        'n_groups': len(groups),
        'n_selectionless': n_selectionless,
        'n_baseline': len(baseline),
        'n_est_tdoa': n_est_tdoa,
        'n_compared': n_compared,
        'n_mismatched': n_mismatched,
    }
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest test_python/test_tdoa_selection.py -v`
Expected: PASS (11 tests).

- [ ] **Step 5: Commit**

```bash
git add bindings/util/tdoa_selection.py test_python/test_tdoa_selection.py
git commit -m "test: freeze candidate-group schema; add baseline-reconstruction verifier (spec F1)"
```

---

### Task 8: Seam — replay module extraction + replay_tdoa.py as thin consumer

Factor the replay entry point out of `replay_tdoa.py` into `bindings/util/tdoa_replay.py` so analysis scripts call it directly (spec Section 4). Add the F1 live-trajectory comparison and the reconstruction report to the CLI.

**Files:**
- Create: `bindings/util/tdoa_replay.py`
- Modify: `tools/usdlog/replay_tdoa.py` (remove moved functions, import the seam, add F1 reporting)
- Test: `test_python/test_tdoa_replay.py` (new; no bindings required)

**Interfaces:**
- Consumes: `EstimatorKalmanEmulator(anchor_positions)` with `run_one_1khz_iteration(samples)` and `TDOA_ENGINE_MEASUREMENT_NOISE_STD` (bindings/util/estimator_kalman_emulator.py); groups + policies from `tdoa_selection.py`; `verify_baseline_reconstruction` from Task 7.
- Produces (the frozen seam):
  - `extract_imu_samples(log_data) -> [(log_type, sample_dict)]`
  - `extract_state_estimate(log_data) -> [(t_ms, (x, y, z))]`
  - `apply_policy(policy, groups) -> [('estTDOA', {'idA', 'idB', 'distanceDiff', 'timestamp'})]`
  - `merge_samples(*sample_lists) -> [samples]` (timestamp-sorted, stable)
  - `replay(anchor_positions, imu_samples, tdoa_samples, params=None) -> [(t_ms, (x, y, z))]`, `params` dict supports `'tdoa_std'` (default 0.15)

- [ ] **Step 1: Write the failing tests**

Create `test_python/test_tdoa_replay.py`:

```python
"""Tests for the replay seam that do NOT require the cffirmware bindings.

The end-to-end replay (which does need the bindings) is covered by
test_tdoa_replay_smoke.py.
"""

from bindings.util.tdoa_replay import (
    apply_policy,
    extract_imu_samples,
    extract_state_estimate,
    merge_samples,
)
from bindings.util.tdoa_selection import build_candidate_groups, make_policy


def test_apply_policy_stamps_measurements_with_the_group_time():
    log = {'estTdoaCand': {
        'timestamp': [103.0, 101.0],
        'group': [7, 7],
        'idA': [1, 1],
        'idB': [2, 3],
        'distanceDiff': [0.5, 0.6],
        'isSelected': [1, 0],
    }}
    groups = build_candidate_groups(log)
    samples = apply_policy(make_policy('baseline'), groups)
    assert samples == [('estTDOA', {
        'idA': 1, 'idB': 2, 'distanceDiff': 0.5, 'timestamp': 101.0,
    })]


def test_merge_samples_orders_by_timestamp_and_is_stable():
    # F2: IMU and TDoA streams interleave into one correctly ordered stream;
    # equal timestamps keep their original relative order (stable sort)
    imu = [
        ('estAcceleration', {'timestamp': 100.0, 'acc.x': 0.0}),
        ('estGyroscope', {'timestamp': 100.0, 'gyro.x': 0.0}),
        ('estAcceleration', {'timestamp': 102.0, 'acc.x': 0.0}),
    ]
    tdoa = [
        ('estTDOA', {'timestamp': 101.0, 'idA': 1, 'idB': 2, 'distanceDiff': 0.5}),
        ('estTDOA', {'timestamp': 100.0, 'idA': 3, 'idB': 4, 'distanceDiff': 0.1}),
    ]
    merged = merge_samples(imu, tdoa)
    assert [s[1]['timestamp'] for s in merged] == [100.0, 100.0, 100.0, 101.0, 102.0]
    # Stability: within t=100.0 the IMU pair stays ahead of the TDoA sample
    assert [s[0] for s in merged[:3]] == ['estAcceleration', 'estGyroscope', 'estTDOA']


def test_extract_imu_samples_reads_both_imu_blocks():
    log = {
        'estAcceleration': {'timestamp': [1.0], 'acc.x': [0.1], 'acc.y': [0.2], 'acc.z': [1.0]},
        'estGyroscope': {'timestamp': [2.0], 'gyro.x': [0.0], 'gyro.y': [0.0], 'gyro.z': [0.0]},
    }
    samples = extract_imu_samples(log)
    assert {s[0] for s in samples} == {'estAcceleration', 'estGyroscope'}
    assert len(samples) == 2


def test_extract_state_estimate_reads_the_fixed_frequency_block():
    log = {'fixedFrequency': {
        'timestamp': [10.0, 20.0],
        'stateEstimate.x': [0.1, 0.2],
        'stateEstimate.y': [0.3, 0.4],
        'stateEstimate.z': [0.5, 0.6],
    }}
    assert extract_state_estimate(log) == [
        (10.0, (0.1, 0.3, 0.5)),
        (20.0, (0.2, 0.4, 0.6)),
    ]


def test_extract_state_estimate_handles_missing_block():
    assert extract_state_estimate({}) == []
    assert extract_state_estimate({'fixedFrequency': {'timestamp': [1.0]}}) == []


def test_ground_truth_interpolation_and_scoring():
    # Spec 5.1: ground-truth interpolation + scoring. These live in the CLI
    # module; importing it must not require the cffirmware bindings.
    from tools.usdlog.replay_tdoa import _interp_ground_truth, score_trajectory

    gt = [(100.0, (0.0, 0.0, 0.0)), (200.0, (1.0, 0.0, 0.0))]

    assert _interp_ground_truth(gt, 150.0) == (0.5, 0.0, 0.0)
    assert _interp_ground_truth(gt, 100.0) == (0.0, 0.0, 0.0)
    assert _interp_ground_truth(gt, 99.0) is None
    assert _interp_ground_truth(gt, 201.0) is None

    # Trajectory exactly on the ground truth scores zero error
    trajectory = [(100.0, (0.0, 0.0, 0.0)), (150.0, (0.5, 0.0, 0.0))]
    metrics, errors = score_trajectory(trajectory, gt, flyaway_threshold_m=0.3)
    assert metrics['n'] == 2
    assert metrics['rms'] == 0.0
    assert metrics['flyaway_frac'] == 0.0

    # A 1 m error on one of two samples is reflected in max and flyaway_frac
    trajectory = [(100.0, (0.0, 0.0, 0.0)), (150.0, (0.5, 1.0, 0.0))]
    metrics, errors = score_trajectory(trajectory, gt, flyaway_threshold_m=0.3)
    assert metrics['max'] == 1.0
    assert metrics['flyaway_frac'] == 0.5
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python3 -m pytest test_python/test_tdoa_replay.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bindings.util.tdoa_replay'`.

- [ ] **Step 3: Create the seam module**

Create `bindings/util/tdoa_replay.py`:

```python
"""
Replay seam: uSD log -> sample streams -> the real firmware Kalman core.

This is the stable, documented interface that analysis scripts build on::

    log_data = cfusdlog.decode(path)
    groups   = tdoa_selection.build_candidate_groups(log_data)
    imu      = tdoa_replay.extract_imu_samples(log_data)
    tdoa     = tdoa_replay.apply_policy(policy, groups)
    traj     = tdoa_replay.replay(anchor_positions, imu, tdoa,
                                  {'tdoa_std': 0.15})

``replay`` imports cffirmware lazily, so everything else in this module can be
used (and unit tested) without the SWIG bindings being built.

Sample format (shared with EstimatorKalmanEmulator): ``(log_type, sample_dict)``
where ``sample_dict`` always carries a ``'timestamp'`` in milliseconds.
"""

DEFAULT_TDOA_STD = 0.15


def extract_imu_samples(log_data):
    """Return IMU samples in the (log_type, sample_dict) form the emulator wants."""
    samples = []
    for log_type in ('estAcceleration', 'estGyroscope'):
        data = log_data.get(log_type)
        if not data or 'timestamp' not in data:
            continue
        n = len(data['timestamp'])
        for i in range(n):
            sample = {name: values[i] for name, values in data.items()}
            samples.append((log_type, sample))
    return samples


def extract_state_estimate(log_data):
    """The live on-drone estimate [(t_ms, (x, y, z))] from the fixedFrequency block.

    Used to quantify replay-vs-live divergence (spec F1). Requires
    stateEstimate.x/y/z in the uSD fixedFrequency config.
    """
    ff = log_data.get('fixedFrequency')
    if not ff or 'timestamp' not in ff:
        return []
    needed = ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z')
    if not all(k in ff for k in needed):
        return []
    return [
        (float(ff['timestamp'][i]),
         (float(ff['stateEstimate.x'][i]),
          float(ff['stateEstimate.y'][i]),
          float(ff['stateEstimate.z'][i])))
        for i in range(len(ff['timestamp']))
    ]


def apply_policy(policy, groups):
    """Map candidate groups to 'estTDOA' samples using a selection policy.

    The policy is reset first, so one policy instance per replay run.
    All measurements of a group get the group's packet time (min event
    timestamp, see build_candidate_groups).
    """
    policy.reset()
    samples = []
    for group in groups:
        for m in policy.select(group):
            samples.append(('estTDOA', {
                'idA': m['idA'],
                'idB': m['idB'],
                'distanceDiff': m['distanceDiff'],
                'timestamp': group['t_ms'],
            }))
    return samples


def merge_samples(*sample_lists):
    """Merge sample streams into one, ordered by timestamp.

    The sort is stable: samples with equal timestamps keep the relative order
    of their input streams (spec F2).
    """
    merged = [s for lst in sample_lists for s in lst]
    merged.sort(key=lambda s: s[1]['timestamp'])
    return merged


def replay(anchor_positions, imu_samples, tdoa_samples, params=None):
    """Run samples through the real firmware Kalman core.

    Args:
        anchor_positions: dict anchor id -> cffirmware.vec3_s (see
            loco_utils.read_loco_anchor_positions).
        imu_samples: from extract_imu_samples.
        tdoa_samples: from apply_policy (or synthetic, same shape).
        params: optional dict of tuning parameters. Supported:
            'tdoa_std' (float, default 0.15): TDoA measurement std dev [m].

    Returns:
        [(t_ms, (x, y, z))] trajectory, one entry per 1 kHz iteration.
    """
    from bindings.util.estimator_kalman_emulator import EstimatorKalmanEmulator

    params = params or {}
    samples = merge_samples(imu_samples, tdoa_samples)
    if not samples:
        return []

    emulator = EstimatorKalmanEmulator(anchor_positions)
    emulator.TDOA_ENGINE_MEASUREMENT_NOISE_STD = params.get('tdoa_std', DEFAULT_TDOA_STD)

    trajectory = []
    while len(samples):
        now_ms, state = emulator.run_one_1khz_iteration(samples)
        trajectory.append((now_ms, (state.position.x, state.position.y, state.position.z)))
    return trajectory
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python3 -m pytest test_python/test_tdoa_replay.py test_python/test_tdoa_selection.py -v`
Expected: PASS — all tests except `test_ground_truth_interpolation_and_scoring`, which still fails until Step 5 makes `tools.usdlog.replay_tdoa` importable without the bindings.

- [ ] **Step 5: Make replay_tdoa.py a thin consumer**

In `tools/usdlog/replay_tdoa.py`:

1. Replace the import block's seam imports. `loco_utils` imports `cffirmware` at module level, so it moves into `main()` — the CLI module itself must stay importable without the bindings (its pure helpers are unit tested):

```python
import tools.usdlog.cfusdlog as cfusdlog
from bindings.util.tdoa_replay import (
    apply_policy,
    extract_imu_samples,
    extract_state_estimate,
    replay,
)
from bindings.util.tdoa_selection import (
    build_candidate_groups,
    make_policy,
    verify_baseline_reconstruction,
)
```

and in `main()`, replace the `anchor_positions = read_loco_anchor_positions(args.anchors)` line with:

```python
    # Imported here: loco_utils needs the cffirmware bindings, which the pure
    # helpers in this module (tested without bindings) must not depend on
    from bindings.util.loco_utils import read_loco_anchor_positions
    anchor_positions = read_loco_anchor_positions(args.anchors)
```

2. Delete the module-level `extract_imu_samples` function (moved to the seam).
3. Replace the body of `run_policy` with a seam call:

```python
def run_policy(policy, anchor_positions, imu_samples, groups, tdoa_std):
    """Run the Kalman replay for one selection policy. Returns [(t_ms, (x,y,z))]."""
    tdoa_samples = apply_policy(policy, groups)
    return replay(anchor_positions, imu_samples, tdoa_samples, {'tdoa_std': tdoa_std})
```

4. In `main()`, after the `gt = extract_ground_truth(log_data)` line, add:

```python
    live = extract_state_estimate(log_data)
```

5. In `main()`, after the "ground-truth samples" print line, add the F1 reconstruction report:

```python
    check = verify_baseline_reconstruction(log_data)
    if check['n_est_tdoa']:
        status = 'OK' if (check['n_mismatched'] == 0
                          and check['n_baseline'] == check['n_est_tdoa']) else 'MISMATCH'
        print(f"  baseline reconstruction:    {status} "
              f"({check['n_compared']} compared, {check['n_mismatched']} mismatched, "
              f"{check['n_selectionless']} groups without selection)")
```

6. In the per-policy loop in `main()`, right after the `metrics, _ = score_trajectory(...)` line, add the replay-vs-live comparison:

```python
        if name == 'baseline' and live:
            live_metrics, _ = score_trajectory(trajectory, live, args.flyaway_threshold)
            if live_metrics['n']:
                print(f'{"":12} baseline vs live stateEstimate: '
                      f'rms {live_metrics["rms"]:.3f} m, max {live_metrics["max"]:.3f} m '
                      f'(replay-vs-onboard divergence, spec F1)')
```

- [ ] **Step 6: Verify the CLI still imports and shows help**

Run: `python3 -m tools.usdlog.replay_tdoa --help`
Expected: usage text prints, exit 0 (no cffirmware import at module level — `replay` imports it lazily, `loco_utils` is imported inside `main()`, and `estimator_kalman_emulator` must NOT be imported at the top of replay_tdoa.py anymore).

Run: `python3 -m pytest test_python/test_tdoa_replay.py test_python/test_tdoa_selection.py -v`
Expected: PASS (17 tests, including `test_ground_truth_interpolation_and_scoring`).

- [ ] **Step 7: Commit**

```bash
git add bindings/util/tdoa_replay.py tools/usdlog/replay_tdoa.py test_python/test_tdoa_replay.py
git commit -m "refactor: extract replay seam into bindings/util/tdoa_replay.py (spec S4)

replay_tdoa.py becomes a thin CLI consumer; adds baseline-reconstruction
report and replay-vs-live stateEstimate divergence (spec F1) to the output."
```

---

### Task 9: Replay smoke test through the real cffirmware bindings

Prove the seam end-to-end: a synthetic scenario through the real Kalman core converges to the true position (spec Section 5.3, pattern of `test_python/test_kalman_core.py`).

**Files:**
- Test: `test_python/test_tdoa_replay_smoke.py` (new — separate file so `pytest.importorskip` does not skip the bindings-free tests of Task 8)

**Interfaces:**
- Consumes: `replay()` from Task 8; `cffirmware.vec3_s` (same anchor-position shape as `loco_utils.read_loco_anchor_positions`).
- Produces: nothing new (verified guarantee only).

- [ ] **Step 1: Build the bindings**

Run: `make bindings_python`
Expected: completes; `build/cffirmware.py` exists.

- [ ] **Step 2: Write the smoke test**

Create `test_python/test_tdoa_replay_smoke.py`:

```python
"""End-to-end smoke test of the replay seam through the real cffirmware
bindings: a synthetic, noise-free scenario must converge to the true position.

Requires the SWIG bindings: make bindings_python, then run with
PYTHONPATH=build.
"""

import math

import pytest

cffirmware = pytest.importorskip('cffirmware')

from bindings.util.tdoa_replay import replay

# 8 anchors on the corners of a 4 x 4 x 3 m box; tag stationary inside it
ANCHOR_COORDS = {
    0: (0.0, 0.0, 0.0), 1: (4.0, 0.0, 0.0), 2: (0.0, 4.0, 0.0), 3: (4.0, 4.0, 0.0),
    4: (0.0, 0.0, 3.0), 5: (4.0, 0.0, 3.0), 6: (0.0, 4.0, 3.0), 7: (4.0, 4.0, 3.0),
}
TRUE_POS = (2.0, 2.0, 1.0)
DURATION_MS = 5000
POSITION_TOLERANCE_M = 0.5


def _anchor_positions():
    result = {}
    for anchor_id, (x, y, z) in ANCHOR_COORDS.items():
        p = cffirmware.vec3_s()
        p.x, p.y, p.z = x, y, z
        result[anchor_id] = p
    return result


def _dist(a, b):
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _synthetic_imu():
    # Stationary tag: 1 g on the z axis (acc is logged in g), zero rotation
    samples = []
    for t in range(DURATION_MS):
        ts = float(t)
        samples.append(('estAcceleration',
                        {'timestamp': ts, 'acc.x': 0.0, 'acc.y': 0.0, 'acc.z': 1.0}))
        samples.append(('estGyroscope',
                        {'timestamp': ts, 'gyro.x': 0.0, 'gyro.y': 0.0, 'gyro.z': 0.0}))
    return samples


def _synthetic_tdoa():
    # 100 Hz, rotating over anchor pairs. Firmware sign convention
    # (see mm_tdoa.c): distanceDiff = d(idB) - d(idA).
    samples = []
    for k, t in enumerate(range(0, DURATION_MS, 10)):
        id_a = k % 8
        id_b = (k + 1) % 8
        dd = _dist(ANCHOR_COORDS[id_b], TRUE_POS) - _dist(ANCHOR_COORDS[id_a], TRUE_POS)
        samples.append(('estTDOA',
                        {'timestamp': float(t), 'idA': id_a, 'idB': id_b,
                         'distanceDiff': dd}))
    return samples


def test_replay_seam_converges_to_the_true_position():
    trajectory = replay(_anchor_positions(), _synthetic_imu(), _synthetic_tdoa(),
                        {'tdoa_std': 0.15})

    assert len(trajectory) > 4000
    times = [t for t, _ in trajectory]
    assert times == sorted(times)

    _, final_pos = trajectory[-1]
    assert _dist(final_pos, TRUE_POS) < POSITION_TOLERANCE_M
```

- [ ] **Step 3: Run the smoke test**

Run: `PYTHONPATH=build python3 -m pytest test_python/test_tdoa_replay_smoke.py -v`
Expected: PASS. If the convergence assertion fails, print the final position and error; a systematically mirrored position indicates a distanceDiff sign error in the synthetic data (the convention is `d(idB) - d(idA)`, verified against `mm_tdoa.c:61`) — fix the test data, not the seam, unless the seam demonstrably swaps idA/idB.

- [ ] **Step 4: Run the whole Python test suite**

Run: `PYTHONPATH=build python3 -m pytest test_python -v`
Expected: all pass (pre-existing tests plus the new ones).

- [ ] **Step 5: Commit**

```bash
git add test_python/test_tdoa_replay_smoke.py
git commit -m "test: end-to-end replay smoke test through real cffirmware bindings"
```

---

### Task 10: Docs — README update + hardware validation checklist

Bring `tools/usdlog/README_tdoa_candidates.md` in line with the implemented behavior (enable-flag semantics, automatic drop check) and add the hardware validation checklist that Task 11 executes (spec Section 5.4 deliverable).

**Files:**
- Modify: `tools/usdlog/README_tdoa_candidates.md`

**Interfaces:**
- Consumes: behavior implemented in Tasks 2–8.
- Produces: the checklist Task 11 follows.

- [ ] **Step 1: Fix the `logCand` documentation**

In `README_tdoa_candidates.md`, "What was added" section, replace the param bullet:

```markdown
- Param `tdoaEngine.logCand` (default `0` = off). Set to any non-zero value
  (use `1`) to log **all** valid candidate pairs of every processed packet.
  There is no partial mode: the log is always a complete record of what the
  matcher saw, so the selected pair is always included.
```

In "Per-run loop" step 1, change `param set tdoaEngine.logCand 16` to `param set tdoaEngine.logCand 1`.

- [ ] **Step 2: Rewrite the drop-check section**

Replace the whole "Watch for dropped uSD events (important)" section with:

```markdown
## Dropped uSD events are checked automatically

The uSD ring buffer silently drops events if the SD write task can't keep up,
which breaks lossless replay. The firmware counts `usd.eventsRequested` vs
`usd.eventsWritten` (log variables, reset at each log start), and
`read_usd_log.sh` compares them after stopping the log on every readback:
it fails loudly (exit 1) if any events were dropped.

If drops are reported:
- increase the ring buffer size (line 2 of `config.txt`), or
- switch to `config_tdoa_candidates_lean.txt`.

Do not use a log with drops for replay conclusions; the holes are unflagged.
```

- [ ] **Step 3: Add the hardware validation checklist section**

Append before "Notes / limitations":

```markdown
## Hardware validation checklist

Run this end-to-end check whenever the capture/readback/replay chain changes.
Collected logs stay local (do not commit them).

1. **Build** with the ground-truth config (see One-time setup) and **flash**.
2. **Arm logging**: `param set tdoaEngine.logCand 1`, `param set usd.logging 1`.
3. **Collect**: disconnect the radio; move the drone by hand (or fly, with
   explicit permission) for at least 60 s inside loco + Lighthouse coverage.
4. **Read back**: `tools/usdlog/read_usd_log.sh <uri> run.bin`
   - PASS requires: the automatic drop check prints
     `OK: no dropped events` (R1) and the file decodes
     (`replay_tdoa.py` reads it) with a plausible size (R2).
5. **Verify capture faithfulness**:
   `python3 -m tools.usdlog.replay_tdoa run.bin --anchors anchors.yaml --policies baseline`
   - PASS requires: `baseline reconstruction: OK (... 0 mismatched ...)` —
     the selected candidates reproduce the logged `estTDOA` stream exactly
     (F1, measurement level).
6. **Quantify replay fidelity**: the same command prints
   `baseline vs live stateEstimate: rms ... m, max ... m`.
   - Record the numbers here. Documented tolerance (F1, trajectory level):
     **rms ≤ TBD m** — fill in after the first validated run and treat
     regressions beyond it as failures.

Latest validated run: _(date, firmware commit, rms/max)_ — not yet performed.
```

Note: the `TBD` in step 6 is intentional and user-facing — it is filled with a real number in Task 11 and must not survive past it.

- [ ] **Step 4: Proofread the rest of the README**

Search the README for remaining stale claims: any mention of "rate cap", "up to N candidates", or manual `usd.eventsRequested` comparison must be gone. The "Notes / limitations" entry about `--tdoa-std` stays.

- [ ] **Step 5: Commit**

```bash
git add tools/usdlog/README_tdoa_candidates.md
git commit -m "docs: enable-flag semantics, automatic drop check, hardware validation checklist"
```

---

### Task 11: Hardware end-to-end validation (requires user + rig)

Execute the Task 10 checklist on the real rig. This task needs the user present: they place the drone in loco + Lighthouse coverage, and any flight needs their explicit go-ahead. Use the `crazyflie-dev` skill for flashing and hardware interaction.

**Files:**
- Modify: `tools/usdlog/README_tdoa_candidates.md` (fill in the F1 tolerance + validated-run line)

**Interfaces:**
- Consumes: everything from Tasks 1–10; `crazyflie-dev` skill (build/flash/param/log); the user's rig.
- Produces: R1/R2/F1 validated on hardware; the documented F1 trajectory tolerance.

- [ ] **Step 1: Build the validation firmware**

```bash
make cf2_defconfig
./scripts/kconfig/merge_config.sh -m -O build build/.config configs/tdoa3_lh_groundtruth.conf
make olddefconfig
make -j$(nproc)
```

Expected: `build/cf2.bin` produced.

- [ ] **Step 2: Flash and arm (coordinate with the user)**

Invoke the `crazyflie-dev` skill for the hardware loop. Flash `build/cf2.bin`, then:

```bash
cfcli -u <uri> param set tdoaEngine.logCand 1
cfcli -u <uri> param set usd.logging 1
```

Confirm with the user that the drone is in loco + Lighthouse coverage and the SD card carries `config_tdoa_candidates.txt` as `config.txt`.

- [ ] **Step 3: Collect a run**

Ask the user to move the drone by hand for ≥ 60 s (flight only if the user explicitly offers it). Keep the radio disconnected during collection.

- [ ] **Step 4: Read back and verify R1/R2**

```bash
tools/usdlog/read_usd_log.sh <uri> run_validation.bin
```

Expected: `OK: no dropped events (...)` and a saved file of plausible size. If the drop check errors out or warns "could not read", debug before proceeding (Task 5 firmware actually flashed? `cfcli log print` CSV column order as assumed in Task 6? Fix the script parsing if the column order differs). If drops are real, increase the ring buffer per the README and re-collect.

- [ ] **Step 5: Verify F1 and record the tolerance**

```bash
python3 -m tools.usdlog.replay_tdoa run_validation.bin --anchors anchors.yaml --policies baseline
```

Expected: `baseline reconstruction: OK (... 0 mismatched ...)`. Then read the `baseline vs live stateEstimate` rms/max numbers, and with the user agree on the documented tolerance (suggestion: round the observed rms up generously, e.g. observed 0.08 m → document 0.15 m). Replace the `TBD` in the README checklist and fill in the "Latest validated run" line (date, firmware commit, rms/max). Do **not** commit `run_validation.bin`.

- [ ] **Step 6: Commit and wrap up**

```bash
git add tools/usdlog/README_tdoa_candidates.md
git commit -m "docs: record hardware-validated F1 tolerance and validation run"
```

Then run the full verification sweep one last time: `rake unit` and `PYTHONPATH=build python3 -m pytest test_python -v`. Expected: all green. This completes the plan; use superpowers:finishing-a-development-branch to decide integration.
