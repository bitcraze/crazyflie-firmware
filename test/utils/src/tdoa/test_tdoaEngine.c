/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * test_tdoaEngine.c - Unit tests for the anchor pair distance ratio filter in matchRandomAnchor()
 */

// File under test
#include "tdoaEngine.h"

// tdoaEngine.c depends on tdoaStorage.c and tdoaStats.c - include the headers directly so
// the test build system compiles and links them in (it only scans direct includes).
#include "tdoaStorage.h"
#include "tdoaStats.h"
#include "statsCnt.h"

#include "unity.h"

#include "physicalConstants.h"
#include "mock_clockCorrectionEngine.h"

// matchRandomAnchor() is TESTABLE_STATIC in tdoaEngine.c, which drops "static" under
// UNIT_TEST_MODE - declare it here to call it directly.
bool matchRandomAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq, double* distanceDiff);

#define NOW_MS 10000
#define OWN_ANCHOR_ID 5
#define CANDIDATE_A_ID 11
#define CANDIDATE_B_ID 17

// Anchors are placed 10 m apart, own anchor at the origin
#define ANCHOR_TO_ANCHOR_DISTANCE 10.0f

// Chosen so that distanceDiff (in "meters") comes out numerically equal to (RX_TIMESTAMP - tof),
// see fixtureRegisterCandidate()
#define LOCODECK_TS_FREQ_FOR_TEST SPEED_OF_LIGHT
#define RX_TIMESTAMP 100

static tdoaEngineState_t engineState;
static tdoaAnchorContext_t ownAnchorCtx;

static void noopSendTdoaToEstimator(tdoaMeasurement_t* tdoaMeasurement);
static void fixtureRegisterCandidate(const uint8_t candidateId, const int64_t tof, const bool withPosition);

void setUp(void) {
  tdoaEngineInit(&engineState, NOW_MS, noopSendTdoaToEstimator, LOCODECK_TS_FREQ_FOR_TEST, TdoaEngineMatchingAlgorithmRandom);
  clockCorrectionEngineGet_IgnoreAndReturn(1.0);

  tdoaStorageGetCreateAnchorCtx(engineState.anchorInfoArray, OWN_ANCHOR_ID, NOW_MS, &ownAnchorCtx);
  tdoaStorageSetAnchorPosition(&ownAnchorCtx, 0.0f, 0.0f, 0.0f);
}


void testThatDistanceDiffIsReturnedForTheChosenCandidate() {
  // Fixture
  // distanceDiff = RX_TIMESTAMP - tof = 100 - 95 = 5, ratio = 5 / 10 = 0.5, below the default limit
  fixtureRegisterCandidate(CANDIDATE_A_ID, 95, true);

  tdoaAnchorContext_t otherAnchorCtx;
  double distanceDiff = 0.0;

  // Test
  bool actual = matchRandomAnchor(&engineState, &otherAnchorCtx, &ownAnchorCtx, false, 0, 0, RX_TIMESTAMP, LOCODECK_TS_FREQ_FOR_TEST, &distanceDiff);

  // Assert
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_EQUAL_UINT8(CANDIDATE_A_ID, tdoaStorageGetId(&otherAnchorCtx));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, (float)distanceDiff);
}


void testThatACandidateWithRatioAtOrAboveTheLimitIsSkippedInFavorOfTheNextCandidate() {
  // Fixture
  // Registered first, tried second (offset-based rotation): distanceDiff = 5, ratio = 0.5 - passes
  fixtureRegisterCandidate(CANDIDATE_B_ID, 95, true);
  // Registered second, tried first: distanceDiff = 9, ratio = 0.9 - at/above the default 0.85 limit, rejected
  fixtureRegisterCandidate(CANDIDATE_A_ID, 91, true);

  tdoaAnchorContext_t otherAnchorCtx;
  double distanceDiff = 0.0;

  // Test
  bool actual = matchRandomAnchor(&engineState, &otherAnchorCtx, &ownAnchorCtx, false, 0, 0, RX_TIMESTAMP, LOCODECK_TS_FREQ_FOR_TEST, &distanceDiff);

  // Assert
  // The bad-geometry candidate (A) is skipped, the good one (B) is chosen instead
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_EQUAL_UINT8(CANDIDATE_B_ID, tdoaStorageGetId(&otherAnchorCtx));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, (float)distanceDiff);
}


void testThatACandidateWithAMissingAnchorPositionIsSkippedInFavorOfTheNextCandidate() {
  // Fixture
  // Registered first, tried second: has a position, distanceDiff = 5, ratio = 0.5 - passes
  fixtureRegisterCandidate(CANDIDATE_B_ID, 95, true);
  // Registered second, tried first: no position set, so the ratio can't be computed - rejected
  fixtureRegisterCandidate(CANDIDATE_A_ID, 1, false);

  tdoaAnchorContext_t otherAnchorCtx;
  double distanceDiff = 0.0;

  // Test
  bool actual = matchRandomAnchor(&engineState, &otherAnchorCtx, &ownAnchorCtx, false, 0, 0, RX_TIMESTAMP, LOCODECK_TS_FREQ_FOR_TEST, &distanceDiff);

  // Assert
  TEST_ASSERT_TRUE(actual);
  TEST_ASSERT_EQUAL_UINT8(CANDIDATE_B_ID, tdoaStorageGetId(&otherAnchorCtx));
  TEST_ASSERT_EQUAL_FLOAT(5.0f, (float)distanceDiff);
}


void testThatFalseIsReturnedWhenAllCandidatesFail() {
  // Fixture
  // Ratio 0.9 - at/above the limit, rejected
  fixtureRegisterCandidate(CANDIDATE_A_ID, 91, true);
  // No position - can't compute a ratio, rejected
  fixtureRegisterCandidate(CANDIDATE_B_ID, 1, false);

  tdoaAnchorContext_t otherAnchorCtx;
  double distanceDiff = 0.0;

  // Test
  bool actual = matchRandomAnchor(&engineState, &otherAnchorCtx, &ownAnchorCtx, false, 0, 0, RX_TIMESTAMP, LOCODECK_TS_FREQ_FOR_TEST, &distanceDiff);

  // Assert
  TEST_ASSERT_FALSE(actual);
  TEST_ASSERT_NULL(otherAnchorCtx.anchorInfo);
}


// Helpers ///////////////

static void noopSendTdoaToEstimator(tdoaMeasurement_t* tdoaMeasurement) {
  // Nothing to do
}

// Registers a candidate anchor that is visible to ownAnchorCtx with a matching sequence number
// and time of flight, so it passes the pre-existing matching checks in matchRandomAnchor().
// The candidate is placed at (ANCHOR_TO_ANCHOR_DISTANCE, 0, 0), ownAnchorCtx is at the origin.
static void fixtureRegisterCandidate(const uint8_t candidateId, const int64_t tof, const bool withPosition) {
  const uint8_t seqNr = candidateId;

  tdoaAnchorContext_t candidateCtx;
  tdoaStorageGetCreateAnchorCtx(engineState.anchorInfoArray, candidateId, NOW_MS, &candidateCtx);
  tdoaStorageSetRxTxData(&candidateCtx, 0, 0, seqNr);
  if (withPosition) {
    tdoaStorageSetAnchorPosition(&candidateCtx, ANCHOR_TO_ANCHOR_DISTANCE, 0.0f, 0.0f);
  }

  tdoaStorageSetRemoteRxTime(&ownAnchorCtx, candidateId, 0, seqNr);
  tdoaStorageSetRemoteTimeOfFlight(&ownAnchorCtx, candidateId, tof);
}
