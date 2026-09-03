/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * tdoaEngine.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with tdoaEngine.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*
Implementation of LPS TDoA Tag functionality

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-20 anchors are visible
in every point. The tag is attached to a physical object and the expected
velocity is a few m/s, this means that anchors are within range for a time
period of seconds.

The implementation must handle
1. An infinite number of anchors, where around 20 are visible at one time
2. Any anchor ids
3. Dynamically changing visibility of anchors over time
4. Random TX times from anchors with possible packet collisions and packet loss

*/

#include <string.h>

#define DEBUG_MODULE "TDOA_ENGINE"
#include "debug.h"

#include "tdoaEngine.h"
#include "tdoaStats.h"
#include "clockCorrectionEngine.h"
#include "physicalConstants.h"
#include "eventtrigger.h"
#include "test_support.h"
#include "param.h"
#include "usec_time.h"

// Emitted (when engineState->candidateLogEnable is non-zero) for every valid
// anchor-pair candidate of a processed packet, before the matching algorithm
// collapses the candidates to a single pair. Together the events with the same
// "group" describe all pairs that were available for one received packet, which
// allows offline replay and A/B testing of the anchor-pair selection policy.
//   group       : per-packet counter, groups candidates of the same packet
//   idA         : id of the anchor that transmitted the processed packet
//   idB         : id of the candidate (remote) anchor
//   distanceDiff: the TDoA measurement [m] for the (idA, idB) pair
//   isSelected  : 1 if the live matching algorithm picked this pair, else 0
EVENTTRIGGER(estTdoaCand, uint32, group, uint8, idA, uint8, idB, float, distanceDiff, uint8, isSelected)

#define TDOA_ENGINE_DEFAULT_DISTANCE_RATIO_LIMIT 0.85f

static float distanceRatioLimit = TDOA_ENGINE_DEFAULT_DISTANCE_RATIO_LIMIT;

void tdoaEngineInit(tdoaEngineState_t* engineState, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm) {
  tdoaStorageInitialize(engineState->anchorInfoArray);
  tdoaStatsInit(&engineState->stats, now_ms);
  engineState->sendTdoaToEstimator = sendTdoaToEstimator;
  engineState->locodeckTsFreq = locodeckTsFreq;
  engineState->matchingAlgorithm = matchingAlgorithm;
#ifdef CONFIG_DECK_LOCO_TDOA_RATE_LIMIT
  engineState->maxRateHz = TDOA_ENGINE_DEFAULT_MAX_RATE_HZ;
  engineState->lastForwardedTime_us = 0;
#endif

  engineState->matching.offset = 0;

  // Normally off: candidate logging costs uSD event bandwidth and is only
  // wanted for capture runs. A build dedicated to capture can flip the initial
  // value so that no link is needed to arm it (see the Kconfig help).
#ifdef CONFIG_DECK_LOCO_TDOA3_LOG_CANDIDATES
  engineState->candidateLogEnable = 1;
#else
  engineState->candidateLogEnable = 0;
#endif
  engineState->candidateLogGroup = 0;
}

static void enqueueTDOA(const tdoaAnchorContext_t* anchorACtx, const tdoaAnchorContext_t* anchorBCtx, double distanceDiff, tdoaEngineState_t* engineState) {
  tdoaStats_t* stats = &engineState->stats;

  tdoaMeasurement_t tdoa = {
    .stdDev = TDOA_ENGINE_MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff
  };

  if (tdoaStorageGetAnchorPosition(anchorACtx, &tdoa.anchorPositions[0]) && tdoaStorageGetAnchorPosition(anchorBCtx, &tdoa.anchorPositions[1])) {
    STATS_CNT_RATE_EVENT(&stats->packetsToEstimator);

    uint8_t idA = tdoaStorageGetId(anchorACtx);
    uint8_t idB = tdoaStorageGetId(anchorBCtx);
    if (idA == stats->anchorId && idB == stats->remoteAnchorId) {
      stats->tdoa = distanceDiff;
    }
    if (idB == stats->anchorId && idA == stats->remoteAnchorId) {
      stats->tdoa = -distanceDiff;
    }
    tdoa.anchorIds[0] = idA;
    tdoa.anchorIds[1] = idB;

    engineState->sendTdoaToEstimator(&tdoa);
  }
}

static bool updateClockCorrection(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, tdoaStats_t* stats) {
  bool sampleIsReliable = false;

  const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);

  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    double clockCorrectionCandidate = clockCorrectionEngineCalculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP);
    sampleIsReliable = clockCorrectionEngineUpdate(tdoaStorageGetClockCorrectionStorage(anchorCtx), clockCorrectionCandidate);

    if (sampleIsReliable){
      if (tdoaStorageGetId(anchorCtx) == stats->anchorId) {
        stats->clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        STATS_CNT_RATE_EVENT(&stats->clockCorrectionCount);
      }
    }
  }

  return sampleIsReliable;
}

static int64_t calcTDoA(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx);

  const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetRemoteTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

  const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx);

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + tdoaEngineTruncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_T =  tdoaEngineTruncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T;
}

static double calcDistanceDiff(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq) {
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  return SPEED_OF_LIGHT * tdoa / locodeckTsFreq;
}

// Emit an estTdoaCand event for every valid anchor-pair candidate of this
// packet. A candidate (idA, idB) is considered valid using the same criteria
// as the matching algorithms: idA must have a known time-of-flight to idB and
// the cached sequence number must match. All valid candidates are always
// emitted (no truncation) so that the log is a complete record; the pair
// chosen by the live matching algorithm is flagged with isSelected.
static void logTdoaCandidates(tdoaEngineState_t* engineState, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const uint8_t selectedId) {
  int remoteCount = 0;
  uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
  uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, seqNr, id);

  const uint32_t now_ms = anchorCtx->currentTime_ms;
  const uint8_t idA = tdoaStorageGetId(anchorCtx);
  const uint32_t group = engineState->candidateLogGroup++;

  for (int index = 0; index < remoteCount; index++) {
    const uint8_t candidateAnchorId = id[index];

    if (!tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
      continue;
    }

    // Non-creating accessor: observing the candidates must never mutate the
    // anchor storage (a GetCreate miss would evict the oldest stored anchor)
    tdoaAnchorContext_t otherAnchorCtx;
    if (!tdoaStorageGetAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, &otherAnchorCtx)) {
      continue;
    }

    if (seqNr[index] != tdoaStorageGetSeqNr(&otherAnchorCtx)) {
      continue;
    }

    const double distanceDiff = calcDistanceDiff(&otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->locodeckTsFreq);

    eventTrigger_estTdoaCand_payload.group = group;
    eventTrigger_estTdoaCand_payload.idA = idA;
    eventTrigger_estTdoaCand_payload.idB = candidateAnchorId;
    eventTrigger_estTdoaCand_payload.distanceDiff = (float)distanceDiff;
    eventTrigger_estTdoaCand_payload.isSelected = (candidateAnchorId == selectedId) ? 1 : 0;
    eventTrigger(&eventTrigger_estTdoaCand);
  }
}

static float sq(float a) { return a * a; }

static float distanceBetweenAnchorsSquared(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}

static bool isGeometryGoodEnough(const tdoaAnchorContext_t* anchorCtx, const tdoaAnchorContext_t* otherAnchorCtx, const double candidateDistanceDiff) {
  // The measurement is h(p) = |p - A1| - |p - A2|. Its gradient wrt the tag position p is
  // grad(h) = u1 - u2, where u1, u2 are the unit vectors from each anchor towards the tag (i.e.
  // the lines of sight). The true measurement sensitivity is:
  //   |grad(h)| = |u1 - u2| = 2 * sin(theta / 2)
  // with theta the angle between those two lines of sight.
  //
  // A candidate is rejected if the ratio between the measured distance diff and the distance
  // between the anchors is at or above distanceRatioLimit - this indicates bad pair geometry,
  // and inversed approximates the measurement sensitivity.
  
  point_t anchorPosition;
  point_t otherAnchorPosition;

  // Return false (candidate rejected) if either anchor position is unknown.
  if (!tdoaStorageGetAnchorPosition(anchorCtx, &anchorPosition) || !tdoaStorageGetAnchorPosition(otherAnchorCtx, &otherAnchorPosition)) {  
    return false;
  }

  const float anchorDistanceSquared = distanceBetweenAnchorsSquared(&anchorPosition, &otherAnchorPosition);
  if (anchorDistanceSquared <= 0.0f) {
    // Anchors at (near) identical positions - geometry is degenerate, reject to avoid division by zero
    return false;
  }

  const float distanceDiffSquared = sq((float)candidateDistanceDiff);
  const float ratioSquared = distanceDiffSquared / anchorDistanceSquared;
  return ratioSquared < sq(distanceRatioLimit);
}

TESTABLE_STATIC bool matchRandomAnchor(tdoaEngineState_t* engineState,
                                       tdoaAnchorContext_t* otherAnchorCtx,
                                       const tdoaAnchorContext_t* anchorCtx,
                                       const bool doExcludeId,
                                       const uint8_t excludedId,
                                       const int64_t txAn_in_cl_An,
                                       const int64_t rxAn_by_T_in_cl_T,
                                       const double locodeckTsFreq,
                                       double* distanceDiff) {
  engineState->matching.offset++;
  int remoteCount = 0;
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

  uint32_t now_ms = anchorCtx->currentTime_ms;

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
  // Reject a candidate if geometry is not good enough.
  for (int i = engineState->matching.offset; i < (remoteCount + engineState->matching.offset); i++) {
    uint8_t index = i % remoteCount;
    const uint8_t candidateAnchorId = engineState->matching.id[index];
    if (!doExcludeId || (excludedId != candidateAnchorId)) {
      if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
        if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx) && tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          const double candidateDistanceDiff = calcDistanceDiff(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, locodeckTsFreq);
          if (!isGeometryGoodEnough(anchorCtx, otherAnchorCtx, candidateDistanceDiff)) {
            STATS_CNT_RATE_EVENT(&engineState->stats.geometryRejected);
            continue;
          }

          *distanceDiff = candidateDistanceDiff;
          return true;
        }
      }
    }
  }

  otherAnchorCtx->anchorInfo = 0;
  return false;
}

static bool matchYoungestAnchor(tdoaEngineState_t* engineState,
                                tdoaAnchorContext_t* otherAnchorCtx,
                                const tdoaAnchorContext_t* anchorCtx,
                                const bool doExcludeId,
                                const uint8_t excludedId,
                                const int64_t txAn_in_cl_An,
                                const int64_t rxAn_by_T_in_cl_T,
                                const double locodeckTsFreq,
                                double* distanceDiff) {
    int remoteCount = 0;
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

    uint32_t now_ms = anchorCtx->currentTime_ms;
    uint32_t youngestUpdateTime = 0;
    int bestId = -1;

    for (int index = 0; index < remoteCount; index++) {
      const uint8_t candidateAnchorId = engineState->matching.id[index];
      if (!doExcludeId || (excludedId != candidateAnchorId)) {
        if (tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
            uint32_t updateTime = tdoaStorageGetLastUpdateTime(otherAnchorCtx);
            if (updateTime > youngestUpdateTime) {
              if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx)) {
                youngestUpdateTime = updateTime;
                bestId = candidateAnchorId;
              }
            }
          }
        }
      }
    }

    if (bestId >= 0) {
      tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, bestId, now_ms, otherAnchorCtx);
      *distanceDiff = calcDistanceDiff(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, locodeckTsFreq);
      return true;
    }

    otherAnchorCtx->anchorInfo = 0;
    return false;
}

static bool findSuitableAnchor(tdoaEngineState_t* engineState,
                               tdoaAnchorContext_t* otherAnchorCtx,
                               const tdoaAnchorContext_t* anchorCtx,
                               const bool doExcludeId,
                               const uint8_t excludedId,
                               const int64_t txAn_in_cl_An,
                               const int64_t rxAn_by_T_in_cl_T,
                               const double locodeckTsFreq,
                               double* distanceDiff) {
  bool result = false;

  if (tdoaStorageGetClockCorrection(anchorCtx) > 0.0) {
    switch(engineState->matchingAlgorithm) {
      case TdoaEngineMatchingAlgorithmRandom:
        result = matchRandomAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId, txAn_in_cl_An, rxAn_by_T_in_cl_T, locodeckTsFreq, distanceDiff);
        break;

      case TdoaEngineMatchingAlgorithmYoungest:
        result = matchYoungestAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId, txAn_in_cl_An, rxAn_by_T_in_cl_T, locodeckTsFreq, distanceDiff);
        break;

      default:
        // Do nothing
        break;
    }
  }

  return result;
}

void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx)) {
    STATS_CNT_RATE_EVENT(&engineState->stats.contextHitCount);
  } else {
    STATS_CNT_RATE_EVENT(&engineState->stats.contextMissCount);
  }
}

#ifdef CONFIG_DECK_LOCO_TDOA_RATE_LIMIT
// Rate-limits the aggregate stream of measurements forwarded to the estimator, across all anchors.
// maxRateHz <= 0 disables the limit. The shared timer is reset as soon as a packet is let through,
// even if matching later fails to produce a measurement.
static bool isForwardRateLimited(tdoaEngineState_t* engineState, const float maxRateHz) {
  if (maxRateHz <= 0.0f) {
    return false;
  }

  const uint64_t now_us = usecTimestamp();
  const uint64_t minPeriod_us = (uint64_t)(1000000.0f / maxRateHz);
  if (engineState->lastForwardedTime_us != 0 && (now_us - engineState->lastForwardedTime_us) < minPeriod_us) {
    return true;
  } else {
    if (now_us > engineState->lastForwardedTime_us + 10*minPeriod_us) { // Long time since last measurement, or no measurement has yet been forwarded
      // Reset window
      engineState->lastForwardedTime_us = now_us;
    }
    else {
      engineState->lastForwardedTime_us += minPeriod_us;
    }
    return false;
  }
}
#endif

void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  tdoaEngineProcessPacketFiltered(engineState, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, false, 0);
}

bool tdoaEngineProcessPacketFiltered(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const bool doExcludeId, const uint8_t excludedId) {
  bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
  if (timeIsGood) {
    STATS_CNT_RATE_EVENT(&engineState->stats.timeIsGood);

    tdoaAnchorContext_t otherAnchorCtx;
    double tdoaDistDiff = 0.0;

    bool suitableAnchorFound = findSuitableAnchor(engineState, &otherAnchorCtx, anchorCtx, doExcludeId, excludedId, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->locodeckTsFreq, &tdoaDistDiff);

    // Log all candidate pairs (not just the selected one) when enabled. Gated on
    // a valid clock correction, matching the condition under which the matching
    // algorithm runs, so that the logged distanceDiff values are meaningful.
    if (engineState->candidateLogEnable > 0 && tdoaStorageGetClockCorrection(anchorCtx) > 0.0) {
      const uint8_t selectedId = suitableAnchorFound ? tdoaStorageGetId(&otherAnchorCtx) : 0xFF;
      logTdoaCandidates(engineState, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, selectedId);
    }
#ifdef CONFIG_DECK_LOCO_TDOA_RATE_LIMIT
    if (!isForwardRateLimited(engineState, engineState->maxRateHz)) {
#endif
      if (suitableAnchorFound) {
        STATS_CNT_RATE_EVENT(&engineState->stats.suitableDataFound);
        enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, engineState);
      }
#ifdef CONFIG_DECK_LOCO_TDOA_RATE_LIMIT
    }
#endif
  }
  return timeIsGood;
}

PARAM_GROUP_START(tdoaEngine)
/**
 * @brief Anchor pair candidates with a measured-distance / anchor-distance ratio at or
 * above this limit are rejected as bad geometry (TDoA3/matchRandomAnchor only).
 * The value is in range [0, 1], where a value closer to one means letting through more
 * measurements. Reasonable range is [0.6, 0.98].
 */
PARAM_ADD(PARAM_FLOAT, distRatio, &distanceRatioLimit)
PARAM_GROUP_STOP(tdoaEngine)

