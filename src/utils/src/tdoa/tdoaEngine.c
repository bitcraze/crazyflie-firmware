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
#include "test_support.h"
#include "param.h"

#define TDOA_ENGINE_DEFAULT_DISTANCE_RATIO_LIMIT 0.85f

static float distanceRatioLimit = TDOA_ENGINE_DEFAULT_DISTANCE_RATIO_LIMIT;

void tdoaEngineInit(tdoaEngineState_t* engineState, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm) {
  tdoaStorageInitialize(engineState->anchorInfoArray);
  tdoaStatsInit(&engineState->stats, now_ms);
  engineState->sendTdoaToEstimator = sendTdoaToEstimator;
  engineState->locodeckTsFreq = locodeckTsFreq;
  engineState->matchingAlgorithm = matchingAlgorithm;

  engineState->matching.offset = 0;
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

static float sq(float a) { return a * a; }

static float distanceBetweenAnchorsSquared(const point_t* a, const point_t* b) {
  return sq(a->x - b->x) + sq(a->y - b->y) + sq(a->z - b->z);
}

static bool isGeometryGoodEnough(const tdoaAnchorContext_t* anchorCtx, const tdoaAnchorContext_t* otherAnchorCtx, const double candidateDistanceDiff) {
  // A candidate is rejected if the ratio between the measured distance diff and the distance
  // between the anchors is at or above distanceRatioLimit - this indicates bad pair geometry.
  // This is a rough estimation of the measurement sensitivity.

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

TESTABLE_STATIC bool matchRandomAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq, double* distanceDiff) {
  engineState->matching.offset++;
  int remoteCount = 0;
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

  uint32_t now_ms = anchorCtx->currentTime_ms;

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
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

static bool matchYoungestAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq, double* distanceDiff) {
    int remoteCount = 0;
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

    uint32_t now_ms = anchorCtx->currentTime_ms;
    uint32_t youmgestUpdateTime = 0;
    int bestId = -1;

    for (int index = 0; index < remoteCount; index++) {
      const uint8_t candidateAnchorId = engineState->matching.id[index];
      if (!doExcludeId || (excludedId != candidateAnchorId)) {
        if (tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
            uint32_t updateTime = tdoaStorageGetLastUpdateTime(otherAnchorCtx);
            if (updateTime > youmgestUpdateTime) {
              if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx)) {
                youmgestUpdateTime = updateTime;
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

static bool findSuitableAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq, double* distanceDiff) {
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

void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  tdoaEngineProcessPacketFiltered(engineState, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, false, 0);
}

bool tdoaEngineProcessPacketFiltered(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const bool doExcludeId, const uint8_t excludedId) {
  bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
  if (timeIsGood) {
    STATS_CNT_RATE_EVENT(&engineState->stats.timeIsGood);

    tdoaAnchorContext_t otherAnchorCtx;
    double tdoaDistDiff = 0.0;
    if (findSuitableAnchor(engineState, &otherAnchorCtx, anchorCtx, doExcludeId, excludedId, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->locodeckTsFreq, &tdoaDistDiff)) {
      STATS_CNT_RATE_EVENT(&engineState->stats.suitableDataFound);
      enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, engineState);
    }
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
