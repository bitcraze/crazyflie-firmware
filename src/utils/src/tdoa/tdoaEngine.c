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

#include "FreeRTOS.h"
#include "task.h"

#include "tdoaEngine.h"
#include "tdoaStats.h"

#include "locodeck.h"
#include "cfassert.h"

#define DEBUG_MODULE "TDOA_ENGINE"
#include "debug.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "outlierFilter.h"
#include "clockCorrectionEngine.h"

#define MEASUREMENT_NOISE_STD 0.15f
#define ANCHOR_OK_TIMEOUT 1500

void tdoaEngineInit() {
  tdoaStorageInitialize();

  #ifdef LPS_2D_POSITION_HEIGHT
  DEBUG_PRINT("2D positioning enabled at %f m height\n", LPS_2D_POSITION_HEIGHT);
  #endif
}

#define TRUNCATE_TO_ANCHOR_TS_BITMAP 0x00FFFFFFFF
static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

static void enqueueTDOA(const anchorInfo_t* anchorACtx, const anchorInfo_t* anchorBCtx, double distanceDiff) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff
  };

  if (tdoaStorageGetAnchorPosition(anchorACtx, &tdoa.anchorPosition[0]) && tdoaStorageGetAnchorPosition(anchorBCtx, &tdoa.anchorPosition[1])) {
    if (outlierFilterValidateTdoa(&tdoa)) {
      lpsTdoaStats.packetsToEstimator++;
      estimatorKalmanEnqueueTDOA(&tdoa);

      #ifdef LPS_2D_POSITION_HEIGHT
      // If LPS_2D_POSITION_HEIGHT we assume that we are doing 2D positioning.
      // LPS_2D_POSITION_HEIGHT contains the height (Z) that the tag will be located at
      heightMeasurement_t heightData;
      heightData.timestamp = xTaskGetTickCount();
      heightData.height = LPS_2D_POSITION_HEIGHT;
      heightData.stdDev = 0.0001;
      estimatorKalmanEnqueueAsoluteHeight(&heightData);
      #endif

      uint8_t idA = tdoaStorageGetId(anchorACtx);
      uint8_t idB = tdoaStorageGetId(anchorBCtx);
      if (idA == lpsTdoaStats.anchorId && idB == lpsTdoaStats.remoteAnchorId) {
        lpsTdoaStats.tdoa = distanceDiff;
      }
      if (idB == lpsTdoaStats.anchorId && idA == lpsTdoaStats.remoteAnchorId) {
        lpsTdoaStats.tdoa = -distanceDiff;
      }
    }
  }
}

static bool updateClockCorrection(anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  bool sampleIsReliable = false;

  const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);

  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    double clockCorrectionCandidate = clockCorrectionEngine.calculateClockCorrection(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TRUNCATE_TO_ANCHOR_TS_BITMAP);
    sampleIsReliable = clockCorrectionEngine.updateClockCorrection(&anchorCtx->clockCorrectionStorage, clockCorrectionCandidate);

    if (sampleIsReliable){
      if (tdoaStorageGetId(anchorCtx) == lpsTdoaStats.anchorId) {
        lpsTdoaStats.clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        lpsTdoaStats.clockCorrectionCount++;
      }
    }
  }

  return sampleIsReliable;
}

static int64_t calcTDoA(const anchorInfo_t* otherAnchorCtx, const anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx);

  const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

  const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx);

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_T =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T;
}

static double calcDistanceDiff(const anchorInfo_t* otherAnchorCtx, const anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  return SPEED_OF_LIGHT * tdoa / LOCODECK_TS_FREQ;
}


static bool findSuitableAnchor(anchorInfo_t** otherAnchorCtx, const anchorInfo_t* anchorCtx) {
  static uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
  static uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
  static uint8_t offset = 0;

  if (tdoaStorageGetClockCorrection(anchorCtx) <= 0.0) {
    return false;
  }

  offset++;
  int remoteCount = 0;
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, seqNr, id);

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
  for (int i = offset; i < (remoteCount + offset); i++) {
    uint8_t index = i % remoteCount;
    const uint8_t candidateAnchorId = id[index];
    anchorInfo_t* candidateAnchorCtx = tdoaStorageGetAnchorCtx(candidateAnchorId);
    if (candidateAnchorCtx) {
      if (seqNr[index] == tdoaStorageGetSeqNr(candidateAnchorCtx) && tdoaStorageGetTimeOfFlight(anchorCtx, candidateAnchorId)) {
        *otherAnchorCtx = candidateAnchorCtx;
        return true;
      }
    }
  }

  return false;
}

anchorInfo_t* tdoaEngineGetAnchorCtxForPacketProcessing(const uint8_t anchorId) {
  anchorInfo_t* anchorCtx = tdoaStorageGetAnchorCtx(anchorId);
  if (anchorCtx) {
    lpsTdoaStats.contextHitCount++;
  } else {
    lpsTdoaStats.contextMissCount++;
    anchorCtx = tdoaStorageInitializeNewAnchorContext(anchorId);
  }

  return anchorCtx;
}

void tdoaEngineProcessPacket(anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
    bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
    if (timeIsGood) {
      lpsTdoaStats.timeIsGood++;

      anchorInfo_t* otherAnchorCtx = 0;
      if (findSuitableAnchor(&otherAnchorCtx, anchorCtx)) {
        lpsTdoaStats.suitableDataFound++;
        double tdoaDistDiff = calcDistanceDiff(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
        enqueueTDOA(otherAnchorCtx, anchorCtx, tdoaDistDiff);
      }
    }
}
