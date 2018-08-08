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

#include "lpsTdoaTagEngine.h"
#include "lpsTdoaTagStats.h"

#include "locodeck.h"
#include "cfassert.h"

#define DEBUG_MODULE "TDOA_ENGINE"
#include "debug.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "outlierFilter.h"

#define MEASUREMENT_NOISE_STD 0.15f
#define ANCHOR_OK_TIMEOUT 1500

#define TOF_VALIDITY_PERIOD M2T(2 * 1000);
#define REMOTE_DATA_VALIDITY_PERIOD M2T(30);
#define ANCHOR_POSITION_VALIDITY_PERIOD M2T(2 * 1000);

#define MAX_CLOCK_DEVIATION_SPEC 10e-6
#define CLOCK_CORRECTION_SPEC_MIN (1.0 - MAX_CLOCK_DEVIATION_SPEC * 2)
#define CLOCK_CORRECTION_SPEC_MAX (1.0 + MAX_CLOCK_DEVIATION_SPEC * 2)

#define CLOCK_CORRECTION_ACCEPTED_NOISE 0.03e-6
#define CLOCK_CORRECTION_FILTER 0.1
#define CLOCK_CORRECTION_BUCKET_MAX 4


static anchorInfo_t anchorStorage[ANCHOR_STORAGE_COUNT];

void tdoaEngineInit() {
  memset(anchorStorage, 0, sizeof(anchorStorage));
  #ifdef LPS_2D_POSITION_HEIGHT
  DEBUG_PRINT("2D positioning enabled at %f m height\n", LPS_2D_POSITION_HEIGHT);
  #endif
}

static anchorInfo_t* getAnchorCtx(const uint8_t anchor) {
  // TODO krri add lookup table to avoid linear search
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (anchor == anchorStorage[i].id && anchorStorage[i].isInitialized) {
      return &anchorStorage[i];
    }
  }

  return 0;
}

static anchorInfo_t* initializeNewAnchorContext(const uint8_t anchor) {
  int indexToInitialize = 0;
  uint32_t now = xTaskGetTickCount();
  uint32_t oldestUpdateTime = now;

  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (!anchorStorage[i].isInitialized) {
      indexToInitialize = i;
      break;
    }

    if (anchorStorage[i].lastUpdateTime < oldestUpdateTime) {
      oldestUpdateTime = anchorStorage[i].lastUpdateTime;
      indexToInitialize = i;
    }
  }

  memset(&anchorStorage[indexToInitialize], 0, sizeof(anchorInfo_t));
  anchorStorage[indexToInitialize].id = anchor;
  anchorStorage[indexToInitialize].isInitialized = true;

  return &anchorStorage[indexToInitialize];
}

uint8_t tdoaEngineGetId(const anchorInfo_t* anchorCtx) {
  return anchorCtx->id;
}

static int64_t getRxTime(const anchorInfo_t* anchorCtx) {
  return anchorCtx->rxTime;
}

static int64_t getTxTime(const anchorInfo_t* anchorCtx) {
  return anchorCtx->txTime;
}

static uint8_t getSeqNr(const anchorInfo_t* anchorCtx) {
  return anchorCtx->seqNr;
}

static bool getAnchorPosition(const anchorInfo_t* anchorCtx, point_t* position) {
  uint32_t now = xTaskGetTickCount();
  int32_t validCreationTime = now - ANCHOR_POSITION_VALIDITY_PERIOD;
  if ((int32_t)anchorCtx->position.timestamp > validCreationTime) {
    position->x = anchorCtx->position.x;
    position->y = anchorCtx->position.y;
    position->z = anchorCtx->position.z;
    return true;
  }

  return false;
}

void tdoaEngineSetAnchorPosition(anchorInfo_t* anchorCtx, const float x, const float y, const float z) {
  uint32_t now = xTaskGetTickCount();

  anchorCtx->position.timestamp = now;
  anchorCtx->position.x = x;
  anchorCtx->position.y = y;
  anchorCtx->position.z = z;
}

void tdoaEngineSetRxTxData(anchorInfo_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr) {
  uint32_t now = xTaskGetTickCount();

  anchorCtx->rxTime = rxTime;
  anchorCtx->txTime = txTime;
  anchorCtx->seqNr = seqNr;
  anchorCtx->lastUpdateTime = now;
}

static double getClockCorrection(const anchorInfo_t* anchorCtx) {
  return anchorCtx->clockCorrection;
}

static void setClockCorrection(anchorInfo_t* anchorCtx, const double clockCorrection) {
  anchorCtx->clockCorrection = clockCorrection;
}

static int64_t getRemoteRxTime(const anchorInfo_t* anchorCtx, const uint8_t remoteAnchor) {
  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (remoteAnchor == anchorCtx->remoteAnchorData[i].id) {
      uint32_t now = xTaskGetTickCount();
      if (anchorCtx->remoteAnchorData[i].endOfLife > now) {
        return anchorCtx->remoteAnchorData[i].rxTime;
      }
      break;
    }
  }

  return 0;
}

void tdoaEngineSetRemoteRxTime(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr) {
  int indexToUpdate = 0;
  uint32_t now = xTaskGetTickCount();
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (remoteAnchor == anchorCtx->remoteAnchorData[i].id) {
      indexToUpdate = i;
      break;
    }

    if (anchorCtx->remoteAnchorData[i].endOfLife < oldestTime) {
      oldestTime = anchorCtx->remoteAnchorData[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorCtx->remoteAnchorData[indexToUpdate].id = remoteAnchor;
  anchorCtx->remoteAnchorData[indexToUpdate].rxTime = remoteRxTime;
  anchorCtx->remoteAnchorData[indexToUpdate].seqNr = remoteSeqNr;
  anchorCtx->remoteAnchorData[indexToUpdate].endOfLife = now + REMOTE_DATA_VALIDITY_PERIOD;
}

static void getRemoteSeqNrList(const anchorInfo_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]) {
  int count = 0;

  uint32_t now = xTaskGetTickCount();
  for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
    if (anchorCtx->remoteAnchorData[i].endOfLife > now) {
      id[count] = anchorCtx->remoteAnchorData[i].id;
      seqNr[count] = anchorCtx->remoteAnchorData[i].seqNr;
      count++;
    }
  }

  *remoteCount = count;
}

static int64_t getTimeOfFlight(const anchorInfo_t* anchorCtx, const uint8_t otherAnchor) {
  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
    if (otherAnchor == anchorCtx->tof[i].id) {
      uint32_t now = xTaskGetTickCount();
      if (anchorCtx->tof[i].endOfLife > now) {
        return anchorCtx->tof[i].tof;
      }
      break;
    }
  }

  return 0;
}

void tdoaEngineSetTimeOfFlight(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof) {
  int indexToUpdate = 0;
  uint32_t now = xTaskGetTickCount();
  uint32_t oldestTime = 0xFFFFFFFF;

  for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
    if (remoteAnchor == anchorCtx->tof[i].id) {
      indexToUpdate = i;
      break;
    }

    if (anchorCtx->tof[i].endOfLife < oldestTime) {
      oldestTime = anchorCtx->tof[i].endOfLife;
      indexToUpdate = i;
    }
  }

  anchorCtx->tof[indexToUpdate].id = remoteAnchor;
  anchorCtx->tof[indexToUpdate].tof = tof;
  anchorCtx->tof[indexToUpdate].endOfLife = now + TOF_VALIDITY_PERIOD;
}


///////////////////////////////////////////////////////////////


static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static void enqueueTDOA(const anchorInfo_t* anchorACtx, const anchorInfo_t* anchorBCtx, double distanceDiff) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff
  };

  if (getAnchorPosition(anchorACtx, &tdoa.anchorPosition[0]) && getAnchorPosition(anchorBCtx, &tdoa.anchorPosition[1])) {
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

      uint8_t idA = tdoaEngineGetId(anchorACtx);
      uint8_t idB = tdoaEngineGetId(anchorBCtx);
      if (idA == lpsTdoaStats.anchorId && idB == lpsTdoaStats.remoteAnchorId) {
        lpsTdoaStats.tdoa = distanceDiff;
      }
      if (idB == lpsTdoaStats.anchorId && idA == lpsTdoaStats.remoteAnchorId) {
        lpsTdoaStats.tdoa = -distanceDiff;
      }
    }
  }
}

static void fillClockCorrectionBucket(anchorInfo_t* anchorCtx) {
    if (anchorCtx->clockCorrectionBucket < CLOCK_CORRECTION_BUCKET_MAX) {
      anchorCtx->clockCorrectionBucket++;
    }
}

static bool emptyClockCorrectionBucket(anchorInfo_t* anchorCtx) {
    if (anchorCtx->clockCorrectionBucket > 0) {
      anchorCtx->clockCorrectionBucket--;
      return false;
    }

    return true;
}

static double calcClockCorrection(const int64_t latest_rxAn_by_T_in_cl_T, const int64_t latest_txAn_in_cl_An, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const double tickCount_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - latest_txAn_in_cl_An);
  const double tickCount_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);

  if (tickCount_in_cl_An == 0) {
    return 0.0;
  }

  return tickCount_in_T / tickCount_in_cl_An;
}

static bool updateClockCorrection(anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  bool sampleIsAccepted = false;

  const int64_t latest_rxAn_by_T_in_cl_T = getRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = getTxTime(anchorCtx);

  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    double clockCorrectionCandidate = calcClockCorrection(latest_rxAn_by_T_in_cl_T, latest_txAn_in_cl_An, txAn_in_cl_An, rxAn_by_T_in_cl_T);

    const double currClockCorrection = getClockCorrection(anchorCtx);
    const double diff = clockCorrectionCandidate - currClockCorrection;

    if (-CLOCK_CORRECTION_ACCEPTED_NOISE < diff && diff < CLOCK_CORRECTION_ACCEPTED_NOISE) {
      // LP filter
      const double newClockCorrection = currClockCorrection * CLOCK_CORRECTION_FILTER + clockCorrectionCandidate * (1.0 - CLOCK_CORRECTION_FILTER);
      fillClockCorrectionBucket(anchorCtx);

      setClockCorrection(anchorCtx, newClockCorrection);
      sampleIsAccepted = true;
    } else {
      if (emptyClockCorrectionBucket(anchorCtx)) {
        if (CLOCK_CORRECTION_SPEC_MIN < clockCorrectionCandidate && clockCorrectionCandidate < CLOCK_CORRECTION_SPEC_MAX) {
          setClockCorrection(anchorCtx, clockCorrectionCandidate);
        }
      }
    }

    if (sampleIsAccepted){
      if (tdoaEngineGetId(anchorCtx) == lpsTdoaStats.anchorId) {
        lpsTdoaStats.clockCorrection = getClockCorrection(anchorCtx);
        lpsTdoaStats.clockCorrectionCount++;
      }
    }
  }

  return sampleIsAccepted;
}

static int64_t calcTDoA(const anchorInfo_t* otherAnchorCtx, const anchorInfo_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const uint8_t otherAnchorId = tdoaEngineGetId(otherAnchorCtx);

  const int64_t tof_Ar_to_An_in_cl_An = getTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = getRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = getClockCorrection(anchorCtx);

  const int64_t rxAr_by_T_in_cl_T = getRxTime(otherAnchorCtx);

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

  if (getClockCorrection(anchorCtx) == 0) {
    return false;
  }

  int remoteCount = 0;
  getRemoteSeqNrList(anchorCtx, &remoteCount, seqNr, id);

  // TODO krri Add randomization of which anchor to pick. Current implementation will favour anchors early in the list.
  for (int i = 0; i < remoteCount; i++) {
    const uint8_t candidateAnchorId = id[i];
    anchorInfo_t* candidateAnchorCtx = getAnchorCtx(candidateAnchorId);
    if (candidateAnchorCtx) {
      if (seqNr[i] == getSeqNr(candidateAnchorCtx) && getTimeOfFlight(anchorCtx, candidateAnchorId)) {
        *otherAnchorCtx = candidateAnchorCtx;
        return true;
      }
    }
  }

  return false;
}

anchorInfo_t* getAnchorCtxForPacketProcessing(const uint8_t anchorId) {
  anchorInfo_t* anchorCtx = getAnchorCtx(anchorId);
  if (anchorCtx) {
    lpsTdoaStats.contextHitCount++;
  } else {
    lpsTdoaStats.contextMissCount++;
    anchorCtx = initializeNewAnchorContext(anchorId);
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
