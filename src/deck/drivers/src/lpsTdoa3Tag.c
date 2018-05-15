/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoa3Tag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoa3Tag.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-15 anchors are visible
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

#include "log.h"
#include "param.h"
#include "debug.h"
#include "lpsTdoa3Tag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "outlierFilter.h"

#define DEBUG_MODULE "tdoa3"

#define MEASUREMENT_NOISE_STD 0.15f
#define STATS_INTERVAL 500
#define ANCHOR_OK_TIMEOUT 1500


#define ANCHOR_STORAGE_COUNT 8
#define REMOTE_ANCHOR_DATA_COUNT 8
#define TOF_PER_ANCHOR_COUNT 8

#define TOF_VALIDITY_PERIOD M2T(10 * 1000);
#define REMOTE_DATA_VALIDITY_PERIOD M2T(30);
#define ANCHOR_POSITION_VALIDITY_PERIOD M2T(10 * 1000);

// State
typedef struct {
  uint8_t id; // Id of remote remote anchor
  uint8_t seqNr; // Sequence number of the packet received in the remote anchor (7 bits)
  int64_t rxTime; // Receive time of packet from anchor id in the remote anchor, in remote DWM clock
  uint32_t endOfLife;
} remoteAnchorData_t;

typedef struct {
  uint8_t id;
  int64_t tof;
  uint32_t endOfLife; // Time stamp when the tof data is outdated, local system time
} timeOfFlight_t;

typedef struct {
  bool isInitialized;
  uint32_t lastUpdateTime; // The time when this anchor was updated the last time
  uint8_t id; // Anchor id

  int64_t txTime; // Transmit time of last packet, in remote DWM clock
  int64_t rxTime; // Receive time of last packet, in local DWM clock
  uint8_t seqNr; // Sequence nr of last packet (7 bits)

  double clockCorrection; // local DWM clock frequency / Ratio of remote DWM clock frequency

  point_t position; // The coordinates of the anchor

  timeOfFlight_t tof[TOF_PER_ANCHOR_COUNT];
  remoteAnchorData_t remoteAnchorData[REMOTE_ANCHOR_DATA_COUNT];
} anchorInfo_t;

anchorInfo_t anchorStorage[ANCHOR_STORAGE_COUNT];


// TODO krri Debug code to dump state to the console, remove
uint8_t dumpTrigger;
uint8_t dumpTriggerOld;

static void dumpData(uint8_t anchorId) {
  uint32_t now = xTaskGetTickCount();

  DEBUG_PRINT("Dump----\n");
  for (int slot = 0; slot < ANCHOR_STORAGE_COUNT; slot++) {
    const anchorInfo_t* ctx = &anchorStorage[slot];
    if (ctx->isInitialized && anchorId == ctx->id) {
      DEBUG_PRINT("\nAnchor id %i\n", ctx->id);
      DEBUG_PRINT(" seqNr: %i\n", ctx->seqNr);
      DEBUG_PRINT(" rx: %lli, tx: %lli\n", ctx->rxTime, ctx->txTime);
      DEBUG_PRINT(" clockCorrection: %f\n", ctx->clockCorrection);

      DEBUG_PRINT(" TOF:\n");
      for (int i = 0; i < TOF_PER_ANCHOR_COUNT; i++) {
        if (ctx->tof[i].endOfLife > now) {
          DEBUG_PRINT("  id: %i, tof: %lli\n", ctx->tof[i].id, ctx->tof[i].tof);
        }
      }

      DEBUG_PRINT(" Remote:\n");
      for (int i = 0; i < REMOTE_ANCHOR_DATA_COUNT; i++) {
        if (ctx->remoteAnchorData[i].endOfLife > now) {
          DEBUG_PRINT("  id: %i, seq: %i, rx: %lli \n", ctx->remoteAnchorData[i].id, ctx->remoteAnchorData[i].seqNr, ctx->remoteAnchorData[i].rxTime);
        }
      }
    }
  }
}

static int64_t packetTxTime(const rangePacket3_t* packet) {
  return packet->header.txTimeStamp;
}

static anchorInfo_t* historyGetAnchorCtx(const uint8_t anchor) {
  for (int i = 0; i < ANCHOR_STORAGE_COUNT; i++) {
    if (anchor == anchorStorage[i].id && anchorStorage[i].isInitialized) {
      return &anchorStorage[i];
    }
  }

  return 0;
}

static anchorInfo_t* historyInitializeNewAchorContext(const uint8_t anchor) {
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

static uint8_t historyGetId(const anchorInfo_t* anchorCtx) {
  return anchorCtx->id;
}

static int64_t historyGetRxTime(const anchorInfo_t* anchorCtx) {
  return anchorCtx->rxTime;
}

static int64_t historyGetTxTime(const anchorInfo_t* anchorCtx) {
  return anchorCtx->txTime;
}

static uint8_t historyGetSeqNr(const anchorInfo_t* anchorCtx) {
  return anchorCtx->seqNr;
}

static bool historyGetAnchorPosition(const anchorInfo_t* anchorCtx, point_t* position) {
  uint32_t now = xTaskGetTickCount();
  uint32_t validCreationTime = now - ANCHOR_POSITION_VALIDITY_PERIOD;
  if (anchorCtx->position.timestamp > validCreationTime) {
    position->x = anchorCtx->position.x;
    position->y = anchorCtx->position.y;
    position->z = anchorCtx->position.z;
    return true;
  }

  return false;
}

static void historySetAnchorPosition(anchorInfo_t* anchorCtx, const float x, const float y, const float z) {
  uint32_t now = xTaskGetTickCount();

  anchorCtx->position.timestamp = now;
  anchorCtx->position.x = x;
  anchorCtx->position.y = y;
  anchorCtx->position.z = z;
}

static void historySetRxTxData(anchorInfo_t* anchorCtx, int64_t rxTime, int64_t txTime, uint8_t seqNr) {
  uint32_t now = xTaskGetTickCount();

  anchorCtx->rxTime = rxTime;
  anchorCtx->txTime = txTime;
  anchorCtx->seqNr = seqNr;
  anchorCtx->lastUpdateTime = now;
}

static double historyGetClockCorrection(const anchorInfo_t* anchorCtx) {
  return anchorCtx->clockCorrection;
}

static void historySetClockCorrection(anchorInfo_t* anchorCtx, const double clockCorrection) {
  anchorCtx->clockCorrection = clockCorrection;
}

static int64_t historyGetRemoteRxTime(const anchorInfo_t* anchorCtx, const uint8_t remoteAnchor) {
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

static void historySetRemoteRxTime(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t remoteRxTime, const uint8_t remoteSeqNr) {
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

static void historyGetRemoteSeqNrList(const anchorInfo_t* anchorCtx, int* remoteCount, uint8_t seqNr[], uint8_t id[]) {
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

static int64_t historyGetTimeOfFlight(const anchorInfo_t* anchorCtx, const uint8_t otherAnchor) {
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

static void historySetTimeOfFlight(anchorInfo_t* anchorCtx, const uint8_t remoteAnchor, const int64_t tof) {
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


static lpsAlgoOptions_t* options;

// Outgoing LPP packet
lpsLppShortPacket_t lppPacket;

// Log data

static struct {
  uint32_t packetsReceived;
  uint32_t packetsToEstimator;
  uint32_t contextHitCount;
  uint32_t contextMissCount;
  uint32_t suitableDataFound;

  uint16_t packetsReceivedRate;
  uint16_t packetsToEstimatorRate;
  uint16_t contextHitRate;
  uint16_t contextMissRate;
  uint16_t suitableDataFoundRate;

  uint32_t nextStatisticsTime;
  uint32_t previousStatisticsTime;

  // Anchor stats
  uint8_t anchorId; // The id of the anchor to log
  uint8_t newAnchorId; // Used to change anchor to log, set as param

  uint8_t remoteAnchorId; // The id of the remote anchor to log
  uint8_t newRemoteAnchorId; // Used to change remote anchor to log, set as param

  float clockCorrection;
  uint32_t clockCorrectionCount;
  uint16_t clockCorrectionRate;
  uint16_t tof;
} stats;


static void clearStats() {
  stats.packetsReceived = 0;
  stats.packetsToEstimator = 0;
  stats.clockCorrectionCount = 0;
  stats.contextHitCount = 0;
  stats.contextMissCount = 0;
  stats.suitableDataFound = 0;
}

static void updateStats() {
  uint32_t now = xTaskGetTickCount();
  if (now > stats.nextStatisticsTime) {
    float interval = now - stats.previousStatisticsTime;
    ASSERT(interval > 0.0f);
    stats.packetsReceivedRate = (uint16_t)(1000.0f * stats.packetsReceived / interval);
    stats.packetsToEstimatorRate = (uint16_t)(1000.0f * stats.packetsToEstimator / interval);
    stats.clockCorrectionRate = (uint16_t)(1000.0f * stats.clockCorrectionCount / interval);

    stats.contextHitRate = (uint16_t)(1000.0f * stats.contextHitCount / interval);
    stats.contextMissRate = (uint16_t)(1000.0f * stats.contextMissCount / interval);

    stats.suitableDataFoundRate = (uint16_t)(1000.0f * stats.suitableDataFound / interval);

    if (stats.anchorId != stats.newAnchorId) {
      stats.anchorId = stats.newAnchorId;

      // Reset anchor stats
      stats.clockCorrection = 0.0;
      stats.tof = 0;
    }

    if (stats.remoteAnchorId != stats.newRemoteAnchorId) {
      stats.remoteAnchorId = stats.newRemoteAnchorId;

      // Reset remote anchor stats
      stats.tof = 0;
    }

    clearStats();
    stats.previousStatisticsTime = now;
    stats.nextStatisticsTime = now + STATS_INTERVAL;
  }
}

// TODO krri Find better way to communicate system state to the client. Currently only supports 8 anchors
static void updateRangingState() {
  options->rangingState = 0;
//  for (int anchor = 0; anchor < LOCODECK_NR_OF_TDOA2_ANCHORS; anchor++) {
//    if (now < history[anchor].anchorStatusTimeout) {
//      options->rangingState |= (1 << anchor);
//    }
//  }
}

static bool rangingOk;


static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static void enqueueTDOA(const anchorInfo_t* anchorACtx, const anchorInfo_t* anchorBCtx, double distanceDiff) {
  point_t estimatedPos;
  estimatorKalmanGetEstimatedPos(&estimatedPos);

  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff
  };

  if (historyGetAnchorPosition(anchorACtx, &tdoa.anchorPosition[0]) && historyGetAnchorPosition(anchorBCtx, &tdoa.anchorPosition[1])) {
    if (outlierFilterValidateTdoa(&tdoa, &estimatedPos)) {
      stats.packetsToEstimator++;
      estimatorKalmanEnqueueTDOA(&tdoa);

      rangingOk = true;
    }
  }
}

static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

//static bool isSeqNrConsecutive(uint8_t prevSeqNr, uint8_t currentSeqNr) {
//  return (currentSeqNr == ((prevSeqNr + 1) & 0x7f));
//}

static double calcClockCorrection(const anchorInfo_t* anchorCtx, const rangePacket3_t* packet, const dwTime_t* arrival) {
  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packetTxTime(packet);
  const int64_t latest_rxAn_by_T_in_cl_T = historyGetRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = historyGetTxTime(anchorCtx);

  const double tickCount_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - latest_txAn_in_cl_An);
  const double tickCount_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);

  if (tickCount_in_cl_An == 0) {
    return 0.0d;
  }

  return tickCount_in_T / tickCount_in_cl_An;
}

static void updateClockCorrection(anchorInfo_t* anchorCtx, const rangePacket3_t* packet, const dwTime_t* arrival) {
  double clockCorrectionCandidate = calcClockCorrection(anchorCtx, packet, arrival);

//  const double MAX_CLOCK_CORRECTION_ERR 0.00004
//  if (clockCorrectionCandidate < (1.0d - MAX_CLOCK_CORRECTION_ERR) || (1.0d + MAX_CLOCK_CORRECTION_ERR) < clockCorrectionCandidate) {
//    return;
//  }

  // TODO krri Add sanity checks
  //           * reject if missing seq nrs?
  //           * If too long between samples, clocks may have wrapped several times, problem?
  // TODO krri reject outliers?
  // TODO krri LP filter

  historySetClockCorrection(anchorCtx, clockCorrectionCandidate);
  if (historyGetId(anchorCtx) == stats.anchorId) {
    stats.clockCorrection = clockCorrectionCandidate;
    stats.clockCorrectionCount++;
  }
}

static int64_t calcTDoA(const anchorInfo_t* otherAnchorCtx, const anchorInfo_t* anchorCtx, const rangePacket3_t* packet, const dwTime_t* arrival) {
  const uint8_t otherAnchorId = historyGetId(otherAnchorCtx);

  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t tof_Ar_to_An_in_cl_An = historyGetTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = historyGetRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = historyGetClockCorrection(anchorCtx);

  const int64_t txAn_in_cl_An = packetTxTime(packet);
  const int64_t rxAr_by_T_in_cl_T = historyGetRxTime(otherAnchorCtx);

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_T =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T;
}

static float calcDistanceDiff(const anchorInfo_t* otherAnchorCtx, const anchorInfo_t* anchorCtx, const rangePacket3_t* packet, const dwTime_t* arrival) {
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, packet, arrival);
  return SPEED_OF_LIGHT * tdoa / LOCODECK_TS_FREQ;
}

static int updateRemoteData(anchorInfo_t* anchorCtx, const rangePacket3_t* packet) {
  const void* anchorDataPtr = &packet->remoteAnchorData;
  for (uint8_t i = 0; i < packet->header.remoteCount; i++) {
    remoteAnchorDataFull_t* anchorData = (remoteAnchorDataFull_t*)anchorDataPtr;

    uint8_t remoteId = anchorData->id;
    int64_t remoteRxTime = anchorData->rxTimeStamp;
    uint8_t remoteSeqNr = anchorData->seq & 0x7f;

    if (isValidTimeStamp(remoteRxTime)) {
      historySetRemoteRxTime(anchorCtx, remoteId, remoteRxTime, remoteSeqNr);
    }

    bool hasDistance = ((anchorData->seq & 0x80) != 0);
    if (hasDistance) {
      int64_t tof = anchorData->distance;
      if (isValidTimeStamp(tof)) {
        historySetTimeOfFlight(anchorCtx, remoteId, tof);

        uint8_t anchorId = historyGetId(anchorCtx);
        if (anchorId == stats.anchorId && remoteId == stats.remoteAnchorId) {
          stats.tof = (uint16_t)tof;
        }
      }

      anchorDataPtr += sizeof(remoteAnchorDataFull_t);
    } else {
      anchorDataPtr += sizeof(remoteAnchorDataShort_t);
    }
  }

  return anchorDataPtr - (void*)packet;
}

static void updateAnchorData(anchorInfo_t* anchorCtx, const rangePacket3_t* packet, const dwTime_t* arrival) {
  historySetRxTxData(anchorCtx, arrival->full, packet->header.txTimeStamp, packet->header.seq);
}

static bool findSuitableAnchor(anchorInfo_t** otherAnchorCtx, const anchorInfo_t* anchorCtx) {
  static uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
  static uint8_t id[REMOTE_ANCHOR_DATA_COUNT];

  if (historyGetClockCorrection(anchorCtx) == 0) {
    return false;
  }

  int remoteCount = 0;
  historyGetRemoteSeqNrList(anchorCtx, &remoteCount, seqNr, id);

  for (int i = 0; i < remoteCount; i++) {
    const uint8_t candidateAnchorId = id[i];
    anchorInfo_t* candidateAnchorCtx = historyGetAnchorCtx(candidateAnchorId);
    if (candidateAnchorCtx) {
      if (seqNr[i] == historyGetSeqNr(candidateAnchorCtx) && historyGetTimeOfFlight(anchorCtx, candidateAnchorId)) {
        *otherAnchorCtx = candidateAnchorCtx;
        return true;
      }
    }
  }

  return false;
}

static void handleLppShortPacket(anchorInfo_t* anchorCtx, const uint8_t *data, const int length) {
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
    historySetAnchorPosition(anchorCtx, newpos->x, newpos->y, newpos->z);
  }
}

static void handleLppPacket(const int dataLength, int rangePacketLength, const packet_t* rxPacket, anchorInfo_t* anchorCtx) {
  const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
  const int32_t startOfLppDataInPayload = rangePacketLength;
  const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;
  const int32_t lppTypeInPayload = startOfLppDataInPayload + 1;

  if (lppDataLength > 0) {
    const uint8_t lppPacketHeader = rxPacket->payload[startOfLppDataInPayload];
    if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
      const int32_t lppTypeAndPayloadLength = lppDataLength - 1;
      handleLppShortPacket(anchorCtx, &rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);

      // TODO krri Find better solution for communicating system state to the client
      // Send it to the "old" path to log anchor 0 - 7 positions to the client.
      lpsHandleLppShortPacket(historyGetId(anchorCtx), &rxPacket->payload[lppTypeInPayload], lppTypeAndPayloadLength);
    }
  }
}

static bool rxcallback(dwDevice_t *dev) {
  stats.packetsReceived++;

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  const uint8_t anchorId = rxPacket.sourceAddress & 0xff;
  anchorInfo_t* anchorCtx = historyGetAnchorCtx(anchorId);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  const rangePacket3_t* packet = (rangePacket3_t*)rxPacket.payload;
  int rangeDataLength = 0;

  if (anchorCtx) {
    stats.contextHitCount++;
    updateClockCorrection(anchorCtx, packet, &arrival);
    rangeDataLength = updateRemoteData(anchorCtx, packet);

    anchorInfo_t* otherAnchorCtx = 0;
    if (findSuitableAnchor(&otherAnchorCtx, anchorCtx)) {
      stats.suitableDataFound++;
      float tdoaDistDiff = calcDistanceDiff(otherAnchorCtx, anchorCtx, packet, &arrival);
      enqueueTDOA(otherAnchorCtx, anchorCtx, tdoaDistDiff);
    }
  } else {
    stats.contextMissCount++;
    anchorCtx = historyInitializeNewAchorContext(anchorId);
    rangeDataLength = updateRemoteData(anchorCtx, packet);
  }

  updateAnchorData(anchorCtx, packet, &arrival);

  handleLppPacket(dataLength, rangeDataLength, &rxPacket, anchorCtx);

  return false;
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  static packet_t txPacket;
  dwIdle(dev);

  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

  txPacket.payload[LPS_TDOA3_TYPE] = LPP_HEADER_SHORT_PACKET;
  memcpy(&txPacket.payload[LPS_TDOA3_SEND_LPP_PAYLOAD], packet->data, packet->length);

  txPacket.pan = 0xbccf;
  txPacket.sourceAddress = 0xbccf000000000000 | 0xff;
  txPacket.destAddress = 0xbccf000000000000 | packet->dest;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwStartTransmit(dev);
}

static bool sendLpp(dwDevice_t *dev) {
  bool lppPacketToSend = lpsGetLppShort(&lppPacket);
  if (lppPacketToSend) {
    sendLppShort(dev, &lppPacket);
    return true;
  }

  return false;
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      break;
    case eventTimeout:
      break;
    case eventReceiveTimeout:
      break;
    case eventPacketSent:
      // Service packet sent, the radio is back to receive automatically
      break;
    default:
      ASSERT_FAILED();
  }

  if(!sendLpp(dev)) {
    setRadioInReceiveMode(dev);
  }

  updateStats();
  updateRangingState();

  if (dumpTrigger != dumpTriggerOld) {
    dumpTriggerOld = dumpTrigger;
    dumpData(dumpTrigger);
  }

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;

  // Reset module state. Needed by unit tests
  memset(anchorStorage, 0, sizeof(anchorStorage));

  options->rangingState = 0;

  clearStats();
  stats.packetsReceivedRate = 0;
  stats.packetsToEstimatorRate = 0;
  stats.clockCorrectionRate = 0;
  stats.nextStatisticsTime = xTaskGetTickCount() + STATS_INTERVAL;
  stats.previousStatisticsTime = 0;

  dwSetReceiveWaitTimeout(dev, TDOA3_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}
#pragma GCC diagnostic pop

static bool isRangingOk()
{
  return rangingOk;
}

uwbAlgorithm_t uwbTdoa3TagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
};


LOG_GROUP_START(tdoa3)
LOG_ADD(LOG_UINT16, stRx, &stats.packetsReceivedRate)
LOG_ADD(LOG_UINT16, stEst, &stats.packetsToEstimatorRate)
LOG_ADD(LOG_UINT16, stFound, &stats.suitableDataFoundRate)

LOG_ADD(LOG_UINT16, stCc, &stats.clockCorrectionRate)

LOG_ADD(LOG_UINT16, stHit, &stats.contextHitRate)
LOG_ADD(LOG_UINT16, stMiss, &stats.contextMissRate)

LOG_ADD(LOG_FLOAT, cc, &stats.clockCorrection)
LOG_ADD(LOG_UINT16, tof, &stats.tof)

LOG_GROUP_STOP(tdoa3)

PARAM_GROUP_START(tdoa3)
PARAM_ADD(PARAM_UINT8, logid, &stats.newAnchorId)
PARAM_ADD(PARAM_UINT8, logremid, &stats.newRemoteAnchorId)
PARAM_ADD(PARAM_UINT8, dump, &dumpTrigger)
PARAM_GROUP_STOP(tdoa3)
