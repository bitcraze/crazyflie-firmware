/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * lpsTdoa2Tag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoa2Tag.c.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "lpsTdoa2Tag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"

#include "physicalConstants.h"

#define MEASUREMENT_NOISE_STD 0.15f
#define STATS_INTERVAL 500
#define ANCHOR_OK_TIMEOUT 1500

// Config
static lpsTdoa2AlgoOptions_t defaultOptions = {
   .anchorAddress = {
     0xbccf000000000000,
     0xbccf000000000001,
     0xbccf000000000002,
     0xbccf000000000003,
     0xbccf000000000004,
     0xbccf000000000005,
 #if LOCODECK_NR_OF_TDOA2_ANCHORS > 6
     0xbccf000000000006,
 #endif
 #if LOCODECK_NR_OF_TDOA2_ANCHORS > 7
     0xbccf000000000007,
 #endif
   },

   .combinedAnchorPositionOk = false,

   // To set a static anchor position from startup, uncomment and modify the
   // following code:
 //   .anchorPosition = {
 //     {timestamp: 1, x: 0.99, y: 1.49, z: 1.80},
 //     {timestamp: 1, x: 0.99, y: 3.29, z: 1.80},
 //     {timestamp: 1, x: 4.67, y: 2.54, z: 1.80},
 //     {timestamp: 1, x: 0.59, y: 2.27, z: 0.20},
 //     {timestamp: 1, x: 4.70, y: 3.38, z: 0.20},
 //     {timestamp: 1, x: 4.70, y: 1.14, z: 0.20},
 //   },
 //
 //   .combinedAnchorPositionOk = true,
};

static lpsTdoa2AlgoOptions_t* options = &defaultOptions;

// State
typedef struct {
  rangePacket2_t packet;
  dwTime_t arrival;
  double clockCorrection_T_To_A;

  uint32_t anchorStatusTimeout;
} history_t;

static uint8_t previousAnchor;
// Holds data for the latest packet from all anchors
static history_t history[LOCODECK_NR_OF_TDOA2_ANCHORS];


// LPP packet handling
static lpsLppShortPacket_t lppPacket;
static bool lppPacketToSend;
static int lppPacketSendTryCounter;

static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data);

// Log data
static float logUwbTdoaDistDiff[LOCODECK_NR_OF_TDOA2_ANCHORS];
static float logClockCorrection[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint16_t logAnchorDistance[LOCODECK_NR_OF_TDOA2_ANCHORS];


static struct {
  uint32_t packetsReceived;
  uint32_t packetsSeqNrPass;
  uint32_t packetsDataPass;
  uint32_t packetsToEstimator;

  uint16_t packetsReceivedRate;
  uint16_t packetsSeqNrPassRate;
  uint16_t packetsDataPassRate;
  uint32_t packetsToEstimatorRate;

  uint32_t nextStatisticsTime;
  uint32_t previousStatisticsTime;

  uint8_t lastAnchor0Seq;
  uint32_t lastAnchor0RxTick;
} stats;

static bool rangingOk;

static void clearStats() {
  stats.packetsReceived = 0;
  stats.packetsSeqNrPass = 0;
  stats.packetsDataPass = 0;
  stats.packetsToEstimator = 0;
}

static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static void enqueueTDOA(uint8_t anchorA, uint8_t anchorB, double distanceDiff) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff,

    .anchorPosition[0] = options->anchorPosition[anchorA],
    .anchorPosition[1] = options->anchorPosition[anchorB]
  };

  if (options->combinedAnchorPositionOk ||
      (options->anchorPosition[anchorA].timestamp && options->anchorPosition[anchorB].timestamp)) {
    stats.packetsToEstimator++;
    estimatorEnqueueTDOA(&tdoa);
  }
}

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
static bool isValidTimeStamp(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

static bool isSeqNrConsecutive(uint8_t prevSeqNr, uint8_t currentSeqNr) {
  return (currentSeqNr == ((prevSeqNr + 1) & 0xff));
}

// A note on variable names. They might seem a bit verbose but express quite a lot of information
// We have three actors: Reference anchor (Ar), Anchor n (An) and the deck on the CF called Tag (T)
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Reference
// Anchor by Anchor N expressed in the clock of Anchor N"

static bool calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket2_t* packet, const dwTime_t* arrival) {

  if (! isSeqNrConsecutive(history[anchor].packet.sequenceNrs[anchor], packet->sequenceNrs[anchor])) {
    return false;
  }

  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t latest_rxAn_by_T_in_cl_T = history[anchor].arrival.full;
  const int64_t latest_txAn_in_cl_An = history[anchor].packet.timestamps[anchor];

  const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - latest_txAn_in_cl_An);
  const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);

  *clockCorrection = frameTime_in_cl_An / frameTime_in_T;
  return true;
}

static bool calcDistanceDiff(float* tdoaDistDiff, const uint8_t previousAnchor, const uint8_t anchor, const rangePacket2_t* packet, const dwTime_t* arrival) {
  const bool isSeqNrInTagOk = isSeqNrConsecutive(history[anchor].packet.sequenceNrs[previousAnchor], packet->sequenceNrs[previousAnchor]);
  const bool isSeqNrInAnchorOk = isSeqNrConsecutive(history[anchor].packet.sequenceNrs[anchor], packet->sequenceNrs[anchor]);
  if (! (isSeqNrInTagOk && isSeqNrInAnchorOk)) {
    return false;
  }
  stats.packetsSeqNrPass++;

  const int64_t rxAn_by_T_in_cl_T  = arrival->full;
  const int64_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];
  const int64_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
  const double clockCorrection = history[anchor].clockCorrection_T_To_A;

  const bool isAnchorDistanceOk = isValidTimeStamp(tof_Ar_to_An_in_cl_An);
  const bool isRxTimeInTagOk = isValidTimeStamp(rxAr_by_An_in_cl_An);
  const bool isClockCorrectionOk = (clockCorrection != 0.0);

  if (! (isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
    return false;
  }
  stats.packetsDataPass++;

  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t rxAr_by_T_in_cl_T = history[previousAnchor].arrival.full;

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection - delta_txAr_to_txAn_in_cl_An;

  *tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;

  return true;
}

static void addToLog(const uint8_t anchor, const uint8_t previousAnchor, const float tdoaDistDiff, const rangePacket2_t* packet) {
  // Only store diffs for anchors when we have consecutive anchor ids. In case of packet
  // loss we can get ranging between any anchors and that messes up the graphs.
  if (((previousAnchor + 1) & 0x07) == anchor) {
    logUwbTdoaDistDiff[anchor] = tdoaDistDiff;
    logAnchorDistance[anchor] = packet->distances[previousAnchor];
  }
}

static void handleLppPacket(const int dataLength, const packet_t* rxPacket) {
  const int32_t payloadLength = dataLength - MAC802154_HEADER_LENGTH;
  const int32_t startOfLppDataInPayload = LPS_TDOA2_LPP_HEADER;
  const int32_t lppDataLength = payloadLength - startOfLppDataInPayload;

  if (lppDataLength > 0) {
    const uint8_t lppPacketHeader = rxPacket->payload[LPS_TDOA2_LPP_HEADER];
    if (lppPacketHeader == LPP_HEADER_SHORT_PACKET) {
      int srcId = -1;

      for (int i=0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
        if (rxPacket->sourceAddress == options->anchorAddress[i]) {
          srcId = i;
          break;
        }
      }

      if (srcId >= 0) {
        lpsHandleLppShortPacket(srcId, &rxPacket->payload[LPS_TDOA2_LPP_TYPE]);
      }
    }
  }
}

// Send an LPP packet, the radio will automatically go back in RX mode
static void sendLppShort(dwDevice_t *dev, lpsLppShortPacket_t *packet)
{
  static packet_t txPacket;
  dwIdle(dev);

  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);

  txPacket.payload[LPS_TDOA2_TYPE_INDEX] = LPP_HEADER_SHORT_PACKET;
  memcpy(&txPacket.payload[LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX], packet->data, packet->length);

  txPacket.pan = 0xbccf;
  txPacket.sourceAddress = 0xbccf000000000000 | 0xff;
  txPacket.destAddress = options->anchorAddress[packet->dest];

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+1+packet->length);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
}

static bool rxcallback(dwDevice_t *dev) {
  stats.packetsReceived++;

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  const rangePacket2_t* packet = (rangePacket2_t*)rxPacket.payload;

  bool lppSent = false;
  if (packet->type == PACKET_TYPE_TDOA2) {
    const uint8_t anchor = rxPacket.sourceAddress & 0xff;

    // Check if we need to send the current LPP packet
    if (lppPacketToSend && lppPacket.dest == anchor) {
      sendLppShort(dev, &lppPacket);
      lppSent = true;
    }

    dwTime_t arrival = {.full = 0};
    dwGetReceiveTimestamp(dev, &arrival);

    if (anchor < LOCODECK_NR_OF_TDOA2_ANCHORS) {

#ifdef LPS_TDOA2_SYNCHRONIZATION_VARIABLE
      // Storing timing
      if (anchor == 0) {
        stats.lastAnchor0Seq = packet->sequenceNrs[anchor];
        stats.lastAnchor0RxTick = xTaskGetTickCount();
      }
#endif

      calcClockCorrection(&history[anchor].clockCorrection_T_To_A, anchor, packet, &arrival);
      logClockCorrection[anchor] = history[anchor].clockCorrection_T_To_A;

      if (anchor != previousAnchor) {
        float tdoaDistDiff = 0.0;
        if (calcDistanceDiff(&tdoaDistDiff, previousAnchor, anchor, packet, &arrival)) {
          rangingOk = true;
          enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);
          addToLog(anchor, previousAnchor, tdoaDistDiff, packet);
        }
      }

      history[anchor].arrival.full = arrival.full;
      memcpy(&history[anchor].packet, packet, sizeof(rangePacket2_t));

      history[anchor].anchorStatusTimeout = xTaskGetTickCount() + ANCHOR_OK_TIMEOUT;

      previousAnchor = anchor;

      handleLppPacket(dataLength, &rxPacket);
    }
  }

  return lppSent;
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      if (rxcallback(dev)) {
        lppPacketToSend = false;
      } else {
        setRadioInReceiveMode(dev);

        // Discard lpp packet if we cannot send it for too long
        if (++lppPacketSendTryCounter >= TDOA2_LPP_PACKET_SEND_TIMEOUT) {
          lppPacketToSend = false;
        }
      }

      if (!lppPacketToSend) {
        // Get next lpp packet
        lppPacketToSend = lpsGetLppShort(&lppPacket);
        lppPacketSendTryCounter = 0;
      }
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventReceiveTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventPacketSent:
      // Service packet sent, the radio is back to receive automatically
      break;
    default:
      ASSERT_FAILED();
  }

  uint32_t now = xTaskGetTickCount();
  if (now > stats.nextStatisticsTime) {
    float interval = now - stats.previousStatisticsTime;
    stats.packetsReceivedRate = (uint16_t)(1000.0f * stats.packetsReceived / interval);
    stats.packetsSeqNrPassRate = (uint16_t)(1000.0f * stats.packetsSeqNrPass / interval);
    stats.packetsDataPassRate = (uint16_t)(1000.0f * stats.packetsDataPass / interval);
    stats.packetsToEstimatorRate = (uint16_t)(1000.0f * stats.packetsToEstimator / interval);

    clearStats();
    stats.previousStatisticsTime = now;
    stats.nextStatisticsTime = now + STATS_INTERVAL;
  }

  uint16_t rangingState = 0;
  for (int anchor = 0; anchor < LOCODECK_NR_OF_TDOA2_ANCHORS; anchor++) {
    if (now < history[anchor].anchorStatusTimeout) {
      rangingState |= (1 << anchor);
    }
  }
  locoDeckSetRangingState(rangingState);

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev) {
  // Reset module state. Needed by unit tests
  memset(history, 0, sizeof(history));

  memset(logClockCorrection, 0, sizeof(logClockCorrection));
  memset(logAnchorDistance, 0, sizeof(logAnchorDistance));
  memset(logUwbTdoaDistDiff, 0, sizeof(logUwbTdoaDistDiff));

  previousAnchor = 0;

  lppPacketToSend = false;

  locoDeckSetRangingState(0);

  clearStats();
  stats.packetsReceivedRate = 0;
  stats.packetsSeqNrPassRate = 0;
  stats.packetsDataPassRate = 0;
  stats.packetsToEstimatorRate = 0;
  stats.nextStatisticsTime = xTaskGetTickCount() + STATS_INTERVAL;
  stats.previousStatisticsTime = 0;

  dwSetReceiveWaitTimeout(dev, TDOA2_RECEIVE_TIMEOUT);

  dwCommitConfiguration(dev);

  rangingOk = false;
}
#pragma GCC diagnostic pop

static bool isRangingOk()
{
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < LOCODECK_NR_OF_TDOA2_ANCHORS) {
    *position = options->anchorPosition[anchorId];
    return true;
  }

  return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
    unorderedAnchorList[i] = i;
  }

  return LOCODECK_NR_OF_TDOA2_ANCHORS;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint32_t now = xTaskGetTickCount();
  uint8_t count = 0;

  for (int i = 0; i < LOCODECK_NR_OF_TDOA2_ANCHORS; i++) {
    if (now < history[i].anchorStatusTimeout) {
      unorderedAnchorList[count] = i;
      count++;
    }
  }

  return count;
}

// Loco Posisioning Protocol (LPP) handling
static void lpsHandleLppShortPacket(const uint8_t srcId, const uint8_t *data)
{
  uint8_t type = data[0];

  if (type == LPP_SHORT_ANCHORPOS) {
    if (srcId < LOCODECK_NR_OF_TDOA2_ANCHORS) {
      struct lppShortAnchorPos_s *newpos = (struct lppShortAnchorPos_s*)&data[1];
      options->anchorPosition[srcId].timestamp = xTaskGetTickCount();
      options->anchorPosition[srcId].x = newpos->x;
      options->anchorPosition[srcId].y = newpos->y;
      options->anchorPosition[srcId].z = newpos->z;
    }
  }
}

uwbAlgorithm_t uwbTdoa2TagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

void lpsTdoa2TagSetOptions(lpsTdoa2AlgoOptions_t* newOptions) {
  options = newOptions;
}

LOG_GROUP_START(tdoa)
LOG_ADD(LOG_FLOAT, d7-0, &logUwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d0-1, &logUwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d1-2, &logUwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d2-3, &logUwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d3-4, &logUwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d4-5, &logUwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d5-6, &logUwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d6-7, &logUwbTdoaDistDiff[7])

LOG_ADD(LOG_FLOAT, cc0, &logClockCorrection[0])
LOG_ADD(LOG_FLOAT, cc1, &logClockCorrection[1])
LOG_ADD(LOG_FLOAT, cc2, &logClockCorrection[2])
LOG_ADD(LOG_FLOAT, cc3, &logClockCorrection[3])
LOG_ADD(LOG_FLOAT, cc4, &logClockCorrection[4])
LOG_ADD(LOG_FLOAT, cc5, &logClockCorrection[5])
LOG_ADD(LOG_FLOAT, cc6, &logClockCorrection[6])
LOG_ADD(LOG_FLOAT, cc7, &logClockCorrection[7])

LOG_ADD(LOG_UINT16, dist7-0, &logAnchorDistance[0])
LOG_ADD(LOG_UINT16, dist0-1, &logAnchorDistance[1])
LOG_ADD(LOG_UINT16, dist1-2, &logAnchorDistance[2])
LOG_ADD(LOG_UINT16, dist2-3, &logAnchorDistance[3])
LOG_ADD(LOG_UINT16, dist3-4, &logAnchorDistance[4])
LOG_ADD(LOG_UINT16, dist4-5, &logAnchorDistance[5])
LOG_ADD(LOG_UINT16, dist5-6, &logAnchorDistance[6])
LOG_ADD(LOG_UINT16, dist6-7, &logAnchorDistance[7])

LOG_ADD(LOG_UINT16, stRx, &stats.packetsReceivedRate)
LOG_ADD(LOG_UINT16, stSeq, &stats.packetsSeqNrPassRate)
LOG_ADD(LOG_UINT16, stData, &stats.packetsDataPassRate)
LOG_ADD(LOG_UINT16, stEst, &stats.packetsToEstimatorRate)

#ifdef LPS_TDOA2_SYNCHRONIZATION_VARIABLE
LOG_ADD(LOG_UINT8, a0Seq, &stats.lastAnchor0Seq)
LOG_ADD(LOG_UINT32, a0RxTick, &stats.lastAnchor0RxTick)
#endif

LOG_GROUP_STOP(tdoa)
