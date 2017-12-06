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
 * lpsTdoaTag.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTag.c.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "lpsTdoaTag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"
#include "estimator_kalman.h"

#define MEASUREMENT_NOISE_STD 0.15f
#define STATS_INTERVAL 500
#define ANCHOR_OK_TIMEOUT 1500


// State
static lpsAlgoOptions_t* options;

static uint8_t previousAnchor;
static rangePacket_t storedPackets[LOCODECK_NR_OF_TDOA2_ANCHORS];
static dwTime_t storedArrivals[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint8_t storedSequenceNrs[LOCODECK_NR_OF_TDOA2_ANCHORS];
static double storedClockCorrection_T_To_A[LOCODECK_NR_OF_TDOA2_ANCHORS];

static uint32_t anchorStatusTimeout[LOCODECK_NR_OF_TDOA2_ANCHORS];

// LPP packet handling
lpsLppShortPacket_t lppPacket;
bool lppPacketToSend;

// Log data
static float logUwbTdoaDistDiff[LOCODECK_NR_OF_TDOA2_ANCHORS];
static float logClockCorrection[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint16_t logAnchorDistance[LOCODECK_NR_OF_TDOA2_ANCHORS];

static uint32_t statsPacketsReceived = 0;
static uint32_t statsPacketsSeqNrPass = 0;
static uint32_t statsPacketsDataPass = 0;

static uint16_t statsPacketsReceivedRate = 0;
static uint16_t statsPacketsSeqNrPassRate = 0;
static uint16_t statsPacketsDataPassRate = 0;
static uint32_t nextStatisticsTime = 0;
static uint32_t previousStatisticsTime = 0;

static void clearStats() {
  statsPacketsReceived = 0;
  statsPacketsSeqNrPass = 0;
  statsPacketsDataPass = 0;
}

static uint64_t truncateToLocalTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static uint64_t truncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFul;
}

static void enqueueTDOA(uint8_t anchor1, uint8_t anchor2, double distanceDiff) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff,

    .anchorPosition[0] = options->anchorPosition[anchor1],
    .anchorPosition[1] = options->anchorPosition[anchor2]
  };

  if (options->combinedAnchorPositionOk ||
      (options->anchorPosition[anchor1].timestamp && options->anchorPosition[anchor2].timestamp)) {
    estimatorKalmanEnqueueTDOA(&tdoa);
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

static bool calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {

  if (! isSeqNrConsecutive(storedPackets[anchor].sequenceNrs[anchor], packet->sequenceNrs[anchor])) {
    return false;
  }

  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t latest_rxAn_by_T_in_cl_T = storedArrivals[anchor].full;
  const int64_t latest_txAn_in_cl_An = storedPackets[anchor].timestamps[anchor];

  const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - latest_txAn_in_cl_An);
  const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - latest_rxAn_by_T_in_cl_T);

  *clockCorrection = frameTime_in_cl_An / frameTime_in_T;
  return true;
}

static bool calcDistanceDiff(float* tdoaDistDiff, const uint8_t previousAnchor, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
  const bool isSeqNrInTagOk = isSeqNrConsecutive(storedPackets[anchor].sequenceNrs[previousAnchor], packet->sequenceNrs[previousAnchor]);
  const bool isSeqNrInAnchorOk = isSeqNrConsecutive(storedSequenceNrs[anchor], packet->sequenceNrs[anchor]);
  if (! (isSeqNrInTagOk && isSeqNrInAnchorOk)) {
    return false;
  }
  statsPacketsSeqNrPass++;

  const int64_t rxAn_by_T_in_cl_T  = arrival->full;
  const int64_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];
  const int64_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
  const double clockCorrection = storedClockCorrection_T_To_A[anchor];

  const bool isAnchorDistanceOk = isValidTimeStamp(tof_Ar_to_An_in_cl_An);
  const bool isRxTimeInTagOk = isValidTimeStamp(rxAr_by_An_in_cl_An);
  const bool isClockCorrectionOk = (clockCorrection != 0.0);

  if (! (isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
    return false;
  }
  statsPacketsDataPass++;

  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t rxAr_by_T_in_cl_T = storedArrivals[previousAnchor].full;

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection - delta_txAr_to_txAn_in_cl_An;

  *tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;

  return true;
}

static void addToLog(const uint8_t anchor, const uint8_t previousAnchor, const float tdoaDistDiff, const rangePacket_t* packet) {
  // Only store diffs for anchors when we have consecutive anchor ids. In case of packet
  // loss we can get ranging between any anchors and that messes up the graphs.
  if (((previousAnchor + 1) & 0x07) == anchor) {
    logUwbTdoaDistDiff[anchor] = tdoaDistDiff;
    logAnchorDistance[anchor] = packet->distances[previousAnchor];
  }
}

static void handleLppPacket(const int dataLength, const packet_t* rxPacket) {
  if (dataLength - MAC802154_HEADER_LENGTH > (int)LPS_TDOA_LPP_HEADER) {
    if (rxPacket->payload[LPS_TDOA_LPP_HEADER] == LPP_HEADER_SHORT_PACKET) {
      int srcId = -1;

      for (int i=0; i<LOCODECK_NR_OF_ANCHORS; i++) {
        if (rxPacket->sourceAddress == options->anchorAddress[i]) {
          srcId = i;
          break;
        }
      }

      if (srcId >= 0) {
        lpsHandleLppShortPacket(srcId, &rxPacket->payload[LPS_TDOA_LPP_TYPE],
          dataLength - MAC802154_HEADER_LENGTH - LPS_TDOA_LPP_HEADER);
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

  txPacket.payload[LPS_TDOA2_TYPE] = LPP_HEADER_SHORT_PACKET;
  memcpy(&txPacket.payload[LPS_TDOA2_SEND_LPP_PAYLOAD], packet->data, packet->length);

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
  statsPacketsReceived++;

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  const uint8_t anchor = rxPacket.sourceAddress & 0xff;

  // Check if we need to send the current LPP packet
  bool lppSent = false;
  if (lppPacketToSend && lppPacket.dest == anchor) {
    sendLppShort(dev, &lppPacket);
    lppSent = true;
  }

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  if (anchor < LOCODECK_NR_OF_TDOA2_ANCHORS) {
    const rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    calcClockCorrection(&storedClockCorrection_T_To_A[anchor], anchor, packet, &arrival);
    logClockCorrection[anchor] = storedClockCorrection_T_To_A[anchor];

    if (anchor != previousAnchor) {
      float tdoaDistDiff = 0.0;
      if (calcDistanceDiff(&tdoaDistDiff, previousAnchor, anchor, packet, &arrival)) {
        enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);
        addToLog(anchor, previousAnchor, tdoaDistDiff, packet);
      }
    }

    storedArrivals[anchor].full = arrival.full;
    memcpy(&storedPackets[anchor], rxPacket.payload, sizeof(rangePacket_t));
    storedSequenceNrs[anchor] = packet->sequenceNrs[anchor];

    anchorStatusTimeout[anchor] = xTaskGetTickCount() + ANCHOR_OK_TIMEOUT;

    previousAnchor = anchor;

    handleLppPacket(dataLength, &rxPacket);
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
      }

      if (!lppPacketToSend) {
        // Get next lpp packet
        lppPacketToSend = lpsGetLppShort(&lppPacket);
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
  if (now > nextStatisticsTime) {
    float interval = now - previousStatisticsTime;
    statsPacketsReceivedRate = (uint16_t)(1000.0f * statsPacketsReceived / interval);
    statsPacketsSeqNrPassRate = (uint16_t)(1000.0f * statsPacketsSeqNrPass / interval);
    statsPacketsDataPassRate = (uint16_t)(1000.0f * statsPacketsDataPass / interval);

    clearStats();
    previousStatisticsTime = now;
    nextStatisticsTime = now + STATS_INTERVAL;
  }

  options->rangingState = 0;
  for (int anchor = 0; anchor < LOCODECK_NR_OF_TDOA2_ANCHORS; anchor++) {
    if (now < anchorStatusTimeout[anchor]) {
      options->rangingState |= (1 << anchor);
    }
  }


  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;

  // Reset module state. Needed by unit tests
  memset(storedPackets, 0, sizeof(storedPackets));
  memset(storedArrivals, 0, sizeof(storedArrivals));
  memset(storedSequenceNrs, 0, sizeof(storedSequenceNrs));

  memset(storedClockCorrection_T_To_A, 0, sizeof(storedClockCorrection_T_To_A));
  memset(logClockCorrection, 0, sizeof(logClockCorrection));
  memset(logAnchorDistance, 0, sizeof(logAnchorDistance));

  previousAnchor = 0;

  memset(logUwbTdoaDistDiff, 0, sizeof(logUwbTdoaDistDiff));

  options->rangingState = 0;
  memset(anchorStatusTimeout, 0, sizeof(anchorStatusTimeout));

  clearStats();
  statsPacketsReceivedRate = 0;
  statsPacketsSeqNrPassRate = 0;
  statsPacketsDataPassRate = 0;
  nextStatisticsTime = xTaskGetTickCount() + STATS_INTERVAL;
  previousStatisticsTime = 0;
}
#pragma GCC diagnostic pop

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
};


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

LOG_ADD(LOG_UINT16, stRx, &statsPacketsReceivedRate)
LOG_ADD(LOG_UINT16, stSeq, &statsPacketsSeqNrPassRate)
LOG_ADD(LOG_UINT16, stData, &statsPacketsDataPassRate)

LOG_GROUP_STOP(tdoa)
