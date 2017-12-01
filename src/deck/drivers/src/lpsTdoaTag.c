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


static lpsAlgoOptions_t* options;

static float uwbTdoaDistDiff[LOCODECK_NR_OF_TDOA2_ANCHORS];
static float clockCorrectionLog[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint16_t anchorDistanceLog[LOCODECK_NR_OF_TDOA2_ANCHORS];

static uint8_t previousAnchor;
static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_TDOA2_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_TDOA2_ANCHORS];
static uint8_t sequenceNrs[LOCODECK_NR_OF_TDOA2_ANCHORS];

static double clockCorrection_T_To_A[LOCODECK_NR_OF_TDOA2_ANCHORS];


#define MEASUREMENT_NOISE_STD 0.15f
#define STATS_INTERVAL 500

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

  estimatorKalmanEnqueueTDOA(&tdoa);
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
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Reference Anchor by Anchor N expressed in the clock of Anchor N"

static bool calcClockCorrection(double* clockCorrection, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
  const int64_t previous_txAn_in_cl_An = rxPacketBuffer[anchor].timestamps[anchor];

  if (! isSeqNrConsecutive(rxPacketBuffer[anchor].sequenceNrs[anchor], packet->sequenceNrs[anchor])) {
    return false;
  }

  const int64_t rxAn_by_T_in_cl_T = arrival->full;
  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t previous_rxAn_by_T_in_cl_T = arrivals[anchor].full;
  const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
  const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);

  *clockCorrection = frameTime_in_cl_An / frameTime_in_T;
  return true;
}

static bool calcDistanceDiff(float* tdoaDistDiff, const uint8_t previousAnchor, const uint8_t anchor, const rangePacket_t* packet, const dwTime_t* arrival) {
  const int64_t rxAn_by_T_in_cl_T  = arrival->full;
  const int64_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];
  const int64_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
  const double clockCorrection = clockCorrection_T_To_A[anchor];

  const bool isSeqNrInTagOk = isSeqNrConsecutive(rxPacketBuffer[anchor].sequenceNrs[previousAnchor], packet->sequenceNrs[previousAnchor]);
  const bool isSeqNrInAnchorOk = isSeqNrConsecutive(sequenceNrs[anchor], packet->sequenceNrs[anchor]);

  if (! (isSeqNrInTagOk && isSeqNrInAnchorOk)) {
    return false;
  }
  statsPacketsSeqNrPass++;

  const bool isAnchorDistanceOk = isValidTimeStamp(tof_Ar_to_An_in_cl_An);
  const bool isRxTimeInTagOk = isValidTimeStamp(rxAr_by_An_in_cl_An);
  const bool isClockCorrectionOk = (clockCorrection != 0.0);

  if (! (isAnchorDistanceOk && isRxTimeInTagOk && isClockCorrectionOk)) {
    return false;
  }
  statsPacketsDataPass++;

  const int64_t txAn_in_cl_An = packet->timestamps[anchor];
  const int64_t rxAr_by_T_in_cl_T = arrivals[previousAnchor].full;

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection - delta_txAr_to_txAn_in_cl_An;

  *tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;

  return true;
}

static void addToLog(const uint8_t anchor, const uint8_t previousAnchor, const float tdoaDistDiff, const rangePacket_t* packet) {
  // Only store diffs for anchors with succeeding seq numbers. In case of packet
  // loss we can get ranging between any anchors and that messes up the graphs.
  if (((previousAnchor + 1) & 0x07) == anchor) {
    uwbTdoaDistDiff[anchor] = tdoaDistDiff;
    anchorDistanceLog[anchor] = packet->distances[previousAnchor];
  }
}

static void rxcallback(dwDevice_t *dev) {
  statsPacketsReceived++;

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  const uint8_t anchor = rxPacket.sourceAddress & 0xff;

  if (anchor < LOCODECK_NR_OF_TDOA2_ANCHORS) {
    const rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    calcClockCorrection(&clockCorrection_T_To_A[anchor], anchor, packet, &arrival);
    clockCorrectionLog[anchor] = clockCorrection_T_To_A[anchor];

    if (anchor != previousAnchor) {
      float tdoaDistDiff = 0.0;
      if (calcDistanceDiff(&tdoaDistDiff, previousAnchor, anchor, packet, &arrival)) {
        enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);
        addToLog(anchor, previousAnchor, tdoaDistDiff, packet);
      }
    }

    arrivals[anchor].full = arrival.full;
    memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));
    sequenceNrs[anchor] = packet->sequenceNrs[anchor];

    previousAnchor = anchor;
  }
}

static void setRadioInReceiveMode(dwDevice_t *dev) {
  dwNewReceive(dev);
  dwSetDefaults(dev);
  dwStartReceive(dev);
}

static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      rxcallback(dev);
      setRadioInReceiveMode(dev);
      break;
    case eventTimeout:
      setRadioInReceiveMode(dev);
      break;
    case eventReceiveTimeout:
      setRadioInReceiveMode(dev);
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

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;

  // Reset module state. Needed by unit tests
  memset(rxPacketBuffer, 0, sizeof(rxPacketBuffer));
  memset(arrivals, 0, sizeof(arrivals));
  memset(sequenceNrs, 0, sizeof(sequenceNrs));

  memset(clockCorrection_T_To_A, 0, sizeof(clockCorrection_T_To_A));
  memset(clockCorrectionLog, 0, sizeof(clockCorrectionLog));
  memset(anchorDistanceLog, 0, sizeof(anchorDistanceLog));

  previousAnchor = 0;

  memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));

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
LOG_ADD(LOG_FLOAT, d7-0, &uwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d0-1, &uwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d1-2, &uwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d2-3, &uwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d3-4, &uwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d4-5, &uwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d5-6, &uwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d6-7, &uwbTdoaDistDiff[7])

LOG_ADD(LOG_FLOAT, cc0, &clockCorrectionLog[0])
LOG_ADD(LOG_FLOAT, cc1, &clockCorrectionLog[1])
LOG_ADD(LOG_FLOAT, cc2, &clockCorrectionLog[2])
LOG_ADD(LOG_FLOAT, cc3, &clockCorrectionLog[3])
LOG_ADD(LOG_FLOAT, cc4, &clockCorrectionLog[4])
LOG_ADD(LOG_FLOAT, cc5, &clockCorrectionLog[5])
LOG_ADD(LOG_FLOAT, cc6, &clockCorrectionLog[6])
LOG_ADD(LOG_FLOAT, cc7, &clockCorrectionLog[7])

LOG_ADD(LOG_UINT16, dist7-0, &anchorDistanceLog[0])
LOG_ADD(LOG_UINT16, dist0-1, &anchorDistanceLog[1])
LOG_ADD(LOG_UINT16, dist1-2, &anchorDistanceLog[2])
LOG_ADD(LOG_UINT16, dist2-3, &anchorDistanceLog[3])
LOG_ADD(LOG_UINT16, dist3-4, &anchorDistanceLog[4])
LOG_ADD(LOG_UINT16, dist4-5, &anchorDistanceLog[5])
LOG_ADD(LOG_UINT16, dist5-6, &anchorDistanceLog[6])
LOG_ADD(LOG_UINT16, dist6-7, &anchorDistanceLog[7])

LOG_ADD(LOG_UINT16, stRx, &statsPacketsReceivedRate)
LOG_ADD(LOG_UINT16, stSeq, &statsPacketsSeqNrPassRate)
LOG_ADD(LOG_UINT16, stData, &statsPacketsDataPassRate)

LOG_GROUP_STOP(tdoa)
