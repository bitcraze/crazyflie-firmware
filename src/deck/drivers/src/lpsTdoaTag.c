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

#include "log.h"
#include "lpsTdoaTag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

#include "estimator.h"
#include "estimator_kalman.h"


static lpsAlgoOptions_t* options;

static float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];

static uint8_t previousAnchor;
static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];

static double clockCorrection_T_To_A[LOCODECK_NR_OF_ANCHORS];


#define MEASUREMENT_NOISE_STD 0.15f

// The maximum diff in distances that we consider to be valid
// Used to sanity check results and remove results that are wrong due to packet loss
#define MAX_DISTANCE_DIFF (5.0f)

static uint32_t statsReceivedPackets = 0;
static uint32_t statsAcceptedAnchorDataPackets = 0;
static uint32_t statsAcceptedPackets = 0;

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

static double calcClockCorrection(const double frameTime, const double previuosFrameTime) {
    double clockCorrection = 1.0;

    if (frameTime != 0.0) {
      clockCorrection = previuosFrameTime / frameTime;
    }

    return clockCorrection;
}

// The default receive time in the anchors for messages from other anchors is 0
// and is overwritten with the actual receive time when a packet arrives.
// That is, if no message was received the rx time will be 0.
static bool isValidRxTime(const int64_t anchorRxTime) {
  return anchorRxTime != 0;
}

// A note on variable names. They might seem a bit verbose but express quite a lot of information
// We have three actors: Reference anchor (Ar), Anchor n (An) and the deck on the CF called Tag (T)
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Referecne Anchor by Anchor N expressed in the clock of Anchor N"
static void rxcallback(dwDevice_t *dev) {
  statsReceivedPackets++;

  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  const uint8_t anchor = rxPacket.sourceAddress & 0xff;

  if (anchor < LOCODECK_NR_OF_ANCHORS) {
    const rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    const int64_t rxAn_by_T_in_cl_T  = arrival.full;
    const int64_t txAn_in_cl_An = packet->timestamps[anchor];

    // Calculate clock correction for anchor clock compared to tag clock
    const int64_t previous_rxAn_by_T_in_cl_T  = arrivals[anchor].full;
    const int64_t previous_txAn_in_cl_An = rxPacketBuffer[anchor].timestamps[anchor];
    const double frameTime_in_T = truncateToLocalTimeStamp(rxAn_by_T_in_cl_T - previous_rxAn_by_T_in_cl_T);
    const double frameTime_in_cl_An = truncateToAnchorTimeStamp(txAn_in_cl_An - previous_txAn_in_cl_An);
    clockCorrection_T_To_A[anchor] = calcClockCorrection(frameTime_in_T, frameTime_in_cl_An);

    if (anchor != previousAnchor) {
      const int64_t rxAr_by_An_in_cl_An = packet->timestamps[previousAnchor];

      if (isValidRxTime(rxAr_by_An_in_cl_An)) {
        statsAcceptedAnchorDataPackets++;

        const int64_t rxAr_by_T_in_cl_T  = arrivals[previousAnchor].full;

        // Calculate distance diff
        const int64_t tof_Ar_to_An_in_cl_An = packet->distances[previousAnchor];
        const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + truncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
        const int64_t timeDiffOfArrival_in_cl_An =  truncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection_T_To_A[anchor] - delta_txAr_to_txAn_in_cl_An;

        const float tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_An / LOCODECK_TS_FREQ;

        uwbTdoaDistDiff[anchor] = tdoaDistDiff;

        enqueueTDOA(previousAnchor, anchor, tdoaDistDiff);

        statsAcceptedPackets++;
      }
    }

    arrivals[anchor].full = arrival.full;
    memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));

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

  return MAX_TIMEOUT;
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;

  // Reset module state. Needed by unit tests
  memset(rxPacketBuffer, 0, sizeof(rxPacketBuffer));
  memset(arrivals, 0, sizeof(arrivals));

  memset(clockCorrection_T_To_A, 0, sizeof(clockCorrection_T_To_A));

  previousAnchor = 0;

  memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));

  statsReceivedPackets = 0;
  statsAcceptedAnchorDataPackets = 0;
  statsAcceptedPackets = 0;
}
#pragma GCC diagnostic pop

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
};


LOG_GROUP_START(tdoa)
LOG_ADD(LOG_FLOAT, d0, &uwbTdoaDistDiff[0])
LOG_ADD(LOG_FLOAT, d1, &uwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d2, &uwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d3, &uwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d4, &uwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d5, &uwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d6, &uwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d7, &uwbTdoaDistDiff[7])

LOG_ADD(LOG_UINT32, rxCnt, &statsReceivedPackets)
LOG_ADD(LOG_UINT32, anCnt, &statsAcceptedAnchorDataPackets)
LOG_ADD(LOG_UINT32, okCnt, &statsAcceptedPackets)

LOG_GROUP_STOP(tdoa)
