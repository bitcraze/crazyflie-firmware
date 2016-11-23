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

#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#endif // ESTIMATOR_TYPE_kalman


static lpsAlgoOptions_t* options;

float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];

static uint8_t previousAnchor;
static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];

static double frameTime_in_cl_A[LOCODECK_NR_OF_ANCHORS];
static double clockCorrection_T_To_A[LOCODECK_NR_OF_ANCHORS];


typedef struct {
  int64_t offset;
  int64_t latestTime;
} clockWrap_t;

#define MEASUREMENT_NOISE_STD 20.0f

// The maximum diff in distances that we consider to be valid
// Used to sanity check results and remove results that are wrong due to packet loss
#define MAX_DISTANCE_DIFF (300.0f)

static uint64_t timestampToUint64(uint8_t *ts) {
  dwTime_t timestamp = {.full = 0};
  memcpy(timestamp.raw, ts, sizeof(timestamp.raw));

  return timestamp.full;
}

static uint64_t truncateToTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & 0x00FFFFFFFFFFul;
}

#ifdef ESTIMATOR_TYPE_kalman
static void enqueueTDOA(uint8_t anchor1, uint8_t anchor2, double distanceDiff, double timeBetweenMeasurements) {
  tdoaMeasurement_t tdoa = {
    .stdDev = MEASUREMENT_NOISE_STD,
    .distanceDiff = distanceDiff,
    .timeBetweenMeasurements = timeBetweenMeasurements,

    .anchorPosition[0] = {
      .x = options->anchorPosition[anchor1].x,
      .y = options->anchorPosition[anchor1].y,
      .z = options->anchorPosition[anchor1].z
    },

    .anchorPosition[1] = {
      .x = options->anchorPosition[anchor2].x,
      .y = options->anchorPosition[anchor2].y,
      .z = options->anchorPosition[anchor2].z
    }
  };

  stateEstimatorEnqueueTDOA(&tdoa);
}
#endif

// A note on variable names. They might seem a bit verbose but express quite a lot of information
// We have three actors: Reference anchor (Ar), Anchor n (An) and the deck on the CF called Tag (T)
// rxAr_by_An_in_cl_An should be interpreted as "The time when packet was received from the Referecne Anchor by Anchor N expressed in the clock of Anchor N"
static void rxcallback(dwDevice_t *dev) {
  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  uint8_t anchor = rxPacket.sourceAddress & 0xff;

  if (anchor < LOCODECK_NR_OF_ANCHORS) {
    rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    int64_t rxAn_by_T_in_cl_T  = arrival.full;

    {
      int64_t previous_rxAr_by_T_in_cl_T  = arrivals[anchor].full;

      int64_t previous_txAr_in_cl_Ar = timestampToUint64(rxPacketBuffer[anchor].timestamps[anchor]);
      int64_t txAr_in_cl_Ar = timestampToUint64(packet->timestamps[anchor]);
      frameTime_in_cl_A[anchor] = truncateToTimeStamp(txAr_in_cl_Ar - previous_txAr_in_cl_Ar);
      double frameTime_in_T = truncateToTimeStamp(rxAn_by_T_in_cl_T - previous_rxAr_by_T_in_cl_T);

      clockCorrection_T_To_A[anchor] = 1.0;
      if (frameTime_in_T != 0.0) {
        clockCorrection_T_To_A[anchor] = frameTime_in_cl_A[anchor] / frameTime_in_T;
      }
    }

    {
      int64_t rxAr_by_T_in_cl_T  = arrivals[previousAnchor].full;

      int64_t previous_txAn_in_cl_An = timestampToUint64(rxPacketBuffer[anchor].timestamps[anchor]);
      int64_t rxAn_by_Ar_in_cl_Ar = timestampToUint64(rxPacketBuffer[previousAnchor].timestamps[anchor]);
      int64_t rxAr_by_An_in_cl_An = timestampToUint64(packet->timestamps[previousAnchor]);
      // TODO krri name reused
      int64_t txAr_in_cl_Ar = timestampToUint64(rxPacketBuffer[previousAnchor].timestamps[previousAnchor]);

      int64_t previuos_rxAr_by_An_in_cl_An = timestampToUint64(rxPacketBuffer[anchor].timestamps[previousAnchor]);

      int64_t txAn_in_cl_An = timestampToUint64(packet->timestamps[anchor]);

      double frameTime_in_cl_An = truncateToTimeStamp(rxAr_by_An_in_cl_An - previuos_rxAr_by_An_in_cl_An);

      double clockCorrection_An_To_Ar = 1.0;
      if (frameTime_in_cl_An != 0.0) {
        clockCorrection_An_To_Ar = frameTime_in_cl_A[previousAnchor] / frameTime_in_cl_An;
      }

      int64_t tof_Ar_to_An_in_cl_Ar = (((truncateToTimeStamp(rxAr_by_An_in_cl_An - previous_txAn_in_cl_An) * clockCorrection_An_To_Ar) - truncateToTimeStamp(txAr_in_cl_Ar - rxAn_by_Ar_in_cl_Ar))) / 2.0;
      int64_t delta_txAr_to_txAn_in_cl_Ar = (tof_Ar_to_An_in_cl_Ar + truncateToTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An) * clockCorrection_An_To_Ar);
      int64_t timeDiffOfArrival_in_cl_Ar =  truncateToTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) * clockCorrection_T_To_A[previousAnchor] - delta_txAr_to_txAn_in_cl_Ar;

      float tdoaDistDiff = SPEED_OF_LIGHT * timeDiffOfArrival_in_cl_Ar / LOCODECK_TS_FREQ;

      // Sanity check distances in case of missed packages
      if (tdoaDistDiff > -MAX_DISTANCE_DIFF && tdoaDistDiff < MAX_DISTANCE_DIFF) {
        uwbTdoaDistDiff[anchor] = tdoaDistDiff;
#ifdef ESTIMATOR_TYPE_kalman
        float timeBetweenMeasurements = truncateToTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) / LOCODECK_TS_FREQ;
        enqueueTDOA(previousAnchor, anchor, tdoaDistDiff, timeBetweenMeasurements);
#endif
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
  memset(uwbTdoaDistDiff, 0, sizeof(uwbTdoaDistDiff));
  memset(rxPacketBuffer, 0, sizeof(rxPacketBuffer));
  memset(arrivals, 0, sizeof(arrivals));

  memset(frameTime_in_cl_A, 0, sizeof(frameTime_in_cl_A));
  memset(clockCorrection_T_To_A, 0, sizeof(clockCorrection_T_To_A));

  previousAnchor = 0;
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
LOG_GROUP_STOP(tdoa)
