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

float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];
static toaMeasurement_t lastTOA;

static lpsAlgoOptions_t* options;

static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];

static double frameTimeInMasterClock = 0.0;
static double localClockCorrection = 1.0;

#define MASTER 0

#define MEASUREMENT_NOISE_STD 0.5f

#define CAP_timer

// The maximum diff in distances that we concider to be valid
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

//tdoaMeasurement_t tdoa;
//
//tdoa.stdDev = MEASUREMENT_NOISE_STD;
//memcpy(&(tdoa.measurement[1]), &TOA, sizeof(toaMeasurement_t));
//memcpy(&(tdoa.measurement[0]), &lastTOA, sizeof(toaMeasurement_t));
//memcpy(&lastTOA, &TOA, sizeof(toaMeasurement_t));
//
//stateEstimatorEnqueueTDOA(&tdoa);
//
// From:Michael Hamer
// TOA is the current packet that has just come in
//
//toaMeasurement_t TOA = {
//    .rx = sysRxTime,
//    .tx = rxPacket.sysTxTime,
//    .x = rxPacket.x,
//    .y = rxPacket.y,
//    .z = rxPacket.z,
//    .senderId = rxPacket.senderID
//};
static void enqueueTDOA(uint8_t senderId, int64_t rxT, int64_t txAn_A0time)
{
  tdoaMeasurement_t tdoa;

  ASSERT(senderId < LOCODECK_NR_OF_ANCHORS);

  tdoa.stdDev = MEASUREMENT_NOISE_STD;
  memcpy(&(tdoa.measurement[0]), &lastTOA, sizeof(toaMeasurement_t));

  tdoa.measurement[1].senderId = senderId;
  tdoa.measurement[1].rx = rxT;
  tdoa.measurement[1].tx = txAn_A0time;
  tdoa.measurement[1].x = options->anchorPosition[senderId].x;
  tdoa.measurement[1].y = options->anchorPosition[senderId].y;
  tdoa.measurement[1].z = options->anchorPosition[senderId].z;

  memcpy(&lastTOA, &tdoa.measurement[1], sizeof(toaMeasurement_t));
#ifdef ESTIMATOR_TYPE_kalman
  stateEstimatorEnqueueTDOA(&tdoa);
#endif
}

static void rxcallback(dwDevice_t *dev) {
  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  uint8_t anchor = rxPacket.sourceAddress & 0xff;

  if (anchor < LOCODECK_NR_OF_ANCHORS) {
    rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    if (anchor == MASTER) {
      frameTimeInMasterClock = truncateToTimeStamp(timestampToUint64(packet->timestamps[MASTER]) - timestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]));
      double frameTimeInLocalClock = truncateToTimeStamp(arrival.full - arrivals[MASTER].full);

      localClockCorrection = 1.0;
      if (frameTimeInLocalClock != 0.0) {
        localClockCorrection = frameTimeInMasterClock / frameTimeInLocalClock;
      }

      int64_t txA0_X = timestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]);
      int64_t rxT_0  = arrivals[MASTER].full;
      enqueueTDOA(MASTER, rxT_0, txA0_X);
    } else {
      double frameTimeInAnchorClock = truncateToTimeStamp(timestampToUint64(packet->timestamps[MASTER]) - timestampToUint64(rxPacketBuffer[anchor].timestamps[MASTER]));

      double anchorClockCorrection = 1.0;
      if (frameTimeInAnchorClock != 0.0) {
        anchorClockCorrection = frameTimeInMasterClock / frameTimeInAnchorClock;
      }

      float tdoaDistDiff;

      int64_t txAn_X = timestampToUint64(rxPacketBuffer[anchor].timestamps[anchor]);
      int64_t txA0_X = timestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]);
      int64_t rxAn_0 = timestampToUint64(rxPacketBuffer[MASTER].timestamps[anchor]);
      int64_t rxA0_n = timestampToUint64(packet->timestamps[MASTER]);

      int64_t rxT_0  = arrivals[MASTER].full;
      int64_t rxT_n  = arrival.full;
      int64_t txAn_X2 = timestampToUint64(packet->timestamps[anchor]);


      int64_t tA0_n = (((truncateToTimeStamp(rxA0_n - txAn_X) * anchorClockCorrection) - truncateToTimeStamp(txA0_X - rxAn_0))) / 2.0;
      int64_t txAn_A0time = (tA0_n + truncateToTimeStamp(txAn_X2 - rxA0_n) * anchorClockCorrection);
      int64_t tT =  truncateToTimeStamp(rxT_n - rxT_0) * localClockCorrection - txAn_A0time;

      tdoaDistDiff = SPEED_OF_LIGHT * tT / LOCODECK_TS_FREQ;

      // Sanity check distances in case of missed packages
      if (tdoaDistDiff > -MAX_DISTANCE_DIFF && tdoaDistDiff < MAX_DISTANCE_DIFF) {
        uwbTdoaDistDiff[anchor] = tdoaDistDiff;
        enqueueTDOA(anchor, rxT_n, txAn_A0time);
      }
    }

    arrivals[anchor].full = arrival.full;
    memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));
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

static void Initialize(dwDevice_t *dev, lpsAlgoOptions_t* algoOptions) {
  options = algoOptions;
}

uwbAlgorithm_t uwbTdoaTagAlgorithm = {
  .init = Initialize,
  .onEvent = onEvent,
};


LOG_GROUP_START(tdoa)
LOG_ADD(LOG_FLOAT, d01, &uwbTdoaDistDiff[1])
LOG_ADD(LOG_FLOAT, d02, &uwbTdoaDistDiff[2])
LOG_ADD(LOG_FLOAT, d03, &uwbTdoaDistDiff[3])
LOG_ADD(LOG_FLOAT, d04, &uwbTdoaDistDiff[4])
LOG_ADD(LOG_FLOAT, d05, &uwbTdoaDistDiff[5])
LOG_ADD(LOG_FLOAT, d06, &uwbTdoaDistDiff[6])
LOG_ADD(LOG_FLOAT, d07, &uwbTdoaDistDiff[7])
LOG_GROUP_STOP(tdoa)
