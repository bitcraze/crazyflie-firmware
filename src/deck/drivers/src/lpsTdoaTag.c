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

#include "lpsTdoaTag.h"

#include "stabilizer_types.h"
#include "cfassert.h"

float uwbTdoaDistDiff[LOCODECK_NR_OF_ANCHORS];

static lpsAlgoOptions_t* options;

static rangePacket_t rxPacketBuffer[LOCODECK_NR_OF_ANCHORS];
static dwTime_t arrivals[LOCODECK_NR_OF_ANCHORS];

static double frameTimeInMasterClock = 0.0;
static double localClockCorrection = 1.0;

#define MASTER 0

static uint64_t dwTimestampToUint64(uint8_t *ts) {
  dwTime_t timestamp = {.full = 0};
  memcpy(timestamp.raw, ts, sizeof(timestamp.raw));

  return timestamp.full;
}

static uint32_t rxcallback(dwDevice_t *dev) {
  int dataLength = dwGetDataLength(dev);
  packet_t rxPacket;

  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  dwTime_t arrival = {.full = 0};
  dwGetReceiveTimestamp(dev, &arrival);

  locoAddress_t anchor = rxPacket.sourceAddress;

  if (anchor < LOCODECK_NR_OF_ANCHORS) {
    rangePacket_t* packet = (rangePacket_t*)rxPacket.payload;

    if (anchor == MASTER) {
      frameTimeInMasterClock = dwTimestampToUint64(packet->timestamps[MASTER]) - dwTimestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]);
      double frameTimeInLocalClock = arrival.full - arrivals[MASTER].full;

      localClockCorrection = 1.0;
      if (frameTimeInLocalClock != 0) {
        localClockCorrection = frameTimeInMasterClock / frameTimeInLocalClock;
      }
    } else {
      double frameTimeInAnchorClock = dwTimestampToUint64(packet->timestamps[MASTER]) - dwTimestampToUint64(rxPacketBuffer[anchor].timestamps[MASTER]);

      double anchorClockCorrection = 1.0;
      if (frameTimeInAnchorClock != 0) {
        anchorClockCorrection = frameTimeInMasterClock / frameTimeInAnchorClock;
      }

      int64_t txAn_X = (int64_t)((double)dwTimestampToUint64(rxPacketBuffer[anchor].timestamps[anchor]) * anchorClockCorrection);
      int64_t txA0_X = dwTimestampToUint64(rxPacketBuffer[MASTER].timestamps[MASTER]);
      int64_t rxAn_0 = dwTimestampToUint64(rxPacketBuffer[MASTER].timestamps[anchor]);
      int64_t rxA0_n = (int64_t)((double)dwTimestampToUint64(packet->timestamps[MASTER]) * anchorClockCorrection);

      int64_t rxT_0  = (int64_t)((double)arrivals[MASTER].full * localClockCorrection);
      int64_t rxT_n  = (int64_t)((double)arrival.full * localClockCorrection);
      int64_t txAn_X2 = (int64_t)((double)dwTimestampToUint64(packet->timestamps[anchor]) * anchorClockCorrection);

      int64_t tA0_n = ((rxAn_0 - txAn_X) - (txA0_X - rxA0_n)) / 2;

      int64_t tT =  (rxT_n - rxT_0) - (tA0_n + (txAn_X2 - rxA0_n));

      uwbTdoaDistDiff[anchor] = SPEED_OF_LIGHT * tT / LOCODECK_TS_FREQ;
    }

    arrivals[anchor].full = arrival.full;
    memcpy(&rxPacketBuffer[anchor], rxPacket.payload, sizeof(rangePacket_t));
  }


  return 0;
}


static uint32_t onEvent(dwDevice_t *dev, uwbEvent_t event) {
  switch(event) {
    case eventPacketReceived:
      return rxcallback(dev);
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
