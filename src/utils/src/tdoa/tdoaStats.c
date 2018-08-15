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
 * lpsTdoaTagStats.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with lpsTdoaTagStats.c. If not, see <http://www.gnu.org/licenses/>.
 */

/*
Log statistics for the tdoa engine
*/

#include <string.h>

#include "tdoaStats.h"

#define STATS_INTERVAL 500


static void clearStats(tdoaStats_t* tdoaStats) {
  tdoaStats->packetsReceived = 0;
  tdoaStats->packetsToEstimator = 0;
  tdoaStats->clockCorrectionCount = 0;
  tdoaStats->contextHitCount = 0;
  tdoaStats->contextMissCount = 0;
  tdoaStats->timeIsGood = 0;
  tdoaStats->suitableDataFound = 0;
}

void tdoaStatsInit(tdoaStats_t* tdoaStats, uint32_t now_ms) {
  memset(tdoaStats, 0, sizeof(tdoaStats_t));
  tdoaStats->remoteAnchorId = tdoaStats->newRemoteAnchorId = 1;

  tdoaStats->packetsReceivedRate = 0;
  tdoaStats->clockCorrectionRate = 0;
  tdoaStats->nextStatisticsTime = now_ms + STATS_INTERVAL;
  tdoaStats->previousStatisticsTime = 0;

  clearStats(tdoaStats);
}

void tdoaStatsUpdate(tdoaStats_t* tdoaStats, uint32_t now_ms) {
  if (now_ms > tdoaStats->nextStatisticsTime) {
    float interval = now_ms - tdoaStats->previousStatisticsTime;
    if (interval > 0.0f) {
      tdoaStats->packetsReceivedRate = (uint16_t)(1000.0f * tdoaStats->packetsReceived / interval);
      tdoaStats->packetsToEstimatorRate = (uint16_t)(1000.0f * tdoaStats->packetsToEstimator / interval);
      tdoaStats->clockCorrectionRate = (uint16_t)(1000.0f * tdoaStats->clockCorrectionCount / interval);

      tdoaStats->contextHitRate = (uint16_t)(1000.0f * tdoaStats->contextHitCount / interval);
      tdoaStats->contextMissRate = (uint16_t)(1000.0f * tdoaStats->contextMissCount / interval);

      tdoaStats->suitableDataFoundRate = (uint16_t)(1000.0f * tdoaStats->suitableDataFound / interval);
      tdoaStats->timeIsGoodRate = (uint16_t)(1000.0f * tdoaStats->timeIsGood / interval);
    }

    if (tdoaStats->anchorId != tdoaStats->newAnchorId) {
      tdoaStats->anchorId = tdoaStats->newAnchorId;

      // Reset anchor stats
      tdoaStats->clockCorrection = 0.0;
      tdoaStats->tof = 0;
      tdoaStats->tdoa = 0;
    }

    if (tdoaStats->remoteAnchorId != tdoaStats->newRemoteAnchorId) {
      tdoaStats->remoteAnchorId = tdoaStats->newRemoteAnchorId;

      // Reset remote anchor stats
      tdoaStats->tof = 0;
      tdoaStats->tdoa = 0;
    }

    clearStats(tdoaStats);
    tdoaStats->previousStatisticsTime = now_ms;
    tdoaStats->nextStatisticsTime = now_ms + STATS_INTERVAL;
  }
}
