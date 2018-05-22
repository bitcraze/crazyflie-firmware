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

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"

#include "lpsTdoaTagStats.h"

#define STATS_INTERVAL 500

lpsTdoaStats_t lpsTdoaStats;

static uint16_t packetsReceivedRate;
static uint16_t packetsToEstimatorRate;
static uint16_t contextHitRate;
static uint16_t contextMissRate;
static uint16_t timeIsGoodRate;
static uint16_t suitableDataFoundRate;

static uint32_t nextStatisticsTime;
static uint32_t previousStatisticsTime;

static uint8_t newAnchorId; // Used to change anchor to log, set as param
static uint8_t newRemoteAnchorId; // Used to change remote anchor to log, set as param

static uint16_t clockCorrectionRate;


void lpsTdoaStatsInit() {
  memset(&lpsTdoaStats, 0, sizeof(lpsTdoaStats));
  lpsTdoaStats.remoteAnchorId = newRemoteAnchorId = 1;

  packetsReceivedRate = 0;
  clockCorrectionRate = 0;
  nextStatisticsTime = xTaskGetTickCount() + STATS_INTERVAL;
  previousStatisticsTime = 0;

  lpsTdoaStatsClear();
}

void lpsTdoaStatsClear() {
  lpsTdoaStats.packetsReceived = 0;
  lpsTdoaStats.packetsToEstimator = 0;
  lpsTdoaStats.clockCorrectionCount = 0;
  lpsTdoaStats.contextHitCount = 0;
  lpsTdoaStats.contextMissCount = 0;
  lpsTdoaStats.timeIsGood = 0;
  lpsTdoaStats.suitableDataFound = 0;
}

void lpsTdoaStatsUpdate() {
  uint32_t now = xTaskGetTickCount();
  if (now > nextStatisticsTime) {
    float interval = now - previousStatisticsTime;
    ASSERT(interval > 0.0f);
    packetsReceivedRate = (uint16_t)(1000.0f * lpsTdoaStats.packetsReceived / interval);
    packetsToEstimatorRate = (uint16_t)(1000.0f * lpsTdoaStats.packetsToEstimator / interval);
    clockCorrectionRate = (uint16_t)(1000.0f * lpsTdoaStats.clockCorrectionCount / interval);

    contextHitRate = (uint16_t)(1000.0f * lpsTdoaStats.contextHitCount / interval);
    contextMissRate = (uint16_t)(1000.0f * lpsTdoaStats.contextMissCount / interval);

    suitableDataFoundRate = (uint16_t)(1000.0f * lpsTdoaStats.suitableDataFound / interval);
    timeIsGoodRate = (uint16_t)(1000.0f * lpsTdoaStats.timeIsGood / interval);

    if (lpsTdoaStats.anchorId != newAnchorId) {
      lpsTdoaStats.anchorId = newAnchorId;

      // Reset anchor stats
      lpsTdoaStats.clockCorrection = 0.0;
      lpsTdoaStats.tof = 0;
      lpsTdoaStats.tdoa = 0;
    }

    if (lpsTdoaStats.remoteAnchorId != newRemoteAnchorId) {
      lpsTdoaStats.remoteAnchorId = newRemoteAnchorId;

      // Reset remote anchor stats
      lpsTdoaStats.tof = 0;
      lpsTdoaStats.tdoa = 0;
    }

    lpsTdoaStatsClear();
    previousStatisticsTime = now;
    nextStatisticsTime = now + STATS_INTERVAL;
  }
}


LOG_GROUP_START(tdoa3)
LOG_ADD(LOG_UINT16, stRx, &packetsReceivedRate)
LOG_ADD(LOG_UINT16, stEst, &packetsToEstimatorRate)
LOG_ADD(LOG_UINT16, stTime, &timeIsGoodRate)
LOG_ADD(LOG_UINT16, stFound, &suitableDataFoundRate)

LOG_ADD(LOG_UINT16, stCc, &clockCorrectionRate)

LOG_ADD(LOG_UINT16, stHit, &contextHitRate)
LOG_ADD(LOG_UINT16, stMiss, &contextMissRate)

LOG_ADD(LOG_FLOAT, cc, &lpsTdoaStats.clockCorrection)
LOG_ADD(LOG_UINT16, tof, &lpsTdoaStats.tof)
LOG_ADD(LOG_FLOAT, tdoa, &lpsTdoaStats.tdoa)

LOG_GROUP_STOP(tdoa3)

PARAM_GROUP_START(tdoa3)
PARAM_ADD(PARAM_UINT8, logId, &newAnchorId)
PARAM_ADD(PARAM_UINT8, logOthrId, &newRemoteAnchorId)
PARAM_GROUP_STOP(tdoa3)
