/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * tdoaEngineInstance.c - holds the tdoa engine state instance
 */

#include "tdoaEngineInstance.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"

NO_DMA_CCM_SAFE_ZERO_INIT tdoaEngineState_t tdoaEngineState;

/**
 * Log group for the TDoA engine module.
 *
 * Some of the logs in this group use the parameters tdoaEngine.logId and
 * tdoaEngine.logOthrId as selectors for which anchors to log.
 */
LOG_GROUP_START(tdoaEngine)

/**
 * @brief UWB packet receive rate [1/s]. This is the raw receive rate before data has been examined and possibly discarded.
 */
STATS_CNT_RATE_LOG_ADD(stRx, &tdoaEngineState.stats.packetsReceived)

/**
 * @brief Rate of data sent to the state estimator [1/s]. This is the rate of useful data after all checks.
 */
STATS_CNT_RATE_LOG_ADD(stEst, &tdoaEngineState.stats.packetsToEstimator)

/**
 * @brief Rate of packets with a time stamp that seems to be reasonable [1/s]
 */
STATS_CNT_RATE_LOG_ADD(stTime, &tdoaEngineState.stats.timeIsGood)

/**
 * @brief Rate of packets that could be matched with an anchor to calculate a TDoA value [1/s]
 */
STATS_CNT_RATE_LOG_ADD(stFound, &tdoaEngineState.stats.suitableDataFound)

/**
 * @brief Rate of packets where the time stamp is used to update the clock correction factor for an anchor [1/s]
 */
STATS_CNT_RATE_LOG_ADD(stCc, &tdoaEngineState.stats.clockCorrectionCount)

/**
 * @brief Rate of hits when looking up anchor contexts for packets [1/s]
 */
STATS_CNT_RATE_LOG_ADD(stHit, &tdoaEngineState.stats.contextHitCount)

/**
 * @brief Rate of misses when looking up anchor contexts for packets [1/s].
 *
 * If this number is high, the CF is receiving packets from more anchors than can be stored in the TDoA storage.
 */
STATS_CNT_RATE_LOG_ADD(stMiss, &tdoaEngineState.stats.contextMissCount)

/**
 * @brief The clock correction factor for the anchor with the id selected by the tdoaEngine.logId parameter
 */
LOG_ADD(LOG_FLOAT, cc, &tdoaEngineState.stats.clockCorrection)

/**
 * @brief The Time Of Flight from anchor A to anchor B (including antenna delay), as measured by anchor A [UWB radio ticks].
 *
 * A is selected using the tdoaEngine.logId parameter and B is selected by tdoaEngine.logOthrId.
 */
LOG_ADD(LOG_UINT16, tof, &tdoaEngineState.stats.tof)

/**
 * @brief The difference in distance to anchor A and B, as measured by the Crazyflie [m].
 *
 * A is selected using the tdoaEngine.logId parameter and B is selected by tdoaEngine.logOthrId.
 */
LOG_ADD(LOG_FLOAT, tdoa, &tdoaEngineState.stats.tdoa)
LOG_GROUP_STOP(tdoaEngine)

/**
 * The TDoA engine processes TDoA data from the Loco Positioning System.
 */
PARAM_GROUP_START(tdoaEngine)
/**
 * @brief Id of anchor used for logging, primary anchor.
 */
PARAM_ADD_CORE(PARAM_UINT8, logId, &tdoaEngineState.stats.newAnchorId)
/**
 * @brief Id of anchor used for logging, secondary anchor.
 */
PARAM_ADD_CORE(PARAM_UINT8, logOthrId, &tdoaEngineState.stats.newRemoteAnchorId)

// This variable should not be exposed as a parameter since it is changed from inside the CF FW.
// It only happens when the LPS system mode is changed to TDoA2 or TDoA3 though, and as this is
// not a frequent action, we chose to expose it anyway.
PARAM_ADD(PARAM_UINT8, matchAlgo, &tdoaEngineState.matchingAlgorithm)
PARAM_GROUP_STOP(tdoaEngine)
