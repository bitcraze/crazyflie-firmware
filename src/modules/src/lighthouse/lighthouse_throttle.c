
/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 */

#include <stdlib.h>
#include "lighthouse_throttle.h"
#include "param.h"

// Uncomment next line to add extra debug log variables
// #define CONFIG_DEBUG_LOG_ENABLE 1
#include "log.h"

static const uint32_t evaluationIntervalMs = 100;
static uint16_t maxRate = 50;  // Samples / second
static float discardProbability = 0.0f;

bool throttleLh2Samples(const uint32_t nowMs) {
    static uint32_t previousEvaluationTime = 0;
    static uint32_t nextEvaluationTime = 0;
    static uint32_t eventCounter = 0;
    static int discardThreshold = 0;

    eventCounter++;

    if (nowMs > nextEvaluationTime) {
        const float currentRate = 1000.0f * (float)eventCounter / (float)(nowMs - previousEvaluationTime);
        if (currentRate < (float)maxRate) {
            discardProbability = 0.0;
        } else {
            discardProbability = 1.0f - (float)maxRate / currentRate;
        }
        discardThreshold = RAND_MAX * discardProbability;

        previousEvaluationTime = nowMs;
        eventCounter = 0;
        nextEvaluationTime = nowMs + evaluationIntervalMs;
    }

    return (rand() > discardThreshold);
}

PARAM_GROUP_START(lighthouse)

/**
 * @brief Maximum rate of samples sent to the estimator
 *
 * When many LH V2 base stations are available in a system, the over all rate of samples sent to the estimator might be
 * too high to handle. This parameter sets the (approximate) maximum rate (samples/s). 50 By default.
 */
PARAM_ADD(PARAM_UINT16, lh2maxRate, &maxRate)

PARAM_GROUP_STOP(lighthouse)

LOG_GROUP_START(lighthouse)
LOG_ADD_DEBUG(LOG_FLOAT, disProb, &discardProbability)
LOG_GROUP_STOP(lighthouse)
