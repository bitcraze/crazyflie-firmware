/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 * ranges.c: Centralize range measurements for different directions
 *           and make them available as log
 */
#include <stdint.h>

#include "log.h"

#include "range.h"
#include "stabilizer_types.h"
#include "estimator.h"

static uint16_t ranges[RANGE_T_END] = {0,};

void rangeSet(rangeDirection_t direction, float range_m)
{
  if (direction > (RANGE_T_END-1)) return;

  ranges[direction] = range_m * 1000;
}

float rangeGet(rangeDirection_t direction)
{
    if (direction > (RANGE_T_END-1)) return 0;

  return ranges[direction];
}

void rangeEnqueueDownRangeInEstimator(float distance, float stdDev, uint32_t timeStamp) {
  tofMeasurement_t tofData;
  tofData.timestamp = timeStamp;
  tofData.distance = distance;
  tofData.stdDev = stdDev;
  estimatorEnqueueTOF(&tofData);
}

/**
 * Log group for the multi ranger and Z-ranger decks
 */
LOG_GROUP_START(range)
/**
 * @brief Distance from the front sensor to an obstacle [mm]
 */
LOG_ADD_CORE(LOG_UINT16, front, &ranges[rangeFront])

/**
 * @brief Distance from the back sensor to an obstacle [mm]
 */
LOG_ADD_CORE(LOG_UINT16, back, &ranges[rangeBack])

/**
 * @brief Distance from the top sensor to an obstacle [mm]
 */
LOG_ADD_CORE(LOG_UINT16, up, &ranges[rangeUp])

/**
 * @brief Distance from the left sensor to an obstacle [mm]
 */
LOG_ADD_CORE(LOG_UINT16, left, &ranges[rangeLeft])

/**
 * @brief Distance from the right sensor to an obstacle [mm]
 */
LOG_ADD_CORE(LOG_UINT16, right, &ranges[rangeRight])

/**
 * @brief Distance from the Z-ranger (bottom) sensor to an obstacle [mm]
 */
LOG_ADD_CORE(LOG_UINT16, zrange, &ranges[rangeDown])
LOG_GROUP_STOP(range)
