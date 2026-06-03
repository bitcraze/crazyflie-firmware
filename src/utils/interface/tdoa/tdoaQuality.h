/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2024, Bitcraze AB
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
 * tdoaQuality.h - optional per-anchor TDoA quality logging for debugging.
 *
 * Enabled by CONFIG_DEBUG_TDOA_QUALITY. When disabled, this module is not
 * compiled in and the report/update calls are removed by the preprocessor, so
 * there is no flash, RAM or CPU overhead.
 *
 * Unlike the param-selected single-pair logs in the tdoaEngine group, this
 * exposes a snapshot of *all* tracked anchors at once: position, clock
 * correction, age, and the residual / outlier-reject statistics aggregated
 * over the anchor pairs each anchor takes part in. The data is published in the
 * "tdoaQuality" log group, indexed by snapshot slot 0..N-1, with an id<i> field
 * mapping each slot to the anchor occupying it.
 */

#ifndef __TDOA_QUALITY_H__
#define __TDOA_QUALITY_H__

#include <stdbool.h>
#include <inttypes.h>
#include "tdoaEngine.h"

/**
 * Report the result of a single TDoA measurement update to the quality
 * accumulator. Called from the Kalman TDoA measurement model for every
 * measurement, for both anchors of the pair.
 *
 * @param idA       Anchor A id of the measured pair
 * @param idB       Anchor B id of the measured pair
 * @param residual  measurement - predicted distance difference [m]
 * @param accepted  true if the outlier filter accepted the measurement
 */
void tdoaQualityReportResidual(const uint8_t idA, const uint8_t idB, const float residual, const bool accepted);

/**
 * Periodically refresh the loggable per-anchor snapshot from the engine
 * storage and the residual accumulator. Rate-limited internally, so it is safe
 * to call at the packet event rate. Resets the residual/reject window each time
 * a slot is refreshed.
 *
 * @param engineState  The TDoA engine state to read anchor storage from
 * @param now_ms       Current system time [ms]
 */
void tdoaQualityUpdate(tdoaEngineState_t* engineState, const uint32_t now_ms);

#endif // __TDOA_QUALITY_H__
