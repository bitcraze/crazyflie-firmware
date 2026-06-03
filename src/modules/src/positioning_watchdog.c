/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * positioning_watchdog.c - Liveness monitoring of positioning decks
 */

#include <stdint.h>

#include "positioning_watchdog.h"
#include "log.h"
#include "cfassert.h"

#define MAX_POSITIONING_SOURCES 4

static const positioningSource_t* sources[MAX_POSITIONING_SOURCES];
static uint8_t sourceCount = 0;

// For logging/diagnostics, updated on every fault evaluation
static uint8_t logFault = 0;
static uint8_t logAliveMap = 0;

void positioningWatchdogRegister(const positioningSource_t* source) {
  ASSERT(sourceCount < MAX_POSITIONING_SOURCES);
  sources[sourceCount] = source;
  sourceCount++;
}

bool positioningWatchdogHasFault(void) {
  bool fault = false;
  uint8_t aliveMap = 0;

  for (int i = 0; i < sourceCount; i++) {
    if (sources[i]->isAlive()) {
      aliveMap |= (1 << i);
    } else {
      fault = true;
    }
  }

  logAliveMap = aliveMap;
  logFault = fault ? 1 : 0;

  return fault;
}

/**
 * Diagnostics for the positioning deck watchdog. Monitoring of positioning decks
 * (lighthouse, loco): a deck that has been detected but stops delivering data is
 * reported as a fault to the supervisor.
 */
LOG_GROUP_START(posWd)
/**
 * @brief Nonzero if a registered positioning source has stopped delivering data
 */
LOG_ADD(LOG_UINT8, fault, &logFault)
/**
 * @brief Number of positioning sources registered (decks detected at boot)
 */
LOG_ADD(LOG_UINT8, nSrc, &sourceCount)
/**
 * @brief Bitmap of sources currently alive, bit n corresponds to registration order
 */
LOG_ADD(LOG_UINT8, aliveMap, &logAliveMap)
LOG_GROUP_STOP(posWd)
