/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * usec_time.c - Replacement of usec_time.h for the Simmyflie.
 *
 * The mainline usec_time.c extends a 16-bit hardware timer to 64 bits. Here
 * usecTimestamp() reads the host's CLOCK_MONOTONIC, so it returns the real
 * time elapsed since usecTimerInit() or usecTimerReset(), which the estimator
 * and the stabilizer
 * loop depend on.
 */

#include "usec_time.h"

#include <time.h>
#include <stdbool.h>

static bool isInit = false;
static struct timespec startTime;

void usecTimerInit(void)
{
  if (isInit) {
    return;
  }

  clock_gettime(CLOCK_MONOTONIC, &startTime);
  isInit = true;
}

void usecTimerReset(void)
{
  clock_gettime(CLOCK_MONOTONIC, &startTime);
}

uint64_t usecTimestamp(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);

  int64_t deltaSec = (int64_t)now.tv_sec - (int64_t)startTime.tv_sec;
  int64_t deltaNsec = (int64_t)now.tv_nsec - (int64_t)startTime.tv_nsec;

  return (uint64_t)(deltaSec * 1000000LL + deltaNsec / 1000LL);
}
