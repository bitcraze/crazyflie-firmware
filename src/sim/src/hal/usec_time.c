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
 * usec_time_sim.c - usec_time.h backend for CONFIG_PLATFORM_SIM
 * (Simmyflie), Phase 4.0.
 *
 * Unlike this chunk's other stubs, this is a REAL implementation, not a
 * placeholder: later chunks (4.8's estimator/controller/stabilizer loop
 * especially) depend on usecTimestamp() returning genuine elapsed time, not
 * a constant or no-op. The real usec_time.c free-runs a 16-bit hardware
 * timer (TIM7) and software-extends it to 64 bits via an IRQ-incremented
 * high word; this uses CLOCK_MONOTONIC via clock_gettime() instead, which
 * is already the timebase Phase 0's drift spike validated for this port.
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
