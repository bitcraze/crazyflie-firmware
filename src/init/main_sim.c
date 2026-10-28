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
 * main_sim.c - entry point for CONFIG_PLATFORM_SIM (Simmyflie), following
 * main.c: platformInit(), then the real system.c:systemLaunch(), then the
 * scheduler.
 *
 * instanceInit() (see instance_sim.h) comes first: it parses the command
 * line, binds the UDP socket and starts the Physics engine manager before
 * the scheduler starts, so a port that is already in use fails the process
 * at once. comm.c:commInit() picks the socket up through udplinkInit().
 */

#include "autoconf.h"
#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "instance_sim.h"
#include "system.h"

/* Not "platform.h": it pulls in the STM32 hardware chain. See
 * platform_sim.h. */
#include "platform_sim.h"

int main(int argc, char **argv)
{
  instanceInit(argc, argv);

  int err = platformInit();
  if (err != 0) {
    // The firmware is running on the wrong hardware. Halt
    while (1);
  }

  systemLaunch();

  vTaskStartScheduler();

  // Should never reach this point!
  fprintf(stderr, "Simmyflie: scheduler returned unexpectedly\n");
  return 1;
}

/* heap_3 (malloc/free) has no notion of free heap. system.c prints this
 * value once at boot. */
size_t xPortGetFreeHeapSize(void)
{
  return 0;
}

void vApplicationMallocFailedHook(void)
{
  fprintf(stderr, "Simmyflie: malloc failed\n");
  abort();
}
