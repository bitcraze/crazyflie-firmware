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
 * CrazySim's main_sitl.c pattern (minus its Gazebo-specific argv handling).
 *
 * Phase 1 build skeleton: boots the FreeRTOS scheduler with no CRTP/sim
 * components wired in yet, so systemLaunch() below is a local placeholder
 * task, not the real src/modules/src/system.c:systemLaunch() -- that one
 * pulls in comm/commander/deck/estimator and the rest of the firmware core,
 * which is Phase 4's job (see dev/implementation-plan.md in the
 * simulation_model project). This placeholder proves the entry point and
 * scheduler boot mechanics; Phase 4 replaces its body with the real call.
 *
 * Phase 2 adds instanceInit() (see instance_sim.h): CLI port argument and
 * bind-before-scheduler-start UDP socket setup / port-in-use failure
 * handling. The bound socket itself isn't used yet -- Phase 3's
 * Communication component picks it up via instanceGetSocketFd().
 *
 * Phase 3 adds crtpInit()/commInit() (the CRTP port-dispatch layer and its
 * udplinkGetLink() UDP transport) to this placeholder -- the real
 * src/modules/src/system.c:systemLaunch() normally calls these itself, but
 * that file isn't built under PLATFORM_SIM until Phase 4. The
 * phase3VerifyMockInit() call is TEMPORARY verification scaffolding (see
 * phase3_verify_mock.c) so the real `cfcli` can connect end-to-end; remove
 * it once Phase 4 adds the real platformservice.c/log.c/param.c.
 */

#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>

#include "instance_sim.h"
#include "crtp.h"
#include "comm.h"
#include "udplink_sim.h"
#include "phase3_verify_mock.h"

/* Not "platform.h": that header pulls in motors.h and the STM32 hardware
 * chain via the shared platform.c dispatcher, which platform_sim.c
 * deliberately bypasses (see platform_sim.c). */
int platformInit(void);

static void heartbeatTask(void *pvParameters)
{
  (void)pvParameters;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    printf("Simmyflie: scheduler alive, tick=%lu\n", (unsigned long)xTaskGetTickCount());
    fflush(stdout);
  }
}

static void systemLaunch(void)
{
  crtpInit();
  udplinkInit();
  phase3VerifyMockInit();
  commInit();

  xTaskCreate(heartbeatTask, "heartbeat", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
}

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

void vApplicationMallocFailedHook(void)
{
  fprintf(stderr, "Simmyflie: malloc failed\n");
  abort();
}
