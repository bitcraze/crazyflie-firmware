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
 *
 * Phase 4.0 (Foundation HAL stubs) adds calls to the hardware-only inits
 * that block system.c's real systemInit()/systemTask() from running to
 * completion -- configblockInit()/storageInit()/adcInit()/ledInit()/
 * ledseqInit()/buzzerInit()/i2cdevInit()/usecTimerInit()/
 * watchdogNormalStartTest() -- so each has a working, linkable sim
 * implementation before later chunks (4.1-4.9) depend on it, without
 * pulling in real system.c itself yet (that's Phase 4.10's cutover). No
 * CRTP-visible behavior in this chunk: pure smoke test, results only
 * printed locally. i2cdevInit()/watchdogNormalStartTest() are declared
 * locally rather than via their real headers (i2cdev.h/watchdog.h), which
 * pull in the STM32 stm32fxxx.h register chain -- same reasoning as
 * platformInit() below; see i2cdev_sim.c/watchdog_sim.c.
 *
 * Phase 4.1 (Console) adds the real consoleInit()/console.c, the first
 * CRTP-visible piece of Firmware core proper (port 0) -- retires debug.h's
 * Phase-3-era "no console.c yet, print DEBUG_PRINT straight to stdout"
 * branch, so DEBUG_PRINT now routes through consolePrintf() like real
 * hardware. heartbeatTask's periodic line is DEBUG_PRINT'd (in addition to
 * its existing local printf) so a client connecting at any point after boot
 * -- not just in the boot-banner's narrow window -- still observes console
 * output, since Simmyflie is the UDP server and a boot-time DEBUG_PRINT sent
 * before the client's first packet has no known peer to reach yet.
 *
 * Phase 4.2 (Platform) adds the real platformserviceInit()/
 * platformservice.c (port 13), retiring phase3_verify_mock.c's Platform
 * handler -- see phase3_verify_mock.c's updated header comment. Called
 * after commInit() (needs crtpInit()'s queues) but before the mock's
 * remaining handlers no longer matter, since phase3VerifyMockInit() simply
 * doesn't register a Platform callback anymore. ch.0 (arming) is inert
 * (see supervisor_sim.c) until Commander lands in 4.9; ch.1 (version) is
 * fully real.
 *
 * Phase 4.3 (Memory) adds the real memInit()/crtpMemInit()
 * (mem.c/crtp_mem.c, port 4), retiring phase3_verify_mock.c's Memory
 * handler, fully unmodified -- CMD_INFO_NBR now reports nbr_of_mems=1 (the
 * always-registered memTester), matching real hardware with no decks
 * attached exactly, not the zero a decks-only reading of requirements.md
 * would suggest.
 *
 * Phase 4.4 (Parameters) adds the real paramInit() (param_logic.c/
 * param_task.c, port 2), retiring phase3_verify_mock.c's Param handler,
 * fully unmodified -- see tools/make/sim/linker/sim_toc_sections.ld for how
 * its _param_start/_param_stop (and paramGetDefault()'s
 * _sdata/_sidata/_stext) linker-symbol dependencies are supplied on a host
 * build with no custom ARM-style linker script. The TOC is non-empty
 * already at this point: mem.c's memTst.resetW (added in 4.3, unmodified)
 * is a real read-write parameter, so 4.4's round-trip check exercises it
 * rather than hitting the "every parameter is RONLY" known-gap case.
 *
 * Phase 4.5 (Logging) adds the real logInit() (log.c, port 5), retiring
 * phase3_verify_mock.c's Log handler -- including the CMD_RESET_LOGGING
 * ch.1 quirk Phase 3 hand-fixed in the mock, since real log.c's
 * logControlProcess() CONTROL_RESET case already answers it correctly, no
 * special-casing needed. Fully unmodified -- log.c's own _log_start/
 * _log_stop TOC-bounds are supplied by the same sim_toc_sections.ld linker
 * script 4.4 added, now with a second .log output section alongside .param.
 * The TOC is non-empty already at this point too: crtp.c's crtp.rxRate/
 * txRate and mem.c's memTst.errCntW (both unmodified, already linked in)
 * are real log variables, so no new one was needed for this chunk either.
 *
 * Phase 4.6 (Power management) adds pmInit() (pm_sim.c, a full HAL swap for
 * pm_stm32f4.c -- see that file's header comment for why pm.h itself can't
 * be used here). No CRTP port of its own: pm.vbat is just another entry in
 * log.c's TOC, fixed at 4.2V per the MVP's infinite-battery requirement.
 *
 * Phase 4.7 (Pre-Phase-5 sensor/actuator stub, EXPLICITLY TEMPORARY -- see
 * sensors_sim.c/motors_sim.c) adds sensorsInit() (sensors.h has no STM32
 * dependency, included normally) and motorsInit()/motorsSetRatio() (declared
 * locally, same "can't include the real header" reasoning as platformInit()/
 * pmInit() above -- see motors_sim.c). sensorAndActuatorStubInit() below is
 * a pure smoke test, same pattern as Phase 4.0's foundationHalStubsInit():
 * it also sets a known, distinguishable ratio on each motor at boot so the
 * "motor" LOG_GROUP has real, non-zero data to stream over cflib -- nothing
 * else calls motorsSetRatio() yet, that's Phase 4.8's stabilizer loop.
 *
 * Phase 4.8 (Estimator + controller + power distribution + stabilizer loop)
 * adds stabilizerInit() (stabilizer.c, real and fully unmodified -- its
 * three unconditional #include "motors.h"/"pm.h"/"platform.h" lines resolve
 * to sim-safe shims via include-path ordering, not a source edit; see
 * design-specification.md's "Header-shadow-avoidance" section). Unlike
 * every earlier chunk, stabilizer.h itself has no STM32 dependency, so it's
 * included normally alongside the other real headers below. No CRTP port of
 * its own: the stabilizer loop runs on its own 1kHz schedule regardless of
 * CRTP traffic, which is exactly what this chunk verifies (no setpoint
 * packets sent at all -- see stabilizer_check.py).
 */

#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "instance_sim.h"
#include "crtp.h"
#include "comm.h"
#include "udplink_sim.h"
#include "phase3_verify_mock.h"

#include "configblock.h"
#include "storage.h"
#include "adc.h"
#include "led.h"
#include "ledseq.h"
#include "buzzer.h"
#include "usec_time.h"
#include "console.h"
#include "debug.h"
#include "platformservice.h"
#include "mem.h"
#include "crtp_mem.h"
#include "param_task.h"
#include "log.h"
#include "worker.h"
#include "sensors.h"
#include "stabilizer.h"

/* Not "platform.h"/"pm.h"/"motors.h": those pull in the STM32 hardware
 * chain (directly, or via motors.h/deck.h). These _sim.h headers are each
 * the single source of truth for their _sim.c's signatures -- see
 * platform_sim.h/pm_sim.h/motors_sim.h for the full reasoning. */
#include "platform_sim.h"
#include "pm_sim.h"
#include "motors_sim.h"

/* Not "i2cdev.h"/"watchdog.h": both pull in the STM32 stm32fxxx.h register
 * chain (via i2c_drv.h, or directly) -- same reasoning as platformInit()
 * above. See i2cdev_sim.c/watchdog_sim.c. */
int i2cdevInit(void *dev);
bool watchdogNormalStartTest(void);

static void heartbeatTask(void *pvParameters)
{
  (void)pvParameters;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(1000));
    unsigned long tick = (unsigned long)xTaskGetTickCount();
    printf("Simmyflie: scheduler alive, tick=%lu\n", tick);
    fflush(stdout);
    /* Same line, over CRTP console (port 0) -- see the Phase 4.1 doc
     * comment above for why this is periodic rather than boot-once. */
    DEBUG_PRINT("Simmyflie: scheduler alive, tick=%lu\n", tick);
  }
}

/* Phase 4.0: exercise the foundation HAL stubs at boot. Pure smoke test --
 * nothing here is CRTP-visible, so results are only printed locally
 * (DEBUG_PRINT/console isn't wired in until Phase 4.1). */
static void foundationHalStubsInit(void)
{
  bool pass = true;

  pass &= (configblockInit() == 0);
  pass &= configblockTest();
  storageInit();
  pass &= storageTest();
  adcInit();
  ledInit();
  pass &= ledTest();
  ledseqInit();
  pass &= ledseqTest();
  buzzerInit();
  pass &= buzzerTest();
  pass &= (i2cdevInit(NULL) != 0);
  usecTimerInit();
  pass &= watchdogNormalStartTest();

  printf("Simmyflie: Phase 4.0 foundation HAL stubs %s (usecTimestamp=%llu)\n",
         pass ? "OK" : "FAILED",
         (unsigned long long)usecTimestamp());
  fflush(stdout);
}

/* Phase 4.7: exercise the sensor/actuator stub at boot. Pure smoke test,
 * same pattern as foundationHalStubsInit() above -- also sets a known,
 * distinguishable ratio on each motor so the "motor" LOG_GROUP has real
 * data to stream over cflib before Phase 4.8's stabilizer loop exists to
 * drive it for real. */
static void sensorAndActuatorStubInit(void)
{
  bool pass = true;

  sensorsInit();
  pass &= sensorsTest();
  pass &= sensorsAreCalibrated();

  Axis3f acc, gyro;
  baro_t baro;
  pass &= sensorsReadAcc(&acc);
  pass &= (acc.z == 1.0f);
  pass &= sensorsReadGyro(&gyro);
  pass &= (gyro.x == 0.0f && gyro.y == 0.0f && gyro.z == 0.0f);
  pass &= sensorsReadBaro(&baro);

  motorsInit(NULL);
  pass &= motorsTest();
  motorsSetRatio(0, 1000);
  motorsSetRatio(1, 2000);
  motorsSetRatio(2, 3000);
  motorsSetRatio(3, 4000);
  pass &= (motorsGetRatio(0) == 1000);
  pass &= (motorsGetRatio(3) == 4000);

  DEBUG_PRINT("Simmyflie: Phase 4.7 sensor/actuator stub %s\n",
              pass ? "OK" : "FAILED");
}

static void systemLaunch(void)
{
  crtpInit();
  udplinkInit();
  phase3VerifyMockInit();
  commInit();

  debugInit();
  consoleInit();
  DEBUG_PRINT("Simmyflie: Phase 4.1 console wired in\n");

  foundationHalStubsInit();

  workerInit();

  platformserviceInit();
  DEBUG_PRINT("Simmyflie: Phase 4.2 platform service wired in\n");

  memInit();
  crtpMemInit();
  DEBUG_PRINT("Simmyflie: Phase 4.3 memory wired in\n");

  paramInit();
  DEBUG_PRINT("Simmyflie: Phase 4.4 parameters wired in\n");

  logInit();
  DEBUG_PRINT("Simmyflie: Phase 4.5 logging wired in\n");

  pmInit();
  DEBUG_PRINT("Simmyflie: Phase 4.6 power management wired in\n");

  sensorAndActuatorStubInit();
  DEBUG_PRINT("Simmyflie: Phase 4.7 sensor/actuator stub wired in\n");

  stabilizerInit(StateEstimatorTypeAutoSelect);
  DEBUG_PRINT("Simmyflie: Phase 4.8 stabilizer loop wired in\n");

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
