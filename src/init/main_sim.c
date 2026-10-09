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
 * instanceInit() (see instance_sim.h) comes first: it parses the CLI port
 * argument and binds the UDP socket before the scheduler starts, so a port
 * that is already in use fails the process at once. comm.c:commInit() picks
 * the socket up through udplinkInit().
 *
 * kalmanMeasurementInjectionTask() is temporary verification scaffolding
 * and not part of the firmware core. See its own comment.
 */

#include "autoconf.h"
#include "FreeRTOSConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "instance_sim.h"
#include "system.h"
#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
#include "estimator.h"
#include "estimator_kalman.h"
#endif

/* Not "platform.h": it pulls in the STM32 hardware chain. See
 * platform_sim.h. */
#include "platform_sim.h"

#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
/* Phase 4.9 (TEMPORARY, verification scaffolding only -- see 4.9's own
 * Work item 4 in dev/implementation-plan-phase-4.md): periodically enqueues
 * a fixed, known position and yaw measurement so the Kalman estimator's
 * measurement-update path (estimatorEnqueuePosition()/estimatorEnqueueYawError()
 * -> kalman_core/mm_position.c/mm_yaw_error.c) is actually exercised, not
 * just its IMU-only prediction step -- 4.8's stabilizer_check.py proves the
 * estimator task loop runs, not that an external measurement reaches
 * kalman_core and moves the state. Safe to run regardless of which
 * estimator is active: estimatorEnqueue()'s measurementsQueue is created
 * unconditionally in stateEstimatorInit(), and an estimator that never
 * drains it (Complementary) simply never acts on these measurements -- no
 * crash, no drift, which is exactly what lets Kalman-vs-Complementary A/B
 * verification work by flipping the Kconfig estimator choice alone, no
 * source change needed to disable this. Phase 5's real physics-backed
 * "Simulated position and yaw measurements" component replaces this
 * wholesale. 4.11's cutover removed it at first; it was put back (user
 * decision) so the measurement-update path stays under test until then.
 *
 * Scope correction found while verifying this chunk: unlike position
 * (mm_position.c treats x/y/z as an absolute world-frame target, so a fixed
 * constant is the right injected value), yawErrorMeasurement_t.yawError is
 * NOT an absolute yaw target -- mm_yaw_error.c computes its innovation as
 * `this->S[KC_STATE_D2] - error->yawError`, i.e. the caller is expected to
 * already have computed the small-angle residual against the filter's own
 * current estimate (radians), the same way a real absolute-heading sensor
 * integration would. Feeding a constant absolute value in degrees (this
 * task's first draft) fed a ~45-radian innovation every 100ms into a
 * small-angle-linearized filter -- not a target, closer to a runaway
 * rotation-rate command -- and produced exactly that: an unbounded,
 * effectively arbitrary final heading. Fixed by computing the residual
 * fresh every tick from estimatorKalmanGetEstimatedRot()'s live rotation
 * matrix (yaw = atan2(R10, R00), valid since roll/pitch stay ~0 on this
 * stationary fixture), the same negative-feedback shape Phase 5's real
 * measurement models will need regardless.
 *
 * Second finding: position's stdDev started at 0.01 (very confident) --
 * with a perfectly noise-free constant measurement repeated at 10 Hz, the
 * position covariance shrinks every single update (never gets pushed back
 * up by realistic measurement noise, since there isn't any), and within
 * roughly a minute of uptime the Kalman gain underflows to exactly 0.0f in
 * float32, at which point `S[i] += K[i]*error` stops moving the state at
 * all -- observed directly as stateEstimate.z freezing bit-identically for
 * tens of seconds mid-convergence (X/Y froze too, but only after already
 * reaching their exact target, so it was invisible there). Not a firmware
 * bug -- a real sensor always has nonzero measurement noise, which is
 * exactly what re-injects enough process/measurement uncertainty to keep a
 * real EKF's gain from collapsing to zero. Raised to 0.05 -- confident
 * enough to converge well within this smoke test's observation window,
 * loose enough not to saturate float32 precision first. Phase 5's real
 * measurement models, driven by actual (noisy) simulated sensors, won't
 * need this workaround. */
static void kalmanMeasurementInjectionTask(void *pvParameters)
{
  (void)pvParameters;

  /* The measurement queue is created by stabilizerInit(), in systemTask(). */
  systemWaitStart();

  const float targetYawRad = 0.78539816f; // 45 degrees

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100));

    positionMeasurement_t position = {
      .x = 1.0f,
      .y = 2.0f,
      .z = 3.0f,
      .stdDev = 0.05f,
      .source = MeasurementSourceLocationService,
    };
    estimatorEnqueuePosition(&position);

    float rot[9];
    estimatorKalmanGetEstimatedRot(rot);
    float currentYawRad = atan2f(rot[3], rot[0]); // R[1][0], R[0][0]

    /* mm_yaw_error.c computes its Kalman innovation as
     * this->S[KC_STATE_D2] - error->yawError (prediction MINUS measurement,
     * the opposite convention from every other scalar update in
     * kalman_core.c, e.g. baro's meas - this->S[KC_STATE_Z]) -- so the
     * value to pass here is (current - target), not (target - current), for
     * the resulting D2 correction to carry the sign that rotates yaw toward
     * the target once folded into the quaternion. Confirmed empirically:
     * (target - current) here measurably diverged from the target instead
     * of converging. */
    float yawErrorRad = currentYawRad - targetYawRad;
    while (yawErrorRad > 3.14159265f) {
      yawErrorRad -= 2.0f * 3.14159265f;
    }
    while (yawErrorRad < -3.14159265f) {
      yawErrorRad += 2.0f * 3.14159265f;
    }

    yawErrorMeasurement_t yawError = {
      .yawError = yawErrorRad,
      .stdDev = 0.01f,
    };
    estimatorEnqueueYawError(&yawError);
  }
}
#endif

int main(int argc, char **argv)
{
  instanceInit(argc, argv);

  int err = platformInit();
  if (err != 0) {
    // The firmware is running on the wrong hardware. Halt
    while (1);
  }

  systemLaunch();
#ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  xTaskCreate(kalmanMeasurementInjectionTask, "kalmanInject", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
#endif

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
