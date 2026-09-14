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
 * sensors_sim.c - sensors.h backend for CONFIG_PLATFORM_SIM (Simmyflie),
 * Phase 4.7.
 *
 * EXPLICITLY TEMPORARY (see implementation-plan-phase-4.md's 4.7 section):
 * this is a fixed-value stand-in for the real Sensor shim, narrower than the
 * design spec's own target ("passes through Physics model's true state with
 * no noise/error injection") because there is no Physics model yet -- that's
 * Phase 5's job. Phase 5 replaces this file wholesale rather than extending
 * it, same spirit as Phase 3's phase3_verify_mock.c being retired outright
 * rather than grown.
 *
 * sensors.h itself (unlike motors.h/pm.h/platform.h) only pulls in
 * stabilizer_types.h -- no STM32 hardware chain -- so it's included
 * normally here, no forward-declare workaround needed.
 *
 * Fixed values represent a stationary, level, on-the-ground Crazyflie:
 * acc = (0, 0, 1) g (gravity on Z, matching every real accelerometer driver's
 * convention -- see e.g. sensors_bmi088_bmp3xx.c), gyro = (0, 0, 0) deg/s (no
 * rotation), baro = fixed sea-level standard values. No magnetometer: matches
 * real hardware with no deck attached (default CF2.1 has no onboard mag
 * either -- sensorsReadMag() returns false and leaves *mag untouched, same
 * contract the real no-mag drivers use).
 */

#include "sensors.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usec_time.h"

static bool isInit = false;
static bool suspended = false;

static const Axis3f fixedGyro = {.x = 0.0f, .y = 0.0f, .z = 0.0f};
static const Axis3f fixedAcc = {.x = 0.0f, .y = 0.0f, .z = 1.0f};
static const baro_t fixedBaro = {
  .pressure = 1013.25f,
  .temperature = 25.0f,
  .asl = 0.0f,
};

void sensorsInit(void)
{
  isInit = true;
}

bool sensorsTest(void)
{
  return isInit;
}

bool sensorsAreCalibrated(void)
{
  return true;
}

bool sensorsManufacturingTest(void)
{
  return true;
}

void sensorsAcquire(sensorData_t *sensors)
{
  sensors->gyro = fixedGyro;
  sensors->acc = fixedAcc;
  sensors->mag = (Axis3f){.x = 0.0f, .y = 0.0f, .z = 0.0f};
  sensors->baro = fixedBaro;
  sensors->interruptTimestamp = usecTimestamp();
}

void sensorsWaitDataReady(void)
{
  vTaskDelay(pdMS_TO_TICKS(1));
}

bool sensorsReadGyro(Axis3f *gyro)
{
  *gyro = fixedGyro;
  return true;
}

bool sensorsReadAcc(Axis3f *acc)
{
  *acc = fixedAcc;
  return true;
}

bool sensorsReadMag(Axis3f *mag)
{
  /* No magnetometer on the default sim platform -- matches real hardware
   * with no deck attached. */
  return false;
}

bool sensorsReadBaro(baro_t *baro)
{
  *baro = fixedBaro;
  return true;
}

void sensorsSuspend(void)
{
  suspended = true;
}

void sensorsResume(void)
{
  suspended = false;
}

bool isSensorsSuspended(void)
{
  return suspended;
}

void sensorsSetAccMode(accModes accMode)
{
  (void)accMode;
}
