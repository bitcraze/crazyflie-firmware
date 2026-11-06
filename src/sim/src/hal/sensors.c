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
 * sensors_sim.c - the Sensor shim of the Simmyflie (CONFIG_PLATFORM_SIM):
 * the sensors.h backend.
 *
 * Once per tick, in sensorsWaitDataReady(), the shim steps the physics
 * through the Physics engine manager, reads the true state and turns it into
 * the readings the firmware expects. It never calls the physics engine. The
 * readings are perfect: no noise or error is injected.
 *
 * - Gyro: the body angular rate, in deg/s.
 * - Accelerometer: the world acceleration minus the gravity vector, rotated
 *   into the body frame, in g. At rest this is 1 g on body z.
 * - Pose: every tenth tick (100 Hz) the true position and attitude are
 *   enqueued to the estimator as a pose measurement, as a motion capture
 *   system does on a real Crazyflie (crtp_localization_service.c). The
 *   standard deviations are the firmware defaults extPosStdDev and
 *   extQuatStdDev; they can not be zero although the values are exact.
 * - Barometer: fixed sea-level values. The Kalman filter ignores the
 *   barometer by default.
 * - No magnetometer: matches real hardware with no deck attached.
 *
 * While the sensors are suspended (sensorsSuspend()) the physics is still
 * stepped and the readings updated; only the enqueues are skipped, so the
 * physics never stops.
 *
 * sensors.h itself (unlike motors.h/pm.h/platform.h) only pulls in
 * stabilizer_types.h -- no STM32 hardware chain -- so it's included
 * normally here, no forward-declare workaround needed.
 */

#include "sensors.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usec_time.h"
#include "estimator.h"
#include "physics_engine_manager.h"

#define STEP_DT 0.001f                   // s, one tick
#define GRAVITY 9.81f                    // m/s^2, the built-in engine's value
#define DEG_PER_RAD (180.0f / 3.14159265358979f)
#define POSE_INTERVAL_TICKS 10           // 100 Hz
#define POSE_STD_DEV_POS 0.01f           // m, firmware default extPosStdDev
#define POSE_STD_DEV_QUAT 4.5e-3f        // firmware default extQuatStdDev

static bool isInit = false;
static bool suspended = false;
static uint32_t tickCount = 0;
static trueState_t trueState;

// The readings of the current tick
static Axis3f gyro;
static Axis3f acc;

static const baro_t fixedBaro = {
  .pressure = 1013.25f,
  .temperature = 25.0f,
  .asl = 0.0f,
};

// Gyro (deg/s) and accelerometer (g) readings from a true state
static void convertToReadings(const trueState_t *state, Axis3f *gyroOut, Axis3f *accOut)
{
  gyroOut->x = state->angularRate.x * DEG_PER_RAD;
  gyroOut->y = state->angularRate.y * DEG_PER_RAD;
  gyroOut->z = state->angularRate.z * DEG_PER_RAD;

  // Specific force in the world frame: acceleration minus gravity (0, 0, -g)
  const float fx = state->acceleration.x;
  const float fy = state->acceleration.y;
  const float fz = state->acceleration.z + GRAVITY;

  // Into the body frame with the transpose of the body to world rotation
  const float w = state->attitude.w, x = state->attitude.x, y = state->attitude.y, z = state->attitude.z;
  const float r00 = 1.0f - 2.0f * (y * y + z * z), r01 = 2.0f * (x * y - w * z), r02 = 2.0f * (x * z + w * y);
  const float r10 = 2.0f * (x * y + w * z), r11 = 1.0f - 2.0f * (x * x + z * z), r12 = 2.0f * (y * z - w * x);
  const float r20 = 2.0f * (x * z - w * y), r21 = 2.0f * (y * z + w * x), r22 = 1.0f - 2.0f * (x * x + y * y);

  accOut->x = (r00 * fx + r10 * fy + r20 * fz) / GRAVITY;
  accOut->y = (r01 * fx + r11 * fy + r21 * fz) / GRAVITY;
  accOut->z = (r02 * fx + r12 * fy + r22 * fz) / GRAVITY;
}

static void enqueuePose(const trueState_t *state)
{
  // trueStateQuat_t is w, x, y, z and quaternion_t x, y, z, w: field by field
  const poseMeasurement_t pose = {
    .x = state->position.x,
    .y = state->position.y,
    .z = state->position.z,
    .quat = {.x = state->attitude.x, .y = state->attitude.y, .z = state->attitude.z, .w = state->attitude.w},
    .stdDevPos = POSE_STD_DEV_POS,
    .stdDevQuat = POSE_STD_DEV_QUAT,
  };
  estimatorEnqueuePose(&pose);
}

void sensorsInit(void)
{
  // The manager's snapshot is valid from its init, before the first step
  physicsEngineManagerGetTrueState(&trueState);
  convertToReadings(&trueState, &gyro, &acc);
  tickCount = 0;
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
  sensors->gyro = gyro;
  sensors->acc = acc;
  sensors->mag = (Axis3f){.x = 0.0f, .y = 0.0f, .z = 0.0f};
  sensors->baro = fixedBaro;
  sensors->interruptTimestamp = usecTimestamp();
}

/* The clock of the physics, and the stand-in for the real sensor tasks
 * (e.g. sensors_bmi088_bmp3xx.c): one gyro and one accelerometer measurement
 * enqueued to the estimator per stabilizer tick. No baro enqueue: the Kalman
 * filter ignores it unless kalman.useBaroUpdate is set. */
void sensorsWaitDataReady(void)
{
  vTaskDelay(pdMS_TO_TICKS(1));

  physicsEngineManagerStep(STEP_DT);
  physicsEngineManagerGetTrueState(&trueState);
  convertToReadings(&trueState, &gyro, &acc);
  tickCount++;

  if (suspended) {
    return;
  }

  measurement_t measurement;
  measurement.type = MeasurementTypeGyroscope;
  measurement.data.gyroscope.gyro = gyro;
  estimatorEnqueue(&measurement);

  measurement.type = MeasurementTypeAcceleration;
  measurement.data.acceleration.acc = acc;
  estimatorEnqueue(&measurement);

  if (tickCount % POSE_INTERVAL_TICKS == 0) {
    enqueuePose(&trueState);
  }
}

bool sensorsReadGyro(Axis3f *gyroOut)
{
  *gyroOut = gyro;
  return true;
}

bool sensorsReadAcc(Axis3f *accOut)
{
  *accOut = acc;
  return true;
}

bool sensorsReadMag(Axis3f *mag)
{
  (void)mag;
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
