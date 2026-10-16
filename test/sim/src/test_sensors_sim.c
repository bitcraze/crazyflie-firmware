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
 * test_sensors_sim.c - unit tests of the Sensor shim of the Simmyflie,
 * against a mocked Physics engine manager and estimator.
 */

// File under test. Named by its path under src/: src/hal/src is not in
// gcc.yml's include paths.
// @MODULE "hal/src/sensors_sim.c"
#include "sensors.h"

#include <math.h>
#include <string.h>

#include "unity.h"
#include "mock_physics_engine_manager.h"
#include "mock_estimator.h"
#include "mock_usec_time.h"
#include "freertosMocks.h"

#define G 9.81f
#define DEG (3.14159265358979f / 180.0f)
#define TOLERANCE 1e-5f
#define MAX_MEASUREMENTS 128

static trueState_t managerState;
static int nbrOfSteps;
static float stepDt;
static int nbrOfReads;

static measurement_t measurements[MAX_MEASUREMENTS];
static int nbrOfMeasurements;

static void stepCallback(float dt, int cmock_num_calls)
{
  (void)cmock_num_calls;
  nbrOfSteps++;
  stepDt = dt;
}

static void getTrueStateCallback(trueState_t *state, int cmock_num_calls)
{
  (void)cmock_num_calls;
  nbrOfReads++;
  *state = managerState;
}

static void enqueueCallback(const measurement_t *measurement, int cmock_num_calls)
{
  (void)cmock_num_calls;
  TEST_ASSERT_LESS_THAN(MAX_MEASUREMENTS, nbrOfMeasurements);
  measurements[nbrOfMeasurements++] = *measurement;
}

static int countOf(const MeasurementType type)
{
  int count = 0;
  for (int i = 0; i < nbrOfMeasurements; i++) {
    if (measurements[i].type == type) {
      count++;
    }
  }
  return count;
}

// At rest, level, at the origin
static trueState_t atRest(void)
{
  trueState_t s;
  memset(&s, 0, sizeof(s));
  s.attitude.w = 1.0f;
  return s;
}

static trueStateQuat_t rotationAbout(const float ax, const float ay, const float az, const float angle)
{
  const float s = sinf(0.5f * angle);
  return (trueStateQuat_t){.w = cosf(0.5f * angle), .x = ax * s, .y = ay * s, .z = az * s};
}

// One tick with the given true state, then the readings of that tick
static void tickWith(const trueState_t *state, Axis3f *gyro, Axis3f *acc)
{
  managerState = *state;
  sensorsWaitDataReady();
  sensorsReadGyro(gyro);
  sensorsReadAcc(acc);
}

static void assertAxis(const float x, const float y, const float z, const Axis3f *actual)
{
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, x, actual->x);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, y, actual->y);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, z, actual->z);
}

void setUp(void)
{
  physicsEngineManagerStep_StubWithCallback(stepCallback);
  physicsEngineManagerGetTrueState_StubWithCallback(getTrueStateCallback);
  estimatorEnqueue_StubWithCallback(enqueueCallback);
  usecTimestamp_IgnoreAndReturn(0);

  managerState = atRest();
  sensorsResume();
  sensorsInit();

  nbrOfSteps = 0;
  stepDt = 0.0f;
  nbrOfReads = 0;
  nbrOfMeasurements = 0;
}

void tearDown(void)
{
}

// Conversions

void testThatAtRestAndLevelTheAccelerometerReads1gUpAndTheGyroZero(void)
{
  // Fixture
  const trueState_t state = atRest();
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, 1.0f, &acc);
  assertAxis(0.0f, 0.0f, 0.0f, &gyro);
}

void testThatInFreeFallTheAccelerometerReadsZero(void)
{
  // Fixture
  trueState_t state = atRest();
  state.position.z = 2.0f;
  state.velocity.z = -3.0f;
  state.acceleration.z = -G;
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, 0.0f, &acc);
}

void testThatInFreeFallTiltedTheAccelerometerReadsZero(void)
{
  // Fixture
  trueState_t state = atRest();
  state.attitude = rotationAbout(0.6f, 0.8f, 0.0f, 50.0f * DEG);
  state.acceleration.z = -G;
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, 0.0f, &acc);
}

void testThatRolledPlus90DegreesTheAccelerometerReads1gOnBodyY(void)
{
  // Fixture: positive roll, the body y axis points up
  trueState_t state = atRest();
  state.attitude = rotationAbout(1.0f, 0.0f, 0.0f, 90.0f * DEG);
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 1.0f, 0.0f, &acc);
}

void testThatRolledMinus90DegreesTheAccelerometerReadsMinus1gOnBodyY(void)
{
  // Fixture
  trueState_t state = atRest();
  state.attitude = rotationAbout(1.0f, 0.0f, 0.0f, -90.0f * DEG);
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, -1.0f, 0.0f, &acc);
}

void testThatPitchedPlus90DegreesTheAccelerometerReadsMinus1gOnBodyX(void)
{
  // Fixture: positive rotation about body y, the body x axis points down
  trueState_t state = atRest();
  state.attitude = rotationAbout(0.0f, 1.0f, 0.0f, 90.0f * DEG);
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(-1.0f, 0.0f, 0.0f, &acc);
}

void testThatPitchedMinus90DegreesTheAccelerometerReads1gOnBodyX(void)
{
  // Fixture
  trueState_t state = atRest();
  state.attitude = rotationAbout(0.0f, 1.0f, 0.0f, -90.0f * DEG);
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(1.0f, 0.0f, 0.0f, &acc);
}

void testThatUpsideDownTheAccelerometerReadsMinus1gOnBodyZ(void)
{
  // Fixture
  trueState_t state = atRest();
  state.attitude = rotationAbout(1.0f, 0.0f, 0.0f, 180.0f * DEG);
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, -1.0f, &acc);
}

void testThatAWorldAccelerationIsRotatedIntoTheBodyFrame(void)
{
  // Fixture: yaw 90 degrees, so the body x axis points along world y and
  // world x is body -y
  trueState_t state = atRest();
  state.attitude = rotationAbout(0.0f, 0.0f, 1.0f, 90.0f * DEG);
  state.acceleration.x = 0.5f * G;
  state.acceleration.y = 0.25f * G;
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.25f, -0.5f, 1.0f, &acc);
}

void testThatAnUpwardAccelerationAddsToGravity(void)
{
  // Fixture
  trueState_t state = atRest();
  state.position.z = 1.0f;
  state.acceleration.z = 0.5f * G;
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, 1.5f, &acc);
}

void testThatTheGyroReadsDegreesPerSecondOnEachAxis(void)
{
  // Fixture
  const float expected = 57.29578f;
  trueState_t state = atRest();
  Axis3f gyro, acc;

  // Test and assert
  state.angularRate = (trueStateVec3_t){.x = 1.0f, .y = 0.0f, .z = 0.0f};
  tickWith(&state, &gyro, &acc);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, gyro.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gyro.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gyro.z);

  state.angularRate = (trueStateVec3_t){.x = 0.0f, .y = 1.0f, .z = 0.0f};
  tickWith(&state, &gyro, &acc);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gyro.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, expected, gyro.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gyro.z);

  state.angularRate = (trueStateVec3_t){.x = 0.0f, .y = 0.0f, .z = -1.0f};
  tickWith(&state, &gyro, &acc);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gyro.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, gyro.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -expected, gyro.z);
}

// Readings of the current tick

void testThatAcquireReturnsTheReadingsOfTheCurrentTick(void)
{
  // Fixture
  trueState_t state = atRest();
  state.angularRate.x = 2.0f;
  state.acceleration.z = -G;
  Axis3f gyro, acc;
  tickWith(&state, &gyro, &acc);
  sensorData_t sensors;

  // Test
  sensorsAcquire(&sensors);

  // Assert
  assertAxis(gyro.x, gyro.y, gyro.z, &sensors.gyro);
  assertAxis(0.0f, 0.0f, 0.0f, &sensors.acc);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f / DEG, sensors.gyro.x);
}

void testThatTheReadingsFollowTheTrueStateFromTickToTick(void)
{
  // Fixture
  trueState_t state = atRest();
  Axis3f gyro, acc;
  tickWith(&state, &gyro, &acc);

  // Test
  state.acceleration.z = -G;
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, 0.0f, &acc);
}

void testThatTheReadingsBeforeTheFirstTickAreThoseOfTheInitialTrueState(void)
{
  // Fixture: the manager's snapshot is valid from its init
  Axis3f gyro, acc;

  // Test
  sensorsReadGyro(&gyro);
  sensorsReadAcc(&acc);

  // Assert
  assertAxis(0.0f, 0.0f, 1.0f, &acc);
  assertAxis(0.0f, 0.0f, 0.0f, &gyro);
}

// Step and enqueue

void testThatEachWaitStepsExactlyOnceWithDt1ms(void)
{
  for (int i = 1; i <= 25; i++) {
    // Test
    sensorsWaitDataReady();

    // Assert
    TEST_ASSERT_EQUAL_INT(i, nbrOfSteps);
    TEST_ASSERT_EQUAL_INT(i, nbrOfReads);
  }
  TEST_ASSERT_EQUAL_FLOAT(0.001f, stepDt);
}

void testThatEachWaitStepsExactlyOnceWhileSuspended(void)
{
  // Fixture
  sensorsSuspend();

  // Test
  for (int i = 0; i < 25; i++) {
    sensorsWaitDataReady();
  }

  // Assert
  TEST_ASSERT_EQUAL_INT(25, nbrOfSteps);
  TEST_ASSERT_EQUAL_INT(0, nbrOfMeasurements);
}

void testThatTheReadingsAreUpdatedWhileSuspended(void)
{
  // Fixture
  sensorsSuspend();
  trueState_t state = atRest();
  state.acceleration.z = -G;
  Axis3f gyro, acc;

  // Test
  tickWith(&state, &gyro, &acc);

  // Assert
  assertAxis(0.0f, 0.0f, 0.0f, &acc);
}

void testThatEachWaitEnqueuesOneGyroAndOneAccelerometerMeasurement(void)
{
  // Fixture
  trueState_t state = atRest();
  state.angularRate.y = 1.0f;

  // Test
  managerState = state;
  sensorsWaitDataReady();

  // Assert
  TEST_ASSERT_EQUAL_INT(2, nbrOfMeasurements);
  TEST_ASSERT_EQUAL_INT(MeasurementTypeGyroscope, measurements[0].type);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f / DEG, measurements[0].data.gyroscope.gyro.y);
  TEST_ASSERT_EQUAL_INT(MeasurementTypeAcceleration, measurements[1].type);
  TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 1.0f, measurements[1].data.acceleration.acc.z);
}

void testThatThePoseIsEnqueuedOnEveryTenthWaitOnly(void)
{
  for (int i = 1; i <= 35; i++) {
    // Fixture
    const int before = countOf(MeasurementTypePose);

    // Test
    sensorsWaitDataReady();

    // Assert
    const int expected = (i % 10 == 0) ? 1 : 0;
    TEST_ASSERT_EQUAL_INT_MESSAGE(expected, countOf(MeasurementTypePose) - before, "pose enqueued");
  }
  TEST_ASSERT_EQUAL_INT(3, countOf(MeasurementTypePose));
  TEST_ASSERT_EQUAL_INT(35, countOf(MeasurementTypeGyroscope));
  TEST_ASSERT_EQUAL_INT(35, countOf(MeasurementTypeAcceleration));
}

void testThatThePoseIsEnqueuedAfterTheImuMeasurements(void)
{
  // Test
  for (int i = 0; i < 10; i++) {
    sensorsWaitDataReady();
  }

  // Assert
  TEST_ASSERT_EQUAL_INT(MeasurementTypeGyroscope, measurements[nbrOfMeasurements - 3].type);
  TEST_ASSERT_EQUAL_INT(MeasurementTypeAcceleration, measurements[nbrOfMeasurements - 2].type);
  TEST_ASSERT_EQUAL_INT(MeasurementTypePose, measurements[nbrOfMeasurements - 1].type);
}

void testThatThePoseIsTheTruePositionAndQuaternion(void)
{
  // Fixture
  trueState_t state = atRest();
  state.position = (trueStateVec3_t){.x = 1.25f, .y = -2.5f, .z = 0.75f};
  state.attitude = (trueStateQuat_t){.w = 0.5f, .x = -0.5f, .y = 0.5f, .z = 0.5f};
  managerState = state;

  // Test
  for (int i = 0; i < 10; i++) {
    sensorsWaitDataReady();
  }

  // Assert
  const poseMeasurement_t *pose = &measurements[nbrOfMeasurements - 1].data.pose;
  TEST_ASSERT_EQUAL_FLOAT(1.25f, pose->x);
  TEST_ASSERT_EQUAL_FLOAT(-2.5f, pose->y);
  TEST_ASSERT_EQUAL_FLOAT(0.75f, pose->z);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose->quat.w);
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, pose->quat.x);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose->quat.y);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose->quat.z);
  TEST_ASSERT_EQUAL_FLOAT(0.01f, pose->stdDevPos);
  TEST_ASSERT_EQUAL_FLOAT(4.5e-3f, pose->stdDevQuat);
}

void testThatNoPoseIsEnqueuedWhileSuspended(void)
{
  // Fixture
  sensorsSuspend();
  for (int i = 0; i < 9; i++) {
    sensorsWaitDataReady();
  }

  // Test
  sensorsWaitDataReady();

  // Assert
  TEST_ASSERT_EQUAL_INT(0, countOf(MeasurementTypePose));
}
