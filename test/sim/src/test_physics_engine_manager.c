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
 * test_physics_engine_manager.c - unit tests of the Physics engine manager
 * of the Simmyflie, against a mocked physics engine.
 *
 * Every engine function is stubbed with a callback that records the call, so
 * the tests can check the order of the calls and what was passed.
 */

// File under test
#include "physics_engine_manager.h"

#include <math.h>
#include <string.h>

#include "unity.h"
#include "mock_physics_engine.h"
#include "mock_beacon_sender.h"
#include "freertosMocks.h"

#define NBR_OF_ROTORS TRUE_STATE_NBR_OF_ROTORS
#define DT 0.001f
#define DEG (3.14159265358979f / 180.0f)
#define MAX_CALLS 32

// The start pose parameters and their callbacks, as param_logic.c sees them
extern float startX;
extern float startY;
extern float startYaw;
extern void startXWritten(void);
extern void startYWritten(void);
extern void startYawWritten(void);

typedef enum {
  callInit,
  callStep,
  callGetState,
  callGetStatus,
  callPlace,
} engineCall_t;

static engineCall_t calls[MAX_CALLS];
static int nbrOfCalls;

// What the mocked engine returns
static bool engineInitResult;
static trueState_t engineState;
static physicsEngineStatus_t engineStatus;

// What the mocked engine was given
static physicsEngineConfig_t initConfig;
static uint16_t stepRatios[NBR_OF_ROTORS];
static float stepDt;
static trueState_t placedState;
static int nbrOfPlaces;

static physicsEngineConfig_t config;

// What the mocked Beacon sender was given
static trueState_t beaconState;
static physicsEngineStatus_t beaconStatus;
static uint32_t beaconSimTime;
static int nbrOfBeaconTicks;

static void record(const engineCall_t call)
{
  TEST_ASSERT_LESS_THAN(MAX_CALLS, nbrOfCalls);
  calls[nbrOfCalls++] = call;
}

static bool initCallback(const physicsEngineConfig_t *c, int cmock_num_calls)
{
  (void)cmock_num_calls;
  record(callInit);
  initConfig = *c;
  return engineInitResult;
}

static void stepCallback(const uint16_t *motorRatios, float dt, int cmock_num_calls)
{
  (void)cmock_num_calls;
  record(callStep);
  memcpy(stepRatios, motorRatios, sizeof(stepRatios));
  stepDt = dt;
}

static void getStateCallback(trueState_t *state, int cmock_num_calls)
{
  (void)cmock_num_calls;
  record(callGetState);
  *state = engineState;
}

static physicsEngineStatus_t getStatusCallback(int cmock_num_calls)
{
  (void)cmock_num_calls;
  record(callGetStatus);
  return engineStatus;
}

static void placeCallback(const trueState_t *state, int cmock_num_calls)
{
  (void)cmock_num_calls;
  record(callPlace);
  placedState = *state;
  nbrOfPlaces++;
}

static void beaconTickCallback(const trueState_t *state, physicsEngineStatus_t status, uint32_t simTime,
                               int cmock_num_calls)
{
  (void)cmock_num_calls;
  beaconState = *state;
  beaconStatus = status;
  beaconSimTime = simTime;
  nbrOfBeaconTicks++;
}

static void clearCalls(void)
{
  nbrOfCalls = 0;
  nbrOfPlaces = 0;
  nbrOfBeaconTicks = 0;
}

static void assertCalls(const engineCall_t *expected, const int nbrOfExpected)
{
  TEST_ASSERT_EQUAL_INT(nbrOfExpected, nbrOfCalls);
  for (int i = 0; i < nbrOfExpected; i++) {
    TEST_ASSERT_EQUAL_INT_MESSAGE(expected[i], calls[i], "call order");
  }
}

// A state that differs from zero in every field
static trueState_t someState(void)
{
  // Yaw 30 degrees, then roll 10 degrees
  const float cy = cosf(15.0f * DEG), sy = sinf(15.0f * DEG);
  const float cr = cosf(5.0f * DEG), sr = sinf(5.0f * DEG);

  trueState_t s = {
    .position = {.x = 1.5f, .y = -2.5f, .z = 3.5f},
    .velocity = {.x = 0.1f, .y = 0.2f, .z = 0.3f},
    .acceleration = {.x = 1.0f, .y = 2.0f, .z = 3.0f},
    .attitude = {.w = cy * cr, .x = cy * sr, .y = sy * sr, .z = sy * cr},
    .angularRate = {.x = 0.4f, .y = 0.5f, .z = 0.6f},
    .rotorSpeeds = {100.0f, 200.0f, 300.0f, 400.0f},
  };
  return s;
}

static void assertStatesEqual(const trueState_t *expected, const trueState_t *actual)
{
  TEST_ASSERT_EQUAL_MEMORY(expected, actual, sizeof(trueState_t));
}

static float yawOf(const trueStateQuat_t *q)
{
  return atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}

// Init the manager with the engine at the given status, then clear the calls
static void initAt(const physicsEngineStatus_t status)
{
  engineStatus = status;
  TEST_ASSERT_TRUE(physicsEngineManagerInit(&config));
  clearCalls();
}

static void assertPlacedAtRest(void)
{
  TEST_ASSERT_EQUAL_FLOAT(config.groundLevel, placedState.position.z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.velocity.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.velocity.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.velocity.z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.acceleration.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.acceleration.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.acceleration.z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.angularRate.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.angularRate.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.angularRate.z);
  // Zero pitch and roll
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.attitude.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, placedState.attitude.y);
  const trueStateQuat_t *q = &placedState.attitude;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, q->w * q->w + q->z * q->z);
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    TEST_ASSERT_EQUAL_FLOAT(engineState.rotorSpeeds[i], placedState.rotorSpeeds[i]);
  }
}

void setUp(void)
{
  physicsEngineInit_StubWithCallback(initCallback);
  physicsEngineStep_StubWithCallback(stepCallback);
  physicsEngineGetState_StubWithCallback(getStateCallback);
  physicsEngineGetStatus_StubWithCallback(getStatusCallback);
  physicsEnginePlace_StubWithCallback(placeCallback);
  beaconSenderTick_StubWithCallback(beaconTickCallback);

  clearCalls();
  engineInitResult = true;
  engineState = someState();
  engineStatus = physicsEngineStatusAtRest;
  memset(&initConfig, 0, sizeof(initConfig));
  memset(&placedState, 0, sizeof(placedState));
  memset(stepRatios, 0xff, sizeof(stepRatios));
  stepDt = 0.0f;

  config = (physicsEngineConfig_t){
    .startPose = {.x = 0.25f, .y = -0.75f, .yaw = 30.0f * DEG},
    .groundLevel = 0.5f,
    .crashSpeedLimit = 4.0f,
    .crashAngleLimit = 45.0f * DEG,
  };
}

void tearDown(void)
{
}

// Init

void testThatInitPassesTheConfigOn(void)
{
  // Test
  physicsEngineManagerInit(&config);

  // Assert
  TEST_ASSERT_EQUAL_MEMORY(&config, &initConfig, sizeof(config));
}

void testThatInitReturnsTrueWhenTheEngineStarts(void)
{
  // Fixture
  engineInitResult = true;

  // Test
  const bool actual = physicsEngineManagerInit(&config);

  // Assert
  TEST_ASSERT_TRUE(actual);
}

void testThatInitReturnsFalseWhenTheEngineCanNotStart(void)
{
  // Fixture
  engineInitResult = false;

  // Test
  const bool actual = physicsEngineManagerInit(&config);

  // Assert
  TEST_ASSERT_FALSE(actual);
}

void testThatInitReadsStateAndStatusAfterTheEngineInit(void)
{
  // Test
  physicsEngineManagerInit(&config);

  // Assert
  const engineCall_t expected[] = {callInit, callGetState, callGetStatus};
  assertCalls(expected, 3);
}

void testThatInitFillsTheSnapshotBeforeAnyStep(void)
{
  // Fixture
  trueState_t actual;

  // Test
  physicsEngineManagerInit(&config);
  physicsEngineManagerGetTrueState(&actual);

  // Assert
  assertStatesEqual(&engineState, &actual);
}

void testThatInitFillsTheStatusBeforeAnyStep(void)
{
  // Fixture: flying at init, at rest from the first step's own read
  engineStatus = physicsEngineStatusFlying;
  physicsEngineManagerInit(&config);
  engineStatus = physicsEngineStatusAtRest;
  startXWritten();
  clearCalls();

  // Test
  physicsEngineManagerStep(DT);

  // Assert: the move is decided from the status read at init
  TEST_ASSERT_EQUAL_INT(0, nbrOfPlaces);
}

void testThatInitSetsTheStartPoseParameters(void)
{
  // Test
  physicsEngineManagerInit(&config);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(0.25f, startX);
  TEST_ASSERT_EQUAL_FLOAT(-0.75f, startY);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 30.0f, startYaw);
}

// Motor ratio latch

void testThatRatiosAreZeroBeforeAnyIsStored(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    TEST_ASSERT_EQUAL_UINT16(0, stepRatios[i]);
  }
}

void testThatAStoredRatioIsUsedByEveryFollowingStep(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  physicsEngineManagerSetMotorRatio(0, 1000);
  physicsEngineManagerSetMotorRatio(1, 2000);
  physicsEngineManagerSetMotorRatio(2, 3000);
  physicsEngineManagerSetMotorRatio(3, 65535);

  for (int step = 0; step < 3; step++) {
    // Test
    physicsEngineManagerStep(DT);

    // Assert
    TEST_ASSERT_EQUAL_UINT16(1000, stepRatios[0]);
    TEST_ASSERT_EQUAL_UINT16(2000, stepRatios[1]);
    TEST_ASSERT_EQUAL_UINT16(3000, stepRatios[2]);
    TEST_ASSERT_EQUAL_UINT16(65535, stepRatios[3]);
  }
}

void testThatAStoredRatioIsReplacedOnlyForItsMotor(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  for (uint32_t id = 0; id < NBR_OF_ROTORS; id++) {
    physicsEngineManagerSetMotorRatio(id, 1000);
  }
  physicsEngineManagerStep(DT);

  // Test
  physicsEngineManagerSetMotorRatio(2, 5000);
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(1000, stepRatios[0]);
  TEST_ASSERT_EQUAL_UINT16(1000, stepRatios[1]);
  TEST_ASSERT_EQUAL_UINT16(5000, stepRatios[2]);
  TEST_ASSERT_EQUAL_UINT16(1000, stepRatios[3]);
}

void testThatStoringARatioCallsNoEngineFunction(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);

  // Test
  for (uint32_t id = 0; id < NBR_OF_ROTORS; id++) {
    physicsEngineManagerSetMotorRatio(id, 1234);
  }

  // Assert
  TEST_ASSERT_EQUAL_INT(0, nbrOfCalls);
}

void testThatARatioForAnUnknownMotorIsIgnored(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);

  // Test
  physicsEngineManagerSetMotorRatio(NBR_OF_ROTORS, 1234);
  physicsEngineManagerStep(DT);

  // Assert
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    TEST_ASSERT_EQUAL_UINT16(0, stepRatios[i]);
  }
}

// Step

void testThatStepCallsStepGetStateGetStatusInOrder(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  const engineCall_t expected[] = {callStep, callGetState, callGetStatus};
  assertCalls(expected, 3);
}

void testThatStepPassesDtOn(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(DT, stepDt);
}

void testThatStepPlacesFirstWhenAStartPoseIsWritten(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startXWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  const engineCall_t expected[] = {callPlace, callStep, callGetState, callGetStatus};
  assertCalls(expected, 4);
}

void testThatStepUpdatesTheSnapshot(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  engineState.position.x = 9.0f;
  engineState.rotorSpeeds[3] = 1234.0f;
  trueState_t actual;

  // Test
  physicsEngineManagerStep(DT);
  physicsEngineManagerGetTrueState(&actual);

  // Assert
  assertStatesEqual(&engineState, &actual);
}

void testThatStepUpdatesTheStatus(void)
{
  // Fixture: at rest at init, flying after the first step
  initAt(physicsEngineStatusAtRest);
  engineStatus = physicsEngineStatusFlying;
  physicsEngineManagerStep(DT);
  startXWritten();
  clearCalls();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, nbrOfPlaces);
}

// Start pose move: allowed or not

void testThatAMoveIsAllowedAtRest(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startXWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, nbrOfPlaces);
}

void testThatAMoveIsAllowedWhenCrashed(void)
{
  // Fixture
  initAt(physicsEngineStatusCrashed);
  startXWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, nbrOfPlaces);
}

void testThatAMoveIsRefusedWhenFlying(void)
{
  // Fixture
  initAt(physicsEngineStatusFlying);
  startXWritten();
  startYWritten();
  startYawWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  const engineCall_t expected[] = {callStep, callGetState, callGetStatus};
  assertCalls(expected, 3);
}

// Start pose move: the placed state

void testThatOnlyAWrittenXComesFromTheParameter(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startX = 4.0f;
  startY = 5.0f;
  startYaw = 60.0f;
  startXWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(4.0f, placedState.position.x);
  TEST_ASSERT_EQUAL_FLOAT(engineState.position.y, placedState.position.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 30.0f * DEG, yawOf(&placedState.attitude));
  assertPlacedAtRest();
}

void testThatOnlyAWrittenYComesFromTheParameter(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startX = 4.0f;
  startY = 5.0f;
  startYaw = 60.0f;
  startYWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(engineState.position.x, placedState.position.x);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, placedState.position.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 30.0f * DEG, yawOf(&placedState.attitude));
  assertPlacedAtRest();
}

void testThatOnlyAWrittenYawComesFromTheParameter(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startX = 4.0f;
  startY = 5.0f;
  startYaw = -120.0f;
  startYawWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(engineState.position.x, placedState.position.x);
  TEST_ASSERT_EQUAL_FLOAT(engineState.position.y, placedState.position.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -120.0f * DEG, yawOf(&placedState.attitude));
  assertPlacedAtRest();
}

void testThatAYawOf180DegreesIsPlaced(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startYaw = 180.0f;
  startYawWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 180.0f * DEG, fabsf(yawOf(&placedState.attitude)));
  assertPlacedAtRest();
}

void testThatTwoCoordinatesWrittenBeforeOneStepGiveOnePlace(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startX = 4.0f;
  startY = 5.0f;
  startXWritten();
  startYWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, nbrOfPlaces);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, placedState.position.x);
  TEST_ASSERT_EQUAL_FLOAT(5.0f, placedState.position.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 30.0f * DEG, yawOf(&placedState.attitude));
}

void testThatAllThreeCoordinatesAreApplied(void)
{
  // Fixture
  initAt(physicsEngineStatusCrashed);
  startX = -4.0f;
  startY = -5.0f;
  startYaw = 90.0f;
  startXWritten();
  startYWritten();
  startYawWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(-4.0f, placedState.position.x);
  TEST_ASSERT_EQUAL_FLOAT(-5.0f, placedState.position.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 90.0f * DEG, yawOf(&placedState.attitude));
  assertPlacedAtRest();
}

void testThatAWriteOfTheSameValueIsStillAMove(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startX = config.startPose.x;
  startXWritten();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, nbrOfPlaces);
}

void testThatAMarkIsClearedAfterTheStep(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  startXWritten();
  physicsEngineManagerStep(DT);
  clearCalls();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, nbrOfPlaces);
}

void testThatAMarkIsClearedAfterARefusedMove(void)
{
  // Fixture: refused while flying, then at rest again
  initAt(physicsEngineStatusFlying);
  startXWritten();
  engineStatus = physicsEngineStatusAtRest;
  physicsEngineManagerStep(DT);
  clearCalls();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, nbrOfPlaces);
}

void testThatAWriteAfterAStepIsAppliedOnTheNextStep(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  physicsEngineManagerStep(DT);
  startY = 7.0f;
  startYWritten();
  clearCalls();

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(1, nbrOfPlaces);
  TEST_ASSERT_EQUAL_FLOAT(7.0f, placedState.position.y);
}

// Get true state

void testThatGetTrueStateCallsNoEngineFunction(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);
  trueState_t actual;

  // Test
  physicsEngineManagerGetTrueState(&actual);

  // Assert
  TEST_ASSERT_EQUAL_INT(0, nbrOfCalls);
}

void testThatGetTrueStateReturnsTheSnapshotNotTheEngineState(void)
{
  // Fixture: the engine's state changes, with no step since
  initAt(physicsEngineStatusAtRest);
  const trueState_t expected = engineState;
  engineState.position.x = 100.0f;
  trueState_t actual;

  // Test
  physicsEngineManagerGetTrueState(&actual);

  // Assert
  assertStatesEqual(&expected, &actual);
}

// Beacon sender

void testThatInitDoesNotTickTheBeaconSender(void)
{
  // Fixture
  engineStatus = physicsEngineStatusAtRest;

  // Test
  TEST_ASSERT_TRUE(physicsEngineManagerInit(&config));

  // Assert
  TEST_ASSERT_EQUAL_INT(0, nbrOfBeaconTicks);
}

void testThatEveryStepTicksTheBeaconSenderOnce(void)
{
  // Fixture
  initAt(physicsEngineStatusAtRest);

  // Test
  physicsEngineManagerStep(DT);
  physicsEngineManagerStep(DT);
  physicsEngineManagerStep(DT);

  // Assert
  TEST_ASSERT_EQUAL_INT(3, nbrOfBeaconTicks);
}

void testThatStepPassesTheNewStateStatusAndTickCountToTheBeaconSender(void)
{
  // Fixture: the engine's state and status change in the step
  initAt(physicsEngineStatusAtRest);
  engineState.position.x = 100.0f;
  engineStatus = physicsEngineStatusFlying;
  const trueState_t expected = engineState;

  // Test
  physicsEngineManagerStep(DT);

  // Assert
  assertStatesEqual(&expected, &beaconState);
  TEST_ASSERT_EQUAL_INT(physicsEngineStatusFlying, beaconStatus);
  TEST_ASSERT_EQUAL_UINT32(xTaskGetTickCount(), beaconSimTime);
}
