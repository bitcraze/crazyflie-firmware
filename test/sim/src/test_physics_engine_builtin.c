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
 * test_physics_engine_builtin.c - unit tests of the built-in physics engine
 * of the Simmyflie: the physics model and the ground and crash contract.
 *
 * The mass and inertia follow CONFIG_SIM_BUILTIN_MASS_MG and
 * CONFIG_SIM_BUILTIN_PROPELLER_GUARDS, which this build takes from its
 * command line. To cover them, run the file three times:
 *
 *   make unit FILES=test/sim/src/test_physics_engine_builtin.c
 *   make unit FILES=test/sim/src/test_physics_engine_builtin.c ARCH_CFLAGS=-DCONFIG_SIM_BUILTIN_MASS_MG=50000
 *   make unit FILES=test/sim/src/test_physics_engine_builtin.c ARCH_CFLAGS=-DCONFIG_SIM_BUILTIN_PROPELLER_GUARDS=1
 */

// File under test
// @MODULE "physics_engine_builtin.c"
#include "physics_engine.h"

#include <math.h>
#include <string.h>

#include "unity.h"

#define NBR_OF_ROTORS TRUE_STATE_NBR_OF_ROTORS

// Expected values are computed here from the paper, not taken from the engine
#ifndef CONFIG_SIM_BUILTIN_MASS_MG
#define CONFIG_SIM_BUILTIN_MASS_MG 40000
#endif
#ifdef CONFIG_SIM_BUILTIN_PROPELLER_GUARDS
#define MASS ((CONFIG_SIM_BUILTIN_MASS_MG + 4000) * 1e-6)
#define INERTIA_X 3.3e-5
#else
#define MASS (CONFIG_SIM_BUILTIN_MASS_MG * 1e-6)
#define INERTIA_X 1.8e-5
#endif

#define G 9.81
#define MAX_ROTOR_SPEED 2900.0
#define HALF_WIDTH 0.03535
#define MAX_RATIO 65535
#define DT 0.001f
#define DEG (3.14159265358979 / 180.0)

static const uint16_t zeroRatios[NBR_OF_ROTORS] = {0, 0, 0, 0};
static const uint16_t fullRatios[NBR_OF_ROTORS] = {MAX_RATIO, MAX_RATIO, MAX_RATIO, MAX_RATIO};
// Total thrust is a third of the weight at 40 g
static const uint16_t lowRatios[NBR_OF_ROTORS] = {20000, 20000, 20000, 20000};

static physicsEngineConfig_t config;
static trueState_t state;

// Equation 9 of the paper, without the engine's clamp
static double thrustOf(const double rotorSpeed) {
  const double s = rotorSpeed / MAX_ROTOR_SPEED;
  return -0.23 * s * s * s + 0.562 * s * s - 0.043 * s;
}

// The rotor speed at which one rotor carries a quarter of the weight
static double hoverRotorSpeed(void) {
  double low = 0.1 * MAX_ROTOR_SPEED;
  double high = MAX_ROTOR_SPEED;
  for (int i = 0; i < 60; i++) {
    const double mid = 0.5 * (low + high);
    if (thrustOf(mid) < MASS * G / 4.0) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return low;
}

// The motor ratio that holds a rotor speed in steady state
static uint16_t ratioOf(const double rotorSpeed) {
  return (uint16_t)lround(rotorSpeed / MAX_ROTOR_SPEED * MAX_RATIO);
}

static trueStateQuat_t quatOf(const double roll, const double pitch, const double yaw) {
  const double cr = cos(0.5 * roll), sr = sin(0.5 * roll);
  const double cp = cos(0.5 * pitch), sp = sin(0.5 * pitch);
  const double cy = cos(0.5 * yaw), sy = sin(0.5 * yaw);
  return (trueStateQuat_t){
    .w = (float)(cr * cp * cy + sr * sp * sy),
    .x = (float)(sr * cp * cy - cr * sp * sy),
    .y = (float)(cr * sp * cy + sr * cp * sy),
    .z = (float)(cr * cp * sy - sr * sp * cy),
  };
}

static float yawOf(const trueStateQuat_t* q) {
  return atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}

// A level state in the air with all rotors at the same speed
static trueState_t airborne(const float height, const double rotorSpeed) {
  trueState_t s;
  memset(&s, 0, sizeof(s));
  s.position.z = height;
  s.attitude.w = 1.0f;
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    s.rotorSpeeds[i] = (float)rotorSpeed;
  }
  return s;
}

// One step. Every step of every test passes here, which checks the floor.
static void step(const uint16_t ratios[NBR_OF_ROTORS]) {
  physicsEngineStep(ratios, DT);
  physicsEngineGetState(&state);
  TEST_ASSERT_TRUE_MESSAGE(state.position.z >= config.groundLevel, "Below ground level");
}

static void steps(const uint16_t ratios[NBR_OF_ROTORS], const int count) {
  for (int i = 0; i < count; i++) {
    step(ratios);
  }
}

// Step until the status is no longer flying
static void stepsUntilOnGround(const uint16_t ratios[NBR_OF_ROTORS]) {
  for (int i = 0; i < 10000 && physicsEngineGetStatus() == physicsEngineStatusFlying; i++) {
    step(ratios);
  }
  TEST_ASSERT_NOT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
}

static void assertVec3(const float x, const float y, const float z, const trueStateVec3_t* actual) {
  TEST_ASSERT_EQUAL_FLOAT(x, actual->x);
  TEST_ASSERT_EQUAL_FLOAT(y, actual->y);
  TEST_ASSERT_EQUAL_FLOAT(z, actual->z);
}

static void assertRotorSpeeds(const float expected) {
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    TEST_ASSERT_EQUAL_FLOAT(expected, state.rotorSpeeds[i]);
  }
}

// At rest at the given place: on the ground, level, not moving
static void assertAtRestAt(const float x, const float y, const float yaw) {
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, physicsEngineGetStatus());
  assertVec3(x, y, config.groundLevel, &state.position);
  assertVec3(0.0f, 0.0f, 0.0f, &state.velocity);
  assertVec3(0.0f, 0.0f, 0.0f, &state.acceleration);
  assertVec3(0.0f, 0.0f, 0.0f, &state.angularRate);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.attitude.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, state.attitude.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, yaw, yawOf(&state.attitude));
}

static void place(const trueState_t* s) {
  physicsEnginePlace(s);
  physicsEngineGetState(&state);
}

void setUp(void) {
  config = (physicsEngineConfig_t){
    .startPose = {.x = 0.0f, .y = 0.0f, .yaw = 0.0f},
    .groundLevel = 0.0f,
    .crashSpeedLimit = 4.0f,
    .crashAngleLimit = (float)(45.0 * DEG),
  };
  physicsEngineInit(&config);
  physicsEngineGetState(&state);
}

void tearDown(void) {
  // Empty
}

// Init and start ------------------------------------------------------------

void testThatInitReturnsTrue() {
  TEST_ASSERT_TRUE(physicsEngineInit(&config));
}

void testThatStartIsAtRestAtGroundLevel() {
  // Fixture, test: in setUp()

  // Assert
  assertAtRestAt(0.0f, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, state.attitude.w);
  assertRotorSpeeds(0.0f);
}

void testThatStartIsAtStartPoseAndNonZeroGroundLevel() {
  // Fixture
  config.startPose = (startPose_t){.x = 1.5f, .y = -2.0f, .yaw = (float)(90.0 * DEG)};
  config.groundLevel = 0.75f;

  // Test
  physicsEngineInit(&config);
  physicsEngineGetState(&state);

  // Assert
  assertAtRestAt(1.5f, -2.0f, (float)(90.0 * DEG));
  TEST_ASSERT_EQUAL_FLOAT(0.75f, state.position.z);
  assertRotorSpeeds(0.0f);
}

// Stay at rest ----------------------------------------------------------------

void testThatStateStaysAtRestWithZeroRatios() {
  // Test
  for (int i = 0; i < 100; i++) {
    step(zeroRatios);

    // Assert
    assertAtRestAt(0.0f, 0.0f, 0.0f);
    assertRotorSpeeds(0.0f);
  }
}

void testThatStateStaysAtRestWithThrustBelowWeightWhileRotorSpeedsChange() {
  // Fixture
  float previousRotorSpeed = 0.0f;

  // Test
  for (int i = 0; i < 500; i++) {
    step(lowRatios);

    // Assert
    assertAtRestAt(0.0f, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(state.rotorSpeeds[0] > previousRotorSpeed);
    previousRotorSpeed = state.rotorSpeeds[0];
  }
}

void testThatStateStaysAtRestWithUnequalThrustBelowWeight() {
  // Fixture
  // Torque on all three axes
  const uint16_t ratios[NBR_OF_ROTORS] = {30000, 0, 5000, 10000};

  // Test
  for (int i = 0; i < 500; i++) {
    step(ratios);

    // Assert
    assertAtRestAt(0.0f, 0.0f, 0.0f);
  }
}

void testThatStateStaysAtRestAtNonZeroGroundLevelAndStartPose() {
  // Fixture
  config.startPose = (startPose_t){.x = 1.5f, .y = -2.0f, .yaw = (float)(90.0 * DEG)};
  config.groundLevel = 0.75f;
  physicsEngineInit(&config);

  // Test
  steps(lowRatios, 500);

  // Assert
  assertAtRestAt(1.5f, -2.0f, (float)(90.0 * DEG));
}

// Motor lag -------------------------------------------------------------------

void testThatRotorSpeedFollowsFirstOrderLag() {
  // Test, assert: one time constant
  steps(fullRatios, 50);
  const float expected = (float)(MAX_ROTOR_SPEED * (1.0 - exp(-1.0)));
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected, state.rotorSpeeds[i]);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.632f, state.rotorSpeeds[0] / (float)MAX_ROTOR_SPEED);

  // Test, assert: steady state
  steps(fullRatios, 950);
  for (int i = 0; i < NBR_OF_ROTORS; i++) {
    TEST_ASSERT_FLOAT_WITHIN(0.1f, (float)MAX_ROTOR_SPEED, state.rotorSpeeds[i]);
  }
}

// Take-off --------------------------------------------------------------------

void testThatEqualThrustAboveWeightTakesOffStraightUp() {
  // Fixture
  float previousHeight = state.position.z;
  bool hasLeftGround = false;

  // Test
  for (int i = 0; i < 500; i++) {
    step(fullRatios);

    // Assert
    TEST_ASSERT_TRUE(state.position.z >= previousHeight);
    previousHeight = state.position.z;

    // Flying from the step it leaves the ground, at rest until then
    if (state.position.z > config.groundLevel) {
      hasLeftGround = true;
    }
    if (hasLeftGround) {
      TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
    }
    if (physicsEngineGetStatus() == physicsEngineStatusAtRest) {
      assertAtRestAt(0.0f, 0.0f, 0.0f);
    }

    // No horizontal motion and no rotation
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.position.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.position.y);
    assertVec3(0.0f, 0.0f, 0.0f, &state.angularRate);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, state.attitude.w);
  }
  TEST_ASSERT_TRUE(state.position.z > 0.1f);
  TEST_ASSERT_TRUE(state.velocity.z > 0.0f);
}

void testThatTakeOffWorksAtNonZeroGroundLevel() {
  // Fixture
  config.groundLevel = 0.75f;
  physicsEngineInit(&config);

  // Test
  steps(fullRatios, 500);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
  TEST_ASSERT_TRUE(state.position.z > 0.85f);
}

void testThatThrustSlightlyAboveWeightTakesOff() {
  // Fixture
  const uint16_t ratio = ratioOf(hoverRotorSpeed() * 1.01);
  const uint16_t ratios[NBR_OF_ROTORS] = {ratio, ratio, ratio, ratio};

  // Test
  steps(ratios, 2000);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
  TEST_ASSERT_TRUE(state.position.z > config.groundLevel);
}

// Hover -----------------------------------------------------------------------

// Run with each mass configuration, see the top of the file
void testThatHoverRatioGivesZeroAcceleration() {
  // Fixture
  const double rotorSpeed = hoverRotorSpeed();
  const uint16_t ratio = ratioOf(rotorSpeed);
  const uint16_t ratios[NBR_OF_ROTORS] = {ratio, ratio, ratio, ratio};
  const trueState_t hover = airborne(1.0f, rotorSpeed);
  place(&hover);
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());

  // Test
  for (int i = 0; i < 1000; i++) {
    step(ratios);

    // Assert
    TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.0f, state.acceleration.z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.acceleration.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, state.acceleration.y);
    TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
  }
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, state.position.z);
}

// Free fall -------------------------------------------------------------------

void testThatFreeFallFollowsGravity() {
  // Fixture
  const trueState_t start = airborne(10.0f, 0.0);
  place(&start);

  // Test, assert
  for (int i = 1; i <= 1000; i++) {
    step(zeroRatios);

    const double t = i * 0.001;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(10.0 - 0.5 * G * t * t), state.position.z);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(-G * t), state.velocity.z);
  }
}

void testThatThrustIsClampedWhereThePolynomialIsNegative() {
  // Fixture
  // The polynomial has its minimum at 3.9 % of the maximum rotor speed,
  // where it would add 0.08 m/s^2 downwards
  const double rotorSpeed = 0.039 * MAX_ROTOR_SPEED;
  TEST_ASSERT_TRUE(thrustOf(rotorSpeed) < -0.8e-3);
  const uint16_t ratio = ratioOf(rotorSpeed);
  const uint16_t ratios[NBR_OF_ROTORS] = {ratio, ratio, ratio, ratio};
  const trueState_t start = airborne(10.0f, rotorSpeed);
  place(&start);

  // Test
  for (int i = 0; i < 1000; i++) {
    step(ratios);

    // Assert
    TEST_ASSERT_TRUE(state.acceleration.z >= (float)(-G) - 1e-3f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, (float)rotorSpeed, state.rotorSpeeds[0]);
  }
}

// Rotation --------------------------------------------------------------------

// Start in a hover and apply the ratio difference for 20 ms
static void rotate(const int d1, const int d2, const int d3, const int d4) {
  const double rotorSpeed = hoverRotorSpeed();
  const int ratio = ratioOf(rotorSpeed);
  const uint16_t ratios[NBR_OF_ROTORS] = {
    (uint16_t)(ratio + d1), (uint16_t)(ratio + d2), (uint16_t)(ratio + d3), (uint16_t)(ratio + d4)};
  const trueState_t hover = airborne(1.0f, rotorSpeed);
  place(&hover);

  steps(ratios, 20);
}

// The ratio differences are those of powerDistributionForceTorque() in
// power_distribution_quadrotor.c. powerDistributionLegacy() has the opposite
// pitch and yaw signs.
void testThatRatioDifferenceForPositiveRollGivesPositiveRollRateOnly() {
  // Test
  rotate(-5000, -5000, 5000, 5000);

  // Assert
  TEST_ASSERT_TRUE(state.angularRate.x > 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * state.angularRate.x, 0.0f, state.angularRate.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * state.angularRate.x, 0.0f, state.angularRate.z);
}

void testThatRatioDifferenceForPositivePitchGivesPositivePitchRateOnly() {
  // Test
  rotate(-5000, 5000, 5000, -5000);

  // Assert
  TEST_ASSERT_TRUE(state.angularRate.y > 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * state.angularRate.y, 0.0f, state.angularRate.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * state.angularRate.y, 0.0f, state.angularRate.z);
}

void testThatRatioDifferenceForPositiveYawGivesPositiveYawRateOnly() {
  // Test
  rotate(-5000, 5000, -5000, 5000);

  // Assert
  TEST_ASSERT_TRUE(state.angularRate.z > 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * state.angularRate.z, 0.0f, state.angularRate.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * state.angularRate.z, 0.0f, state.angularRate.y);
}

// Run with and without guards, see the top of the file: the same torque then
// gives angular accelerations in the ratio of the two inertia values
void testThatAngularAccelerationIsTorqueOverInertia() {
  // Fixture
  // Rotors held at two speeds, a steady roll torque
  const double low = 0.45 * MAX_ROTOR_SPEED;
  const double high = 0.55 * MAX_ROTOR_SPEED;
  const uint16_t ratios[NBR_OF_ROTORS] = {ratioOf(low), ratioOf(low), ratioOf(high), ratioOf(high)};
  trueState_t start = airborne(1.0f, low);
  start.rotorSpeeds[2] = (float)high;
  start.rotorSpeeds[3] = (float)high;
  place(&start);
  const double torque = HALF_WIDTH * 2.0 * (thrustOf(high) - thrustOf(low));
  const float expected = (float)(torque / INERTIA_X);

  // Test
  steps(ratios, 10);

  // Assert
  TEST_ASSERT_FLOAT_WITHIN(1e-3f * expected, expected, state.angularRate.x / (10 * DT));
}

void testThatQuaternionNormStaysOneOverLongRunWithRotation() {
  // Fixture
  trueState_t start = airborne(1000.0f, 0.0);
  start.angularRate = (trueStateVec3_t){3.0f, -2.0f, 1.0f};
  place(&start);

  // Test
  for (int i = 0; i < 10000; i++) {
    step(zeroRatios);

    // Assert
    const trueStateQuat_t* q = &state.attitude;
    const float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
  }
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
}

// Acceleration ----------------------------------------------------------------

void testThatAccelerationInFlightIsVelocityDifferenceOverStep() {
  // Fixture
  // Tilted and rotating, so the acceleration has parts on all axes
  const uint16_t ratios[NBR_OF_ROTORS] = {40000, 42000, 45000, 43000};
  trueState_t start = airborne(5.0f, hoverRotorSpeed());
  start.attitude = quatOf(10.0 * DEG, -15.0 * DEG, 30.0 * DEG);
  start.velocity = (trueStateVec3_t){0.5f, -0.3f, 0.2f};
  place(&start);

  // Test
  for (int i = 0; i < 200; i++) {
    const trueStateVec3_t previous = state.velocity;
    step(ratios);

    // Assert
    TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (state.velocity.x - previous.x) / DT, state.acceleration.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (state.velocity.y - previous.y) / DT, state.acceleration.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, (state.velocity.z - previous.z) / DT, state.acceleration.z);
  }
  TEST_ASSERT_TRUE(fabsf(state.acceleration.x) > 0.1f);
  TEST_ASSERT_TRUE(fabsf(state.acceleration.y) > 0.1f);
  TEST_ASSERT_TRUE(fabsf(state.acceleration.z) > 0.1f);
}

// Step until the step before the ground is reached, then take that step
static void stepToGround(const physicsEngineStatus_t expectedStatus) {
  for (int i = 0; i < 10000; i++) {
    step(zeroRatios);
    if (physicsEngineGetStatus() != physicsEngineStatusFlying) {
      break;
    }
    // In the fall, up to the step of the stop
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, (float)(-G), state.acceleration.z);
  }
  TEST_ASSERT_EQUAL(expectedStatus, physicsEngineGetStatus());
}

void testThatAccelerationIsZeroInTheStepOfALanding() {
  // Fixture
  const trueState_t start = airborne(0.5f, 0.0);
  place(&start);

  // Test
  stepToGround(physicsEngineStatusAtRest);

  // Assert
  assertVec3(0.0f, 0.0f, 0.0f, &state.acceleration);
}

void testThatAccelerationIsZeroInTheStepOfACrash() {
  // Fixture
  const trueState_t start = airborne(1.0f, 0.0);
  place(&start);

  // Test
  stepToGround(physicsEngineStatusCrashed);

  // Assert
  assertVec3(0.0f, 0.0f, 0.0f, &state.acceleration);
}

// Landing ---------------------------------------------------------------------

void testThatReachingGroundBelowBothLimitsEndsAtRest() {
  // Fixture
  config.groundLevel = 0.75f;
  physicsEngineInit(&config);
  trueState_t start = airborne(0.85f, 0.0);
  start.position.x = 1.0f;
  start.position.y = 2.0f;
  place(&start);
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());

  // Test
  stepsUntilOnGround(zeroRatios);

  // Assert
  assertAtRestAt(1.0f, 2.0f, 0.0f);

  // Test, assert: it stays there
  steps(zeroRatios, 100);
  assertAtRestAt(1.0f, 2.0f, 0.0f);
}

static void landTilted(const double roll, const double pitch) {
  // Fixture
  trueState_t start = airborne(0.1f, 0.0);
  start.position.x = 1.0f;
  start.position.y = 2.0f;
  start.attitude = quatOf(roll, pitch, 30.0 * DEG);
  place(&start);

  // Test
  stepsUntilOnGround(zeroRatios);

  // Assert: pitch and roll are zero; x, y and yaw are kept
  assertAtRestAt(1.0f, 2.0f, (float)(30.0 * DEG));
}

void testThatLandingRolledBelowLimitEndsLevelWithXYAndYawKept() {
  landTilted(20.0 * DEG, 0.0);
  landTilted(-20.0 * DEG, 0.0);
}

void testThatLandingPitchedBelowLimitEndsLevelWithXYAndYawKept() {
  landTilted(0.0, 20.0 * DEG);
  landTilted(0.0, -20.0 * DEG);
}

void testThatLandingWithHorizontalSpeedKeepsThePositionOfTheLanding() {
  // Fixture
  trueState_t start = airborne(0.1f, 0.0);
  start.velocity.x = 1.0f;
  place(&start);

  // Test
  stepsUntilOnGround(zeroRatios);

  // Assert: 0.1 m takes 0.143 s to fall
  assertAtRestAt(state.position.x, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(2e-3f, 0.143f, state.position.x);
}

void testThatItTakesOffAgainAfterALanding() {
  // Fixture
  const trueState_t start = airborne(0.1f, 0.0);
  place(&start);
  stepsUntilOnGround(zeroRatios);
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, physicsEngineGetStatus());

  // Test
  steps(fullRatios, 500);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
  TEST_ASSERT_TRUE(state.position.z > 0.1f);
}

// Status ----------------------------------------------------------------------

void testThatStatusIsAtRestOnGroundWithRotorsSpinningBelowTakeOffThrust() {
  // Test
  steps(lowRatios, 500);

  // Assert
  TEST_ASSERT_TRUE(state.rotorSpeeds[0] > 100.0f);
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, physicsEngineGetStatus());
}

// Crash -----------------------------------------------------------------------

void testThatFreeFallFromOneMetreCrashes() {
  // Fixture: 4.4 m/s at the ground
  const trueState_t start = airborne(1.0f, 0.0);
  place(&start);

  // Test
  stepsUntilOnGround(zeroRatios);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, physicsEngineGetStatus());
}

void testThatFreeFallFromHalfAMetreLands() {
  // Fixture: 3.1 m/s at the ground
  const trueState_t start = airborne(0.5f, 0.0);
  place(&start);

  // Test
  stepsUntilOnGround(zeroRatios);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, physicsEngineGetStatus());
}

void testThatCrashSpeedIsTheMagnitudeOfTheVelocity() {
  // Fixture: 3.1 m/s down as in the landing above, and 3 m/s sideways
  trueState_t start = airborne(0.5f, 0.0);
  start.velocity.y = 3.0f;
  place(&start);

  // Test
  stepsUntilOnGround(zeroRatios);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, physicsEngineGetStatus());
}

// Reach the ground at low speed with the given tilt
static physicsEngineStatus_t statusAfterReachingGroundTilted(const double roll, const double pitch) {
  trueState_t start = airborne(0.01f, 0.0);
  start.attitude = quatOf(roll, pitch, 30.0 * DEG);
  place(&start);

  stepsUntilOnGround(zeroRatios);

  return physicsEngineGetStatus();
}

void testThatReachingGroundRolledAboveLimitCrashesAlsoAtLowSpeed() {
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, statusAfterReachingGroundTilted(46.0 * DEG, 0.0));
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, statusAfterReachingGroundTilted(-46.0 * DEG, 0.0));
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, statusAfterReachingGroundTilted(180.0 * DEG, 0.0));
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, statusAfterReachingGroundTilted(44.0 * DEG, 0.0));
}

void testThatReachingGroundPitchedAboveLimitCrashesAlsoAtLowSpeed() {
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, statusAfterReachingGroundTilted(0.0, 46.0 * DEG));
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, statusAfterReachingGroundTilted(0.0, -46.0 * DEG));
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, statusAfterReachingGroundTilted(0.0, 44.0 * DEG));
}

void testThatStateIsFrozenAfterACrash() {
  // Fixture
  // Falling tilted, moving sideways, rotating, with the rotors spinning
  trueState_t start = airborne(1.0f, 1000.0);
  start.position.x = 1.0f;
  start.position.y = 2.0f;
  start.velocity.x = 0.5f;
  start.attitude = quatOf(50.0 * DEG, 10.0 * DEG, 30.0 * DEG);
  start.angularRate.z = 0.5f;
  place(&start);
  const uint16_t ratios[NBR_OF_ROTORS] = {10000, 10000, 10000, 10000};

  // Test
  trueState_t beforeCrash = state;
  for (int i = 0; i < 10000 && physicsEngineGetStatus() == physicsEngineStatusFlying; i++) {
    beforeCrash = state;
    step(ratios);
  }

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, physicsEngineGetStatus());
  const trueState_t atCrash = state;
  // Position and attitude as they were at the crash, here within one step
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, beforeCrash.position.x, atCrash.position.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, beforeCrash.position.y, atCrash.position.y);
  TEST_ASSERT_EQUAL_FLOAT(config.groundLevel, atCrash.position.z);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, beforeCrash.attitude.w, atCrash.attitude.w);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, beforeCrash.attitude.x, atCrash.attitude.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, beforeCrash.attitude.y, atCrash.attitude.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-2f, beforeCrash.attitude.z, atCrash.attitude.z);
  TEST_ASSERT_TRUE(fabsf(atCrash.attitude.x) > 0.1f);
  assertVec3(0.0f, 0.0f, 0.0f, &atCrash.velocity);
  assertVec3(0.0f, 0.0f, 0.0f, &atCrash.acceleration);
  assertVec3(0.0f, 0.0f, 0.0f, &atCrash.angularRate);
  assertRotorSpeeds(0.0f);

  // Test, assert: full ratios change nothing
  for (int i = 0; i < 1000; i++) {
    step(fullRatios);
    TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, physicsEngineGetStatus());
    TEST_ASSERT_EQUAL_MEMORY(&atCrash, &state, sizeof(trueState_t));
  }
}

// Place -----------------------------------------------------------------------

static trueState_t stateAtRest(void) {
  trueState_t s = airborne(config.groundLevel, 500.0);
  s.position.x = -1.0f;
  s.position.y = 3.0f;
  s.attitude = (trueStateQuat_t){.w = (float)cos(0.5), .x = 0.0f, .y = 0.0f, .z = (float)sin(0.5)};
  return s;
}

static trueState_t stateInTheAir(void) {
  trueState_t s = airborne(2.0f, 1500.0);
  s.position.x = 4.0f;
  s.position.y = -5.0f;
  s.velocity = (trueStateVec3_t){0.1f, 0.2f, 0.3f};
  s.acceleration = (trueStateVec3_t){0.4f, 0.5f, 0.6f};
  s.attitude = quatOf(10.0 * DEG, 20.0 * DEG, 30.0 * DEG);
  s.angularRate = (trueStateVec3_t){0.7f, 0.8f, 0.9f};
  s.rotorSpeeds[1] = 1600.0f;
  s.rotorSpeeds[2] = 1700.0f;
  s.rotorSpeeds[3] = 1800.0f;
  return s;
}

static void crash(void) {
  const trueState_t start = airborne(1.0f, 0.0);
  place(&start);
  stepsUntilOnGround(zeroRatios);
  TEST_ASSERT_EQUAL(physicsEngineStatusCrashed, physicsEngineGetStatus());
}

static void fly(void) {
  const trueState_t start = airborne(1.0f, 0.0);
  place(&start);
  steps(zeroRatios, 10);
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
}

static void assertPlaceGives(const trueState_t* given, const physicsEngineStatus_t expectedStatus) {
  place(given);

  TEST_ASSERT_EQUAL_MEMORY(given, &state, sizeof(trueState_t));
  TEST_ASSERT_EQUAL(expectedStatus, physicsEngineGetStatus());
}

void testThatPlaceAppliesTheGivenStateFromAtRest() {
  // Fixture
  const trueState_t atRest = stateAtRest();
  const trueState_t inTheAir = stateInTheAir();
  TEST_ASSERT_EQUAL(physicsEngineStatusAtRest, physicsEngineGetStatus());

  // Test, assert
  assertPlaceGives(&inTheAir, physicsEngineStatusFlying);
  physicsEngineInit(&config);
  assertPlaceGives(&atRest, physicsEngineStatusAtRest);
}

void testThatPlaceAppliesTheGivenStateFromFlying() {
  // Fixture
  const trueState_t atRest = stateAtRest();
  const trueState_t inTheAir = stateInTheAir();

  // Test, assert
  fly();
  assertPlaceGives(&inTheAir, physicsEngineStatusFlying);
  fly();
  assertPlaceGives(&atRest, physicsEngineStatusAtRest);
}

void testThatPlaceAppliesTheGivenStateFromCrashed() {
  // Fixture
  const trueState_t atRest = stateAtRest();
  const trueState_t inTheAir = stateInTheAir();

  // Test, assert
  crash();
  assertPlaceGives(&inTheAir, physicsEngineStatusFlying);
  crash();
  assertPlaceGives(&atRest, physicsEngineStatusAtRest);
}

void testThatStateOnTheGroundThatIsNotAtRestIsPlacedAsFlying() {
  // Fixture
  trueState_t tilted = stateAtRest();
  tilted.attitude = quatOf(10.0 * DEG, 0.0, 0.0);
  trueState_t moving = stateAtRest();
  moving.velocity.x = 0.1f;
  trueState_t turning = stateAtRest();
  turning.angularRate.z = 0.1f;

  // Test, assert
  assertPlaceGives(&tilted, physicsEngineStatusFlying);
  assertPlaceGives(&moving, physicsEngineStatusFlying);
  assertPlaceGives(&turning, physicsEngineStatusFlying);
}

void testThatPlaceAtRestWhileCrashedEndsTheCrashAndItFliesAgain() {
  // Fixture
  crash();
  const trueState_t atRest = stateAtRest();

  // Test
  place(&atRest);
  steps(fullRatios, 500);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
  TEST_ASSERT_TRUE(state.position.z > 0.1f);
  TEST_ASSERT_TRUE(state.rotorSpeeds[0] > 2000.0f);
}

void testThatPlaceInTheAirWhileCrashedEndsTheCrashAndItFallsAgain() {
  // Fixture
  crash();
  const trueState_t start = airborne(10.0f, 0.0);

  // Test
  place(&start);
  steps(zeroRatios, 100);

  // Assert
  TEST_ASSERT_EQUAL(physicsEngineStatusFlying, physicsEngineGetStatus());
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, (float)(10.0 - 0.5 * G * 0.1 * 0.1), state.position.z);
}
