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
 * physics_engine_builtin.c - the built-in physics engine of the Simmyflie.
 *
 * The physics model is the brushless Crazyflie model of A. Graefe,
 * C. Scherer, W. Hoenig and S. Trimpe, "How to Model Your Crazyflie
 * Brushless" (2026). Equation and table numbers below are the paper's.
 *
 * One fourth-order Runge-Kutta step is taken per physicsEngineStep() call.
 * The ground and crash contract of physics_engine.h is applied to the result
 * of that step.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "autoconf.h"
#include "physics_engine.h"

// The host unit test build has no sim configuration, the Kconfig default is
// used there unless the value is given on its command line.
#if defined(UNIT_TEST_MODE) && !defined(CONFIG_SIM_BUILTIN_MASS_MG)
#define CONFIG_SIM_BUILTIN_MASS_MG 40000
#endif

// Mass and inertia (Table II), with or without propeller guards
#ifdef CONFIG_SIM_BUILTIN_PROPELLER_GUARDS
#define PROPELLER_GUARDS_MASS_MG 4000
static const float inertia[3] = {3.3e-5f, 3.6e-5f, 5.9e-5f};  // kg m^2, diagonal
#else
#define PROPELLER_GUARDS_MASS_MG 0
static const float inertia[3] = {1.8e-5f, 2.4e-5f, 3.0e-5f};  // kg m^2, diagonal
#endif
static const float mass = (CONFIG_SIM_BUILTIN_MASS_MG + PROPELLER_GUARDS_MASS_MG) * 1e-6f;  // kg

// Table II
#define MOTOR_TIME_CONSTANT 0.05f      // T (s)
#define MOTOR_AMPLIFICATION 2900.0f    // K (rad/s)
#define MOTOR_INERTIA 0.5e-7f          // Jm (kg m^2)
#define HALF_WIDTH (0.5f * 0.0707f)    // l / 2 (m)

#define ROTOR_SPEED_SCALE 2900.0f      // sigma of equations 9 and 10 (rad/s)
#define GRAVITY 9.81f                  // m/s^2
#define MOTOR_RATIO_MAX 65535.0f

// The integrated state, the paper's 17 values
enum {
  IX_POSITION = 0,       // 3, world (m)
  IX_VELOCITY = 3,       // 3, world (m/s)
  IX_ATTITUDE = 6,       // 4, w x y z, body to world
  IX_ANGULAR_RATE = 10,  // 3, body (rad/s)
  IX_ROTOR_SPEED = 13,   // 4, (rad/s)
  STATE_SIZE = 17,
};

static physicsEngineConfig_t config;
static float state[STATE_SIZE];
static float acceleration[3];  // world (m/s^2), not integrated
static physicsEngineStatus_t status;

// Equation 9 (N)
static float rotorThrust(const float rotorSpeed)
{
  const float s = rotorSpeed / ROTOR_SPEED_SCALE;
  const float thrust = ((-0.23f * s + 0.562f) * s - 0.043f) * s;

  // Deviation from the paper: the fitted polynomial is slightly negative
  // below about 8 % of the maximum rotor speed. Clamped, so a Simmyflie
  // falling with its rotors spinning down does not accelerate faster than g.
  return (thrust > 0.0f) ? thrust : 0.0f;
}

// Equation 10 (N m). Not clamped: it is positive for all rotor speeds a
// motor ratio can give (roots at s = -0.30, 0 and 2.86).
static float rotorFrictionTorque(const float rotorSpeed)
{
  const float s = rotorSpeed / ROTOR_SPEED_SCALE;
  return 1e-4f * ((-3.4f * s + 8.7f) * s + 2.9f) * s;
}

// Equations 1 to 8. The paper's motors 1 to 4 are the firmware's M1 to M4:
// the signs of equation 7 are those of powerDistributionForceTorque() for
// every motor on all three axes.
static void derivative(const float s[STATE_SIZE], const float u[TRUE_STATE_NBR_OF_ROTORS], float d[STATE_SIZE])
{
  const float *v = &s[IX_VELOCITY];
  const float qw = s[IX_ATTITUDE], qx = s[IX_ATTITUDE + 1], qy = s[IX_ATTITUDE + 2], qz = s[IX_ATTITUDE + 3];
  const float wx = s[IX_ANGULAR_RATE], wy = s[IX_ANGULAR_RATE + 1], wz = s[IX_ANGULAR_RATE + 2];

  float F[TRUE_STATE_NBR_OF_ROTORS];
  float M[TRUE_STATE_NBR_OF_ROTORS];
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    const float rotorSpeed = s[IX_ROTOR_SPEED + i];
    const float rotorAcceleration = (MOTOR_AMPLIFICATION * u[i] - rotorSpeed) / MOTOR_TIME_CONSTANT;  // (5)

    d[IX_ROTOR_SPEED + i] = rotorAcceleration;
    F[i] = rotorThrust(rotorSpeed);
    M[i] = MOTOR_INERTIA * rotorAcceleration + rotorFrictionTorque(rotorSpeed);
  }

  // (7), with (8) written into the first two rows
  const float torqueX = HALF_WIDTH * (-F[0] - F[1] + F[2] + F[3]);
  const float torqueY = HALF_WIDTH * (-F[0] + F[1] + F[2] - F[3]);
  const float torqueZ = -M[0] + M[1] - M[2] + M[3];

  // (1)
  d[IX_POSITION] = v[0];
  d[IX_POSITION + 1] = v[1];
  d[IX_POSITION + 2] = v[2];

  // (2), with (6): the total thrust along the body z axis, in the world frame
  const float thrustAcceleration = (F[0] + F[1] + F[2] + F[3]) / mass;
  d[IX_VELOCITY] = thrustAcceleration * 2.0f * (qx * qz + qw * qy);
  d[IX_VELOCITY + 1] = thrustAcceleration * 2.0f * (qy * qz - qw * qx);
  d[IX_VELOCITY + 2] = thrustAcceleration * (1.0f - 2.0f * (qx * qx + qy * qy)) - GRAVITY;

  // (3)
  d[IX_ATTITUDE] = 0.5f * (-qx * wx - qy * wy - qz * wz);
  d[IX_ATTITUDE + 1] = 0.5f * (qw * wx + qy * wz - qz * wy);
  d[IX_ATTITUDE + 2] = 0.5f * (qw * wy - qx * wz + qz * wx);
  d[IX_ATTITUDE + 3] = 0.5f * (qw * wz + qx * wy - qy * wx);

  // (4), with a diagonal inertia matrix
  d[IX_ANGULAR_RATE] = (torqueX - (inertia[2] - inertia[1]) * wy * wz) / inertia[0];
  d[IX_ANGULAR_RATE + 1] = (torqueY - (inertia[0] - inertia[2]) * wz * wx) / inertia[1];
  d[IX_ANGULAR_RATE + 2] = (torqueZ - (inertia[1] - inertia[0]) * wx * wy) / inertia[2];
}

static void rk4Step(const float s[STATE_SIZE], const float u[TRUE_STATE_NBR_OF_ROTORS], const float dt, float next[STATE_SIZE])
{
  float k1[STATE_SIZE], k2[STATE_SIZE], k3[STATE_SIZE], k4[STATE_SIZE];
  float tmp[STATE_SIZE];

  derivative(s, u, k1);
  for (int i = 0; i < STATE_SIZE; i++) {
    tmp[i] = s[i] + 0.5f * dt * k1[i];
  }
  derivative(tmp, u, k2);
  for (int i = 0; i < STATE_SIZE; i++) {
    tmp[i] = s[i] + 0.5f * dt * k2[i];
  }
  derivative(tmp, u, k3);
  for (int i = 0; i < STATE_SIZE; i++) {
    tmp[i] = s[i] + dt * k3[i];
  }
  derivative(tmp, u, k4);
  for (int i = 0; i < STATE_SIZE; i++) {
    next[i] = s[i] + dt / 6.0f * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
  }

  float *q = &next[IX_ATTITUDE];
  const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) {
    q[i] /= norm;
  }
}

static void setAttitudeFromYaw(float s[STATE_SIZE], const float yaw)
{
  s[IX_ATTITUDE] = cosf(0.5f * yaw);
  s[IX_ATTITUDE + 1] = 0.0f;
  s[IX_ATTITUDE + 2] = 0.0f;
  s[IX_ATTITUDE + 3] = sinf(0.5f * yaw);
}

static float roll(const float *q)
{
  return atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
}

static float pitch(const float *q)
{
  float sinPitch = 2.0f * (q[0] * q[2] - q[1] * q[3]);
  if (sinPitch > 1.0f) {
    sinPitch = 1.0f;
  } else if (sinPitch < -1.0f) {
    sinPitch = -1.0f;
  }
  return asinf(sinPitch);
}

static float yaw(const float *q)
{
  return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

// Rest as defined by the contract. Exact comparisons: a state is only at
// rest if it was made so, by this engine or by the caller of a place.
static bool isAtRest(const float s[STATE_SIZE])
{
  return s[IX_POSITION + 2] <= config.groundLevel &&
         s[IX_VELOCITY] == 0.0f && s[IX_VELOCITY + 1] == 0.0f && s[IX_VELOCITY + 2] == 0.0f &&
         s[IX_ANGULAR_RATE] == 0.0f && s[IX_ANGULAR_RATE + 1] == 0.0f && s[IX_ANGULAR_RATE + 2] == 0.0f &&
         s[IX_ATTITUDE + 1] == 0.0f && s[IX_ATTITUDE + 2] == 0.0f;
}

static void zero(float *values, const int count)
{
  for (int i = 0; i < count; i++) {
    values[i] = 0.0f;
  }
}

// Accept the result of an integration step as the new state, in flight
static void fly(const float next[STATE_SIZE], const float dt)
{
  for (int i = 0; i < 3; i++) {
    acceleration[i] = (next[IX_VELOCITY + i] - state[IX_VELOCITY + i]) / dt;
  }
  for (int i = 0; i < STATE_SIZE; i++) {
    state[i] = next[i];
  }
  if (state[IX_POSITION + 2] < config.groundLevel) {
    state[IX_POSITION + 2] = config.groundLevel;
  }
  status = physicsEngineStatusFlying;
}

// The Simmyflie has reached ground level: a landing or a crash
static void touchGround(const float next[STATE_SIZE])
{
  const float *v = &next[IX_VELOCITY];
  const float *q = &next[IX_ATTITUDE];
  const float speed = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  const bool isCrash = speed > config.crashSpeedLimit ||
                       fabsf(roll(q)) > config.crashAngleLimit ||
                       fabsf(pitch(q)) > config.crashAngleLimit;

  state[IX_POSITION] = next[IX_POSITION];
  state[IX_POSITION + 1] = next[IX_POSITION + 1];
  state[IX_POSITION + 2] = config.groundLevel;
  zero(&state[IX_VELOCITY], 3);
  zero(&state[IX_ANGULAR_RATE], 3);
  // The stop is not reported as an acceleration
  zero(acceleration, 3);

  if (isCrash) {
    for (int i = 0; i < 4; i++) {
      state[IX_ATTITUDE + i] = q[i];
    }
    zero(&state[IX_ROTOR_SPEED], TRUE_STATE_NBR_OF_ROTORS);
    status = physicsEngineStatusCrashed;
  } else {
    setAttitudeFromYaw(state, yaw(q));
    for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
      state[IX_ROTOR_SPEED + i] = next[IX_ROTOR_SPEED + i];
    }
    status = physicsEngineStatusAtRest;
  }
}

bool physicsEngineInit(const physicsEngineConfig_t *engineConfig)
{
  config = *engineConfig;

  zero(state, STATE_SIZE);
  zero(acceleration, 3);
  state[IX_POSITION] = config.startPose.x;
  state[IX_POSITION + 1] = config.startPose.y;
  state[IX_POSITION + 2] = config.groundLevel;
  setAttitudeFromYaw(state, config.startPose.yaw);
  status = physicsEngineStatusAtRest;

  return true;
}

void physicsEngineStep(const uint16_t motorRatios[TRUE_STATE_NBR_OF_ROTORS], const float dt)
{
  if (status == physicsEngineStatusCrashed) {
    return;
  }

  float u[TRUE_STATE_NBR_OF_ROTORS];
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    u[i] = motorRatios[i] / MOTOR_RATIO_MAX;
  }

  float next[STATE_SIZE];
  rk4Step(state, u, dt, next);

  const bool isMovingUp = next[IX_VELOCITY + 2] > 0.0f;

  if (status == physicsEngineStatusAtRest) {
    if (isMovingUp) {
      // Thrust above weight: take-off
      fly(next, dt);
    } else {
      // Stay at rest: only the rotors move
      for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
        state[IX_ROTOR_SPEED + i] = next[IX_ROTOR_SPEED + i];
      }
      zero(acceleration, 3);
    }
  } else {
    if (next[IX_POSITION + 2] <= config.groundLevel && !isMovingUp) {
      touchGround(next);
    } else {
      fly(next, dt);
    }
  }
}

void physicsEngineGetState(trueState_t *trueState)
{
  trueState->position = (trueStateVec3_t){state[IX_POSITION], state[IX_POSITION + 1], state[IX_POSITION + 2]};
  trueState->velocity = (trueStateVec3_t){state[IX_VELOCITY], state[IX_VELOCITY + 1], state[IX_VELOCITY + 2]};
  trueState->acceleration = (trueStateVec3_t){acceleration[0], acceleration[1], acceleration[2]};
  trueState->attitude = (trueStateQuat_t){state[IX_ATTITUDE], state[IX_ATTITUDE + 1], state[IX_ATTITUDE + 2], state[IX_ATTITUDE + 3]};
  trueState->angularRate = (trueStateVec3_t){state[IX_ANGULAR_RATE], state[IX_ANGULAR_RATE + 1], state[IX_ANGULAR_RATE + 2]};
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    trueState->rotorSpeeds[i] = state[IX_ROTOR_SPEED + i];
  }
}

physicsEngineStatus_t physicsEngineGetStatus(void)
{
  return status;
}

void physicsEnginePlace(const trueState_t *trueState)
{
  state[IX_POSITION] = trueState->position.x;
  state[IX_POSITION + 1] = trueState->position.y;
  state[IX_POSITION + 2] = trueState->position.z;
  state[IX_VELOCITY] = trueState->velocity.x;
  state[IX_VELOCITY + 1] = trueState->velocity.y;
  state[IX_VELOCITY + 2] = trueState->velocity.z;
  acceleration[0] = trueState->acceleration.x;
  acceleration[1] = trueState->acceleration.y;
  acceleration[2] = trueState->acceleration.z;
  state[IX_ATTITUDE] = trueState->attitude.w;
  state[IX_ATTITUDE + 1] = trueState->attitude.x;
  state[IX_ATTITUDE + 2] = trueState->attitude.y;
  state[IX_ATTITUDE + 3] = trueState->attitude.z;
  state[IX_ANGULAR_RATE] = trueState->angularRate.x;
  state[IX_ANGULAR_RATE + 1] = trueState->angularRate.y;
  state[IX_ANGULAR_RATE + 2] = trueState->angularRate.z;
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    state[IX_ROTOR_SPEED + i] = trueState->rotorSpeeds[i];
  }

  status = isAtRest(state) ? physicsEngineStatusAtRest : physicsEngineStatusFlying;
}
