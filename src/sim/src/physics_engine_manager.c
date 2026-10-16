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
 * physics_engine_manager.c - the Physics engine manager of the Simmyflie.
 *
 * A start pose parameter is written on the param task, and the engine is
 * only entered from the task that steps. The parameter callbacks therefore
 * only mark the coordinate as written, and the step applies it. param_logic.c
 * stores the value before it calls the callback, so a set mark means the
 * value is in place. A write that lands after the step has taken the marks
 * sets its mark again and is applied on the next step.
 *
 * Every step ends by handing the new state and status to the Beacon sender,
 * with the firmware tick count as sim time.
 */

#define DEBUG_MODULE "SIM"

#include <math.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "beacon_sender.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "physics_engine.h"
#include "physics_engine_manager.h"
#include "test_support.h"

#define RAD_PER_DEG (3.14159265358979f / 180.0f)

// Written marks of the start pose parameters
#define MARK_START_X   (1u << 0)
#define MARK_START_Y   (1u << 1)
#define MARK_START_YAW (1u << 2)

static float groundLevel;  // m
static uint16_t motorRatios[TRUE_STATE_NBR_OF_ROTORS];
static trueState_t snapshot;
static uint8_t status;     // physicsEngineStatus_t, logged
static uint32_t marks;

// Start pose parameters, the last written start pose
TESTABLE_STATIC float startX;    // m
TESTABLE_STATIC float startY;    // m
TESTABLE_STATIC float startYaw;  // degrees

TESTABLE_STATIC void startXWritten(void)
{
  __atomic_fetch_or(&marks, MARK_START_X, __ATOMIC_SEQ_CST);
}

TESTABLE_STATIC void startYWritten(void)
{
  __atomic_fetch_or(&marks, MARK_START_Y, __ATOMIC_SEQ_CST);
}

TESTABLE_STATIC void startYawWritten(void)
{
  __atomic_fetch_or(&marks, MARK_START_YAW, __ATOMIC_SEQ_CST);
}

// Yaw (rad) of a body to world quaternion, in the yaw, pitch, roll order
static float yawOf(const trueStateQuat_t *q)
{
  return atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}

// At rest at the pose to move to, at ground level, rotor speeds unchanged
static void moveToStartPose(const uint32_t written)
{
  const float x = (written & MARK_START_X) ? startX : snapshot.position.x;
  const float y = (written & MARK_START_Y) ? startY : snapshot.position.y;
  const float yaw = (written & MARK_START_YAW) ? startYaw * RAD_PER_DEG : yawOf(&snapshot.attitude);

  trueState_t placed = {
    .position = {.x = x, .y = y, .z = groundLevel},
    .attitude = {.w = cosf(0.5f * yaw), .x = 0.0f, .y = 0.0f, .z = sinf(0.5f * yaw)},
  };
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    placed.rotorSpeeds[i] = snapshot.rotorSpeeds[i];
  }

  physicsEnginePlace(&placed);
  DEBUG_PRINT("Moved to start pose x %.3f m, y %.3f m, yaw %.1f deg\n",
              (double)x, (double)y, (double)(yaw / RAD_PER_DEG));
}

bool physicsEngineManagerInit(const physicsEngineConfig_t *config)
{
  groundLevel = config->groundLevel;
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    motorRatios[i] = 0;
  }
  marks = 0;
  startX = config->startPose.x;
  startY = config->startPose.y;
  startYaw = config->startPose.yaw / RAD_PER_DEG;

  const bool result = physicsEngineInit(config);
  physicsEngineGetState(&snapshot);
  status = (uint8_t)physicsEngineGetStatus();
  return result;
}

void physicsEngineManagerSetMotorRatio(uint32_t id, uint16_t ratio)
{
  if (id < TRUE_STATE_NBR_OF_ROTORS) {
    motorRatios[id] = ratio;
  }
}

void physicsEngineManagerStep(float dt)
{
  const uint32_t written = __atomic_exchange_n(&marks, 0, __ATOMIC_SEQ_CST);
  if (written != 0) {
    if (status == physicsEngineStatusFlying) {
      DEBUG_PRINT("Start pose move refused, the Simmyflie is flying\n");
    } else {
      moveToStartPose(written);
    }
  }

  physicsEngineStep(motorRatios, dt);
  physicsEngineGetState(&snapshot);
  status = (uint8_t)physicsEngineGetStatus();

  beaconSenderTick(&snapshot, (physicsEngineStatus_t)status, (uint32_t)xTaskGetTickCount());
}

void physicsEngineManagerGetTrueState(trueState_t *state)
{
  *state = snapshot;
}

/**
 * The start pose of the Simmyflie. Writing a parameter moves the Simmyflie
 * there, at rest at ground level, if it is at rest or crashed. A move is
 * refused while it is flying. The parameters hold the last written start
 * pose, not the current pose: see trueState for that.
 */
PARAM_GROUP_START(sim)
/**
 * @brief Start position x (m)
 */
PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, startX, &startX, &startXWritten)
/**
 * @brief Start position y (m)
 */
PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, startY, &startY, &startYWritten)
/**
 * @brief Start yaw (degrees)
 */
PARAM_ADD_WITH_CALLBACK(PARAM_FLOAT, startYaw, &startYaw, &startYawWritten)
PARAM_GROUP_STOP(sim)

/**
 * The true state of the Simmyflie: what it is actually doing, as opposed to
 * what its firmware estimates (stateEstimate). Values are from the latest
 * step, in SI units.
 */
LOG_GROUP_START(trueState)
/**
 * @brief Position x, world (m)
 */
LOG_ADD(LOG_FLOAT, x, &snapshot.position.x)
/**
 * @brief Position y, world (m)
 */
LOG_ADD(LOG_FLOAT, y, &snapshot.position.y)
/**
 * @brief Position z, world (m)
 */
LOG_ADD(LOG_FLOAT, z, &snapshot.position.z)
/**
 * @brief Velocity x, world (m/s)
 */
LOG_ADD(LOG_FLOAT, vx, &snapshot.velocity.x)
/**
 * @brief Velocity y, world (m/s)
 */
LOG_ADD(LOG_FLOAT, vy, &snapshot.velocity.y)
/**
 * @brief Velocity z, world (m/s)
 */
LOG_ADD(LOG_FLOAT, vz, &snapshot.velocity.z)
/**
 * @brief Acceleration x, world (m/s^2). Note: stateEstimate.ax is in g.
 */
LOG_ADD(LOG_FLOAT, ax, &snapshot.acceleration.x)
/**
 * @brief Acceleration y, world (m/s^2). Note: stateEstimate.ay is in g.
 */
LOG_ADD(LOG_FLOAT, ay, &snapshot.acceleration.y)
/**
 * @brief Acceleration z, world (m/s^2). Note: stateEstimate.az is in g.
 */
LOG_ADD(LOG_FLOAT, az, &snapshot.acceleration.z)
/**
 * @brief Attitude quaternion w, body to world
 */
LOG_ADD(LOG_FLOAT, qw, &snapshot.attitude.w)
/**
 * @brief Attitude quaternion x, body to world
 */
LOG_ADD(LOG_FLOAT, qx, &snapshot.attitude.x)
/**
 * @brief Attitude quaternion y, body to world
 */
LOG_ADD(LOG_FLOAT, qy, &snapshot.attitude.y)
/**
 * @brief Attitude quaternion z, body to world
 */
LOG_ADD(LOG_FLOAT, qz, &snapshot.attitude.z)
/**
 * @brief Angular rate around body x (rad/s)
 */
LOG_ADD(LOG_FLOAT, rateX, &snapshot.angularRate.x)
/**
 * @brief Angular rate around body y (rad/s)
 */
LOG_ADD(LOG_FLOAT, rateY, &snapshot.angularRate.y)
/**
 * @brief Angular rate around body z (rad/s)
 */
LOG_ADD(LOG_FLOAT, rateZ, &snapshot.angularRate.z)
/**
 * @brief Rotor speed of M1 (rad/s)
 */
LOG_ADD(LOG_FLOAT, rotor1, &snapshot.rotorSpeeds[0])
/**
 * @brief Rotor speed of M2 (rad/s)
 */
LOG_ADD(LOG_FLOAT, rotor2, &snapshot.rotorSpeeds[1])
/**
 * @brief Rotor speed of M3 (rad/s)
 */
LOG_ADD(LOG_FLOAT, rotor3, &snapshot.rotorSpeeds[2])
/**
 * @brief Rotor speed of M4 (rad/s)
 */
LOG_ADD(LOG_FLOAT, rotor4, &snapshot.rotorSpeeds[3])
/**
 * @brief Status of the physics engine: 0 at rest, 1 flying, 2 crashed
 */
LOG_ADD(LOG_UINT8, status, &status)
LOG_GROUP_STOP(trueState)
