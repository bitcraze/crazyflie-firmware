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
 * physics_engine.h - the Physics engine interface of the Simmyflie.
 *
 * A physics engine owns the true state and advances it one step at a time
 * from the motor ratios. Exactly one engine is compiled into a build, chosen
 * by the "Physics engine" Kconfig choice, and each engine implements the
 * functions below. The only caller is the Physics engine manager.
 *
 * Every engine keeps the ground and crash contract:
 * - Rest: on the ground, with zero velocity, zero angular rate and zero
 *   pitch and roll.
 * - Floor: height is never below ground level.
 * - Stay at rest: on the ground with thrust below weight the true state
 *   stays at rest, with zero acceleration.
 * - Landing: reaching ground level within both crash limits ends at rest.
 *   Pitch and roll are set to zero at once; x, y and yaw are kept.
 * - Crash: reaching ground level with speed (magnitude of the velocity)
 *   above the crash speed limit, or with pitch or roll beyond the crash
 *   angle limit. Position and attitude are then held; velocity,
 *   acceleration, angular rate and rotor speeds are zero whatever the motor
 *   ratios are, until the next physicsEnginePlace().
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "true_state.h"

typedef struct {
  float x;    // m
  float y;    // m
  float yaw;  // rad
} startPose_t;

typedef struct {
  startPose_t startPose;
  float groundLevel;      // m
  float crashSpeedLimit;  // m/s
  float crashAngleLimit;  // rad
} physicsEngineConfig_t;

typedef enum {
  physicsEngineStatusAtRest,   // On the ground and at rest. Rotors may be spinning.
  physicsEngineStatusFlying,   // Anything else that is not crashed, including a hover.
  physicsEngineStatusCrashed,  // Stopped by a crash.
} physicsEngineStatus_t;

/**
 * @brief Start the engine with the true state at rest at the start pose, at
 * ground level, with the rotors stopped.
 *
 * Called before the scheduler starts, so it must not use DEBUG_PRINT.
 *
 * @return false if the engine can not start
 */
bool physicsEngineInit(const physicsEngineConfig_t *config);

/**
 * @brief Advance the true state by dt.
 *
 * @param motorRatios  Motor ratios as written by the firmware, 0 to 65535, index 0 is M1
 * @param dt           Time to advance (s)
 */
void physicsEngineStep(const uint16_t motorRatios[TRUE_STATE_NBR_OF_ROTORS], float dt);

/**
 * @brief Copy out the true state of the latest step or place.
 */
void physicsEngineGetState(trueState_t *state);

/**
 * @brief The status after the latest step or place.
 */
physicsEngineStatus_t physicsEngineGetStatus(void);

/**
 * @brief Set the true state to the given state, at once and without
 * conditions.
 *
 * The status afterwards is at rest if the given state is at rest, otherwise
 * flying. A place therefore ends a crash.
 */
void physicsEnginePlace(const trueState_t *state);
