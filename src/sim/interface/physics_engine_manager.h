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
 * physics_engine_manager.h - the Physics engine manager of the Simmyflie.
 *
 * The manager is the only caller of the Physics engine, and is the same
 * whichever engine is built in. It holds the latest motor ratios until the
 * next step, the start pose parameters (param group "sim"), and the snapshot
 * of the true state that is logged (log group "trueState").
 *
 * physicsEngineManagerStep() and physicsEngineManagerGetTrueState() are
 * called from one task, the one that steps (the stabilizer task), and
 * physicsEngineManagerInit() before the scheduler starts.
 * physicsEngineManagerSetMotorRatio() and physicsEngineManagerGetSnapshot()
 * may be called from any task.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "physics_engine.h"
#include "true_state.h"

// The true state, status and sim time of one step
typedef struct {
  trueState_t state;
  physicsEngineStatus_t status;
  uint32_t simTime;  // firmware tick count at the step (ms)
} physicsEngineManagerSnapshot_t;

/**
 * @brief Set the start pose parameters from the config, start the engine and
 * read its state and status, so both are valid before the first step.
 *
 * Called before the scheduler starts, so it must not use DEBUG_PRINT.
 *
 * @return what physicsEngineInit() returned
 */
bool physicsEngineManagerInit(const physicsEngineConfig_t *config);

/**
 * @brief Store the ratio of one motor, used by every following step until it
 * is replaced. Never calls the engine.
 *
 * @param id     Motor index, 0 is M1. Other ids are ignored.
 * @param ratio  Motor ratio as written by the firmware, 0 to 65535
 */
void physicsEngineManagerSetMotorRatio(uint32_t id, uint16_t ratio);

/**
 * @brief Advance the true state by dt: apply a written start pose if the
 * status allows it, step the engine with the stored ratios, read the state
 * and status, and store them in the snapshot with the tick count.
 *
 * @param dt  Time to advance (s)
 */
void physicsEngineManagerStep(float dt);

/**
 * @brief Copy out the snapshot of the true state. Does not call the engine.
 * For the task that steps only: there is no lock.
 */
void physicsEngineManagerGetTrueState(trueState_t *state);

/**
 * @brief Copy out the true state, status and sim time of the latest step, or
 * of init before the first step, all from the same step. Does not call the
 * engine. May be called from any task.
 */
void physicsEngineManagerGetSnapshot(physicsEngineManagerSnapshot_t *snapshot);
