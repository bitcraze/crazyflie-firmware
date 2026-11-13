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
 * true_state.h - the true state of the Simmyflie: what the simulated
 * Crazyflie is actually doing, as opposed to what its firmware estimates.
 *
 * Shared by the Physics engine, the Physics engine manager and the Sensor
 * shim. Includes no firmware header, so a physics engine depends on nothing
 * from the firmware for its data.
 */

#pragma once

#define TRUE_STATE_NBR_OF_ROTORS 4

typedef struct { float x, y, z; } trueStateVec3_t;

// Stored as w, x, y, z. The firmware's quaternion_t is x, y, z, w: convert
// field by field.
typedef struct { float w, x, y, z; } trueStateQuat_t;

typedef struct {
  trueStateVec3_t position;      // m, world
  trueStateVec3_t velocity;      // m/s, world
  trueStateVec3_t acceleration;  // m/s^2, world
  trueStateQuat_t attitude;      // unit quaternion, body to world
  trueStateVec3_t angularRate;   // rad/s, body
  float rotorSpeeds[TRUE_STATE_NBR_OF_ROTORS];  // rad/s, magnitude, index 0 is M1
} trueState_t;
