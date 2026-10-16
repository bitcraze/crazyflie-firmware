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
 * beacon_sender.h - the Beacon sender of the Simmyflie.
 *
 * Sends a beacon, a fixed-size UDP datagram with the true pose, rotor speeds
 * and status, on every BEACON_INTERVAL_TICKS tick. It has no task of its own
 * and knows nothing of where the state comes from: the Physics engine manager
 * calls beaconSenderTick() at the end of every step, with the state and
 * status of that step.
 *
 * Wire format, version 1: BEACON_SIZE bytes, little-endian, no padding, in
 * Python struct format "<BBHI11f":
 *
 *   u8 version, u8 status (0 at rest, 1 flying, 2 crashed), u16 udpPort,
 *   u32 simTime (ms), f32 x, y, z (m, world), f32 qw, qx, qy, qz (body to
 *   world), f32 rotorSpeed1..4 (rad/s, M1 first)
 */

#pragma once

#include <stdint.h>

#include <netinet/in.h>

#include "physics_engine.h"
#include "true_state.h"

#define BEACON_VERSION 1
#define BEACON_SIZE 52
#define BEACON_INTERVAL_TICKS 20  // 1 ms tick, so 50 Hz

#define BEACON_STATUS_AT_REST 0
#define BEACON_STATUS_FLYING 1
#define BEACON_STATUS_CRASHED 2

/**
 * @brief Start sending to dest on the given socket, which is set up by the
 * caller (multicast options, non-blocking). Called before the scheduler
 * starts.
 *
 * @param fd       Socket to send on
 * @param dest     Beacon address
 * @param udpPort  UDP port of the instance, put in every beacon
 */
void beaconSenderInit(int fd, const struct sockaddr_in *dest, uint16_t udpPort);

/**
 * @brief Count one tick, and on every BEACON_INTERVAL_TICKS tick send a
 * beacon of the given state and status. Never blocks: a beacon that cannot be
 * sent is dropped. Does nothing before init.
 *
 * @param state    True state after this step
 * @param status   Status after this step
 * @param simTime  Firmware tick count at this step (ms)
 */
void beaconSenderTick(const trueState_t *state, physicsEngineStatus_t status, uint32_t simTime);
