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
 * beacon_sender.c - see beacon_sender.h.
 *
 * Only called from the step of the Physics engine manager, so the state,
 * status and sim time in a beacon are from the same step and no lock is
 * needed. The send is non-blocking, since a blocking call stalls the FreeRTOS
 * POSIX port, and a failed send is dropped silently: nobody needs to hear a
 * beacon.
 */

#include "beacon_sender.h"

#include <stdbool.h>
#include <string.h>

#include <sys/socket.h>

#include "test_support.h"

static bool isInit = false;
static int beaconFd = -1;
static struct sockaddr_in beaconDest;
static uint16_t beaconUdpPort;
static uint32_t tickCount;

// The wire values, defined here so the format does not follow the enum order
static uint8_t wireStatusOf(const physicsEngineStatus_t status)
{
  switch (status) {
    case physicsEngineStatusAtRest:
      return BEACON_STATUS_AT_REST;
    case physicsEngineStatusCrashed:
      return BEACON_STATUS_CRASHED;
    case physicsEngineStatusFlying:
    default:
      return BEACON_STATUS_FLYING;
  }
}

static uint8_t *putU16(uint8_t *p, const uint16_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  return p + 2;
}

static uint8_t *putU32(uint8_t *p, const uint32_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
  p[2] = (uint8_t)(value >> 16);
  p[3] = (uint8_t)(value >> 24);
  return p + 4;
}

static uint8_t *putF32(uint8_t *p, const float value)
{
  uint32_t bits;
  memcpy(&bits, &value, sizeof(bits));
  return putU32(p, bits);
}

TESTABLE_STATIC void beaconPack(uint8_t buffer[BEACON_SIZE], const trueState_t *state,
                                const physicsEngineStatus_t status, const uint16_t udpPort,
                                const uint32_t simTime)
{
  uint8_t *p = buffer;
  *p++ = BEACON_VERSION;
  *p++ = wireStatusOf(status);
  p = putU16(p, udpPort);
  p = putU32(p, simTime);
  p = putF32(p, state->position.x);
  p = putF32(p, state->position.y);
  p = putF32(p, state->position.z);
  p = putF32(p, state->attitude.w);
  p = putF32(p, state->attitude.x);
  p = putF32(p, state->attitude.y);
  p = putF32(p, state->attitude.z);
  for (int i = 0; i < TRUE_STATE_NBR_OF_ROTORS; i++) {
    p = putF32(p, state->rotorSpeeds[i]);
  }
}

void beaconSenderInit(int fd, const struct sockaddr_in *dest, uint16_t udpPort)
{
  beaconFd = fd;
  beaconDest = *dest;
  beaconUdpPort = udpPort;
  tickCount = 0;
  isInit = true;
}

void beaconSenderTick(const trueState_t *state, physicsEngineStatus_t status, uint32_t simTime)
{
  if (!isInit) {
    return;
  }

  tickCount++;
  if (tickCount < BEACON_INTERVAL_TICKS) {
    return;
  }
  tickCount = 0;

  uint8_t buffer[BEACON_SIZE];
  beaconPack(buffer, state, status, beaconUdpPort, simTime);

  // Dropped on any failure, e.g. EAGAIN; the next beacon is sent as usual
  (void)sendto(beaconFd, buffer, sizeof(buffer), MSG_DONTWAIT,
               (const struct sockaddr *)&beaconDest, sizeof(beaconDest));
}
