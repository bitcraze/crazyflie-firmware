/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
 *
 */
#include <stdbool.h>
#include <string.h>

#include "crtp_commander.h"

#include "commander.h"
#include "crtp.h"
#include "num.h"
#include "FreeRTOS.h"

/* The generic commander format contains a packet type and data that has to be
 * decoded into a setpoint_t structure. The aim is to make it future-proof
 * by easily allowing the addition of new packets for future use cases.
 *
 * The packet format is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * The type is defined bellow together with a decoder function that should take
 * the data buffer in and fill up a setpoint_t structure.
 * The maximum data size is 29 bytes.
 */

/* To add a new packet:
 *   1 - Add a new type in the packetType_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the packetDecoders array.
 *   4 - Pull-request your change :-)
 */

typedef void (*packetDecoder_t)(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen);

/* ---===== 1 - packetType_e enum =====--- */
enum packet_type {
  stopType          = 0,
  velocityWorldType = 1,
  rateType          = 2,
};

/* ---===== 2 - Decoding functions =====--- */
/* The setpoint structure is reinitialized to 0 before being passed to the
 * functions
 */

/* stopDecoder
 * Keeps setpoint to 0: stops the motors and fall
 */
static void stopDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  return;
}

/* velocityDecoder
 * Set the Crazyflie velocity in the world coordinate system
 */
struct velocityPacket_s {
  float vx;        // m in the world frame of reference
  float vy;        // ...
  float vz;        // ...
  float yawrate;  // rad/s
} __attribute__((packed));
static void velocityDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct velocityPacket_s *values = data;

  ASSERT(datalen == sizeof(struct velocityPacket_s));

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;

  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;
  setpoint->velocity.z = values->vz;

  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate;
}

/* rateDecoder
 * Crazyflie setpoint in 'Rate' aka 'Acro' aka 'Non-self-leveling'
 * mode. This is identical to the legacy rpyt packet except values
 * are to be interpreted as degrees per second instead of degrees
 * which is useful for non self-leveled acrobatic flight
 */
#define MIN_THRUST  1000
#define MAX_THRUST  60000
static bool thrustLocked = true;
struct ratePacket_s {
  float rollRate;   // deg/s
  float pitchRate;  // deg/s
  float yawRate;    // deg/s
  uint16_t thrust;  // ratio 0-MAXUINT16
} __attribute__((packed));
static void rateDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct ratePacket_s *values = data;
  
  ASSERT(datalen == sizeof(struct ratePacket_s));

  // Thrust lock for safety -- same as in crtp_commander_rpyt
  if (commanderGetActivePriority() == COMMANDER_PRIORITY_DISABLE) {
    thrustLocked = true;
  }
  if (values->thrust == 0) {
    thrustLocked = false;
  }

  uint16_t rawThrust = values->thrust;

  if (thrustLocked || (rawThrust < MIN_THRUST)) {
    setpoint->thrust = 0;
  } else {
    setpoint->thrust = min(rawThrust, MAX_THRUST);
  }

  setpoint->mode.x = modeDisable;
  setpoint->mode.y = modeDisable;
  setpoint->mode.z = modeDisable;

  setpoint->mode.roll = modeVelocity;
  setpoint->attitudeRate.roll = values->rollRate;
  
  setpoint->mode.pitch = modeVelocity;
  setpoint->attitudeRate.pitch = values->pitchRate;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = values->yawRate;
}

 /* ---===== 3 - packetDecoders array =====--- */
const static packetDecoder_t packetDecoders[] = {
  [stopType]          = stopDecoder,
  [velocityWorldType] = velocityDecoder,
  [rateType]          = rateDecoder,
};

/* Decoder switch */
void crtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, CRTPPacket *pk)
{
  static int nTypes = -1;

  ASSERT(pk->size > 0);

  if (nTypes<0) {
    nTypes = sizeof(packetDecoders)/sizeof(packetDecoders[0]);
  }

  uint8_t type = pk->data[0];

  memset(setpoint, 0, sizeof(setpoint_t));

  if (type<nTypes && (packetDecoders[type] != NULL)) {
    packetDecoders[type](setpoint, type, ((char*)pk->data)+1, pk->size-1);
  }
}
