/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "commander.h"
#include "crtp.h"
#include "configblock.h"
#include "param.h"
#include "num.h"

#define MIN_THRUST  1000
#define MAX_THRUST  60000
#define COMMANDER_CACH_TIMEOUT  M2T(500)

/**
 * Commander control data
 */
typedef struct _CommanderCach
{
  struct CommanderCrtpValues targetVal[2];
  bool activeSide;
  uint32_t timestamp;
} CommanderCach;

/**
 * Stabilization modes for Roll, Pitch, Yaw.
 */
typedef enum
{
  RATE    = 0,
  ANGLE   = 1,
} RPYType;

/**
 * Yaw flight Modes
 */
typedef enum
{
  CAREFREE  = 0, // Yaw is locked to world coordinates thus heading stays the same when yaw rotates
  PLUSMODE  = 1, // Plus-mode. Motor M1 is defined as front
  XMODE     = 2, // X-mode. M1 & M4 is defined as front
} YawModeType;

static bool isInit;
static CommanderCach crtpCach;
static CommanderCach extrxCach;
static CommanderCach* activeCach;

static uint32_t lastUpdate;
static bool isInactive;
static bool thrustLocked;
static bool altHoldMode = false;
static bool posHoldMode = false;

static RPYType stabilizationModeRoll  = ANGLE; // Current stabilization type of roll (rate or angle)
static RPYType stabilizationModePitch = ANGLE; // Current stabilization type of pitch (rate or angle)
static RPYType stabilizationModeYaw   = RATE;  // Current stabilization type of yaw (rate or angle)

static YawModeType yawMode = DEFUALT_YAW_MODE; // Yaw mode configuration
static bool carefreeResetFront;             // Reset what is front in carefree mode

static void commanderCrtpCB(CRTPPacket* pk);
static void commanderCachSelectorUpdate(void);

/* Private functions */
static void commanderSetActiveThrust(uint16_t thrust)
{
  activeCach->targetVal[activeCach->activeSide].thrust = thrust;
}

static void commanderSetActiveRoll(float roll)
{
  activeCach->targetVal[activeCach->activeSide].roll = roll;
}

static void commanderSetActivePitch(float pitch)
{
  activeCach->targetVal[activeCach->activeSide].pitch = pitch;
}

static void commanderSetActiveYaw(float yaw)
{
  activeCach->targetVal[activeCach->activeSide].yaw = yaw;
}

static uint16_t commanderGetActiveThrust(void)
{
  commanderCachSelectorUpdate();
  return activeCach->targetVal[activeCach->activeSide].thrust;
}

static float commanderGetActiveRoll(void)
{
  return activeCach->targetVal[activeCach->activeSide].roll;
}

static float commanderGetActivePitch(void)
{
  return activeCach->targetVal[activeCach->activeSide].pitch;
}

static float commanderGetActiveYaw(void)
{
  return activeCach->targetVal[activeCach->activeSide].yaw;
}

static void commanderLevelRPY(void)
{
  commanderSetActiveRoll(0);
  commanderSetActivePitch(0);
  commanderSetActiveYaw(0);
}

static void commandeDropToGround(void)
{
  altHoldMode = false;
  commanderSetActiveThrust(0);
  commanderLevelRPY();
}


static void commanderCachSelectorUpdate(void)
{
  uint32_t tickNow = xTaskGetTickCount();

  /* Check inputs and prioritize. Extrx higher then crtp */
  if ((tickNow - extrxCach.timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE)
  {
    activeCach = &extrxCach;
  }
  else if ((tickNow - crtpCach.timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE)
  {
    activeCach = &crtpCach;
  }
  else if ((tickNow - extrxCach.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    activeCach = &extrxCach;
    commanderLevelRPY();
  }
  else if ((tickNow - crtpCach.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN)
  {
    activeCach = &crtpCach;
    commanderLevelRPY();
  }
  else
  {
    activeCach = &crtpCach;
    commandeDropToGround();
  }
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  crtpCach.targetVal[!crtpCach.activeSide] = *((struct CommanderCrtpValues*)pk->data);
  crtpCach.activeSide = !crtpCach.activeSide;
  crtpCach.timestamp = xTaskGetTickCount();

  if (crtpCach.targetVal[crtpCach.activeSide].thrust == 0)
  {
    thrustLocked = false;
  }
}

/**
 * Rotate Yaw so that the Crazyflie will change what is considered front.
 *
 * @param yawRad Amount of radians to rotate yaw.
 */
static void rotateYaw(setpoint_t *setpoint, float yawRad)
{
  float cosy = cosf(yawRad);
  float siny = sinf(yawRad);
  float originalRoll = setpoint->attitude.roll;
  float originalPitch = setpoint->attitude.pitch;

  setpoint->attitude.roll = originalRoll * cosy - originalPitch * siny;
  setpoint->attitude.pitch = originalPitch * cosy + originalRoll * siny;
}

/**
 * Yaw carefree mode means yaw will stay in world coordinates. So even though
 * the Crazyflie rotates around the yaw, front will stay the same as when it started.
 * This makes makes it a bit easier for beginners
 */
static void rotateYawCarefree(setpoint_t *setpoint, const state_t *state, bool reset)
{
  static float carefreeFrontAngle;

  if (reset)
  {
    carefreeFrontAngle = state->attitude.yaw;
  }

  float yawRad = (state->attitude.yaw - carefreeFrontAngle) * (float)M_PI / 180;
  rotateYaw(setpoint, yawRad);
}

/**
 * Update Yaw according to current setting
 */
#ifdef PLATFORM_CF1
static void yawModeUpdate(setpoint_t *setpoint, const state_t *state)
{
  switch (yawMode)
  {
    case CAREFREE:
      rotateYawCarefree(setpoint, state, carefreeResetFront);
      break;
    case PLUSMODE:
      // Default in plus mode. Do nothing
      break;
    case XMODE: // Fall though
    default:
      rotateYaw(setpoint, -45 * M_PI / 180);
      break;
  }
}
#else
static void yawModeUpdate(setpoint_t *setpoint, const state_t *state)
{
  switch (yawMode)
  {
    case CAREFREE:
      rotateYawCarefree(setpoint, state, carefreeResetFront);
      break;
    case PLUSMODE:
      rotateYaw(setpoint, 45 * M_PI / 180);
      break;
    case XMODE: // Fall though
    default:
      // Default in x-mode. Do nothing
      break;
  }
}
#endif

/* Public functions */
void commanderInit(void)
{
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);

  activeCach = &crtpCach;
  lastUpdate = xTaskGetTickCount();
  isInactive = true;
  thrustLocked = true;
  isInit = true;
}

bool commanderTest(void)
{
  crtpTest();
  return isInit;
}

void commanderExtrxSet(struct CommanderCrtpValues* val)
{
  extrxCach.targetVal[!extrxCach.activeSide] = *((struct CommanderCrtpValues*)val);
  extrxCach.activeSide = !extrxCach.activeSide;
  extrxCach.timestamp = xTaskGetTickCount();

  if (extrxCach.targetVal[extrxCach.activeSide].thrust == 0)
  {
    thrustLocked = false;
  }
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  // Thrust
  uint16_t rawThrust = commanderGetActiveThrust();

  if (thrustLocked || (rawThrust < MIN_THRUST)) {
    setpoint->thrust = 0;
  } else {
    setpoint->thrust = min(rawThrust, MAX_THRUST);
  }

  if (altHoldMode) {
    setpoint->thrust = 0;
    setpoint->mode.z = modeDisable;

    setpoint->velocity.z = ((float) rawThrust - 32767.f) / 32767.f;
  } else {
    setpoint->mode.z = modeDisable;
  }

  // roll/pitch
  if (posHoldMode) {
    setpoint->mode.x = modeVelocity;
    setpoint->mode.y = modeVelocity;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;

    setpoint->velocity.x = commanderGetActivePitch()/30.0f;
    setpoint->velocity.y = commanderGetActiveRoll()/30.0f;
    setpoint->attitude.roll  = 0;
    setpoint->attitude.pitch = 0;
  } else {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;

    setpoint->velocity.x = 0;
    setpoint->velocity.y = 0;
    setpoint->attitude.roll  = commanderGetActiveRoll();
    setpoint->attitude.pitch = commanderGetActivePitch();
  }

  // Yaw
  setpoint->attitudeRate.yaw  = commanderGetActiveYaw();
  yawModeUpdate(setpoint, state);

  setpoint->mode.yaw = modeVelocity;
}

// Params for flight modes
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, poshold, &posHoldMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
