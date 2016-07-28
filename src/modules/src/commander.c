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

/**
 * Commander control data
 */
typedef struct
{
  struct CommanderCrtpValues targetVal[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} CommanderCache;

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
  XMODE     = 2, // X-mode. M1 & M4 are defined as front
} YawModeType;

static bool isInit;
static CommanderCache crtpCache;
static CommanderCache extrxCache;
static CommanderCache* activeCache;

static uint32_t lastUpdate;
static bool isInactive;
static bool thrustLocked;
static bool altHoldMode = false;
static bool posHoldMode = false;
static bool posSetMode = false;

static RPYType stabilizationModeRoll  = ANGLE; // Current stabilization type of roll (rate or angle)
static RPYType stabilizationModePitch = ANGLE; // Current stabilization type of pitch (rate or angle)
static RPYType stabilizationModeYaw   = RATE;  // Current stabilization type of yaw (rate or angle)

static YawModeType yawMode = DEFAULT_YAW_MODE; // Yaw mode configuration
static bool carefreeResetFront;             // Reset what is front in carefree mode

static void commanderCrtpCB(CRTPPacket* pk);
static void commanderCacheSelectorUpdate(void);

/* Private functions */
static void commanderSetActiveThrust(uint16_t thrust)
{
  activeCache->targetVal[activeCache->activeSide].thrust = thrust;
}

static void commanderSetActiveRoll(float roll)
{
  activeCache->targetVal[activeCache->activeSide].roll = roll;
}

static void commanderSetActivePitch(float pitch)
{
  activeCache->targetVal[activeCache->activeSide].pitch = pitch;
}

static void commanderSetActiveYaw(float yaw)
{
  activeCache->targetVal[activeCache->activeSide].yaw = yaw;
}

static uint16_t commanderGetActiveThrust(void)
{
  commanderCacheSelectorUpdate();
  return activeCache->targetVal[activeCache->activeSide].thrust;
}

static float commanderGetActiveRoll(void)
{
  return activeCache->targetVal[activeCache->activeSide].roll;
}

static float commanderGetActivePitch(void)
{
  return activeCache->targetVal[activeCache->activeSide].pitch;
}

static float commanderGetActiveYaw(void)
{
  return activeCache->targetVal[activeCache->activeSide].yaw;
}

static void commanderLevelRPY(void)
{
  commanderSetActiveRoll(0);
  commanderSetActivePitch(0);
  commanderSetActiveYaw(0);
}

static void commanderDropToGround(void)
{
  altHoldMode = false;
  commanderSetActiveThrust(0);
  commanderLevelRPY();
}

static void commanderCacheSelectorUpdate(void)
{
  uint32_t tickNow = xTaskGetTickCount();

  /* Check inputs and prioritize. Extrx higher then crtp */
  if ((tickNow - extrxCache.timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE) {
    activeCache = &extrxCache;
  } else if ((tickNow - crtpCache.timestamp) < COMMANDER_WDT_TIMEOUT_STABILIZE) {
    activeCache = &crtpCache;
  } else if ((tickNow - extrxCache.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    activeCache = &extrxCache;
    commanderLevelRPY();
  } else if ((tickNow - crtpCache.timestamp) < COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    activeCache = &crtpCache;
    commanderLevelRPY();
  } else {
    activeCache = &crtpCache;
    commanderDropToGround();
  }
}

static void commanderCrtpCB(CRTPPacket* pk)
{
  crtpCache.targetVal[!crtpCache.activeSide] = *((struct CommanderCrtpValues*)pk->data);
  crtpCache.activeSide = !crtpCache.activeSide;
  crtpCache.timestamp = xTaskGetTickCount();

  if (crtpCache.targetVal[crtpCache.activeSide].thrust == 0) {
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

  if (reset) {
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
    case XMODE: // Fall through
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
    case XMODE: // Fall through
    default:
      // Default in x-mode. Do nothing
      break;
  }
}
#endif

/* Public functions */
void commanderInit(void)
{
  if(isInit) {
    return;
  }

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_COMMANDER, commanderCrtpCB);

  activeCache = &crtpCache;
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

void commanderExtrxSet(const struct CommanderCrtpValues *val)
{
  extrxCache.targetVal[!extrxCache.activeSide] = *((struct CommanderCrtpValues*)val);
  extrxCache.activeSide = !extrxCache.activeSide;
  extrxCache.timestamp = xTaskGetTickCount();

  if (extrxCache.targetVal[extrxCache.activeSide].thrust == 0) {
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
    setpoint->mode.z = modeVelocity;

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
  } else if (posSetMode && commanderGetActiveThrust() != 0) {
    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;
    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.yaw = modeAbs;

    setpoint->position.x = -commanderGetActivePitch();
    setpoint->position.y = commanderGetActiveRoll();
    setpoint->position.z = commanderGetActiveThrust()/1000.0f;

    setpoint->attitude.roll  = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitude.yaw = commanderGetActiveYaw();
    setpoint->thrust = 0;
  } else {
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;

    if (stabilizationModeRoll == RATE) {
      setpoint->mode.roll = modeVelocity;
      setpoint->attitudeRate.roll = commanderGetActiveRoll();
      setpoint->attitude.roll = 0;
    } else {
      setpoint->mode.roll = modeAbs;
      setpoint->attitudeRate.roll = 0;
      setpoint->attitude.roll = commanderGetActiveRoll();
    }

    if (stabilizationModePitch == RATE) {
      setpoint->mode.pitch = modeVelocity;
      setpoint->attitudeRate.pitch = commanderGetActivePitch();
      setpoint->attitude.pitch = 0;
    } else {
      setpoint->mode.pitch = modeAbs;
      setpoint->attitudeRate.pitch = 0;
      setpoint->attitude.pitch = commanderGetActivePitch();
    }

    setpoint->velocity.x = 0;
    setpoint->velocity.y = 0;
  }

  // Yaw
  if (!posSetMode) {
    setpoint->attitudeRate.yaw  = commanderGetActiveYaw();
    yawModeUpdate(setpoint, state);

    setpoint->mode.yaw = modeVelocity;
  }
}

// Params for flight modes
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, poshold, &posHoldMode)
PARAM_ADD(PARAM_UINT8, posSet, &posSetMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
