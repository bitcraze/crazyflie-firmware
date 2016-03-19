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
#include "FreeRTOS.h"
#include "task.h"

#include "commander.h"
#include "crtp.h"
#include "configblock.h"
#include "param.h"

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

static bool isInit;
static CommanderCach crtpCach;
static CommanderCach extrxCach;
static CommanderCach* activeCach;

static uint32_t lastUpdate;
static bool isInactive;
static bool thrustLocked;
static bool altHoldMode = false;

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
  commanderSetAltHoldMode(false);
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

void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired)
{
  *eulerRollDesired  = commanderGetActiveRoll();
  *eulerPitchDesired = commanderGetActivePitch();
  *eulerYawDesired   = commanderGetActiveYaw();
}

void commanderGetAltHold(bool* altHold, float* altHoldChange)
{
  *altHold = altHoldMode; // Still in altitude hold mode
  *altHoldChange = altHoldMode ? ((float) commanderGetActiveThrust() - 32767.f) / 32767.f : 0.0; // Amount to change altitude hold target
}

void commanderSetAltHoldMode(bool altHoldModeNew)
{
	altHoldMode = altHoldModeNew;

	/**
	 * Dirty trick to ensure the altHoldChange variable remains zero after next call to commanderGetAltHold().
	 *
	 * This is needed since the commanderGetAltHold calculates the altHoldChange to -1 if altHoldMode is enabled
	 * with a simultaneous thrust command of 0.
	 *
	 * When altHoldChange is calculated to -1 when enabling altHoldMode, the altTarget will steadily decrease
	 * until thrust is commanded to correct the altitude, which is what we want to avoid.
	 */
	if(altHoldModeNew)
	{
	  commanderSetActiveThrust(32767);
	}
}

void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType)
{
  *rollType  = stabilizationModeRoll;
  *pitchType = stabilizationModePitch;
  *yawType   = stabilizationModeYaw;
}

void commanderGetThrust(uint16_t* thrust)
{
  uint16_t rawThrust = commanderGetActiveThrust();

  if (thrustLocked)
  {
    *thrust = 0;
  }
  else
  {
    if (rawThrust > MIN_THRUST)
    {
      *thrust = rawThrust;
    }
    else
    {
      *thrust = 0;
    }

    if (rawThrust > MAX_THRUST)
    {
      *thrust = MAX_THRUST;
    }
  }
}

YawModeType commanderGetYawMode(void)
{
  return yawMode;
}

bool commanderGetYawModeCarefreeResetFront(void)
{
  return carefreeResetFront;
}

// Params for flight modes
PARAM_GROUP_START(flightmode)
PARAM_ADD(PARAM_UINT8, althold, &altHoldMode)
PARAM_ADD(PARAM_UINT8, yawMode, &yawMode)
PARAM_ADD(PARAM_UINT8, yawRst, &carefreeResetFront)
PARAM_ADD(PARAM_UINT8, stabModeRoll, &stabilizationModeRoll)
PARAM_ADD(PARAM_UINT8, stabModePitch, &stabilizationModePitch)
PARAM_ADD(PARAM_UINT8, stabModeYaw, &stabilizationModeYaw)
PARAM_GROUP_STOP(flightmode)
