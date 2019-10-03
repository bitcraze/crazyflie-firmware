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
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"
#include "pulp_shield.h"
#include "stm32f4xx_rcc.h"

#include "param.h"
#include "debug.h"
#include "log.h"

static bool isInit;
const static setpoint_t nullSetpoint;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
static bool enableHighLevel = false;

/* PULP-Shield variables */
static bool enablePULPShield  = false;
static bool PULPRunning       = false;
static bool landing           = false;
static uint32_t landingStep, lastLandingUpdate;
static float PULP_vel_x, PULP_pos_z, PULP_att_yaw;

QueueHandle_t setpointQueue;
QueueHandle_t priorityQueue;

/* Public functions */
void commanderInit(void)
{
  setpointQueue = xQueueCreate(1, sizeof(setpoint_t));
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = xQueueCreate(1, sizeof(int));
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpCommanderInit();
  crtpCommanderHighLevelInit();
  lastUpdate = xTaskGetTickCount();
  lastLandingUpdate = xTaskGetTickCount();

  /* PULP-Shield initialization */
  PULP_vel_x   = 0.0f;
  PULP_pos_z   = 0.0f;
  PULP_att_yaw = 0.0f;

  isInit = true;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;
  xQueuePeek(priorityQueue, &currentPriority, 0);

  if (priority >= currentPriority) {
    setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
  }
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();

  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    if (enableHighLevel) {
      crtpCommanderHighLevelGetSetpoint(setpoint, state);
    }
    if (!enableHighLevel || crtpCommanderHighLevelIsStopped()) {
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
    }
  } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    xQueueOverwrite(priorityQueue, &priorityDisable);
    // Leveling ...
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
    // Keep Z as it is
  }

  #ifdef PULP_SHIELD

  if(enablePULPShield) {

    if(!PULPRunning) {
      //Turning on the Power-Enable on the Shield
      PULPShieldOn();
      PULPRunning = true;
      landing     = true;
      landingStep = 1;
    }

    /* Watchdog triggering emergency landing in case of missing SPI
     * communication from PULP. Typically due to FLL issue on GAP8 SoC */
#ifdef WATCH_DOG_EN
    if(xTaskGetTickCount()-getLastPULPUpdate()>ERROR_THRESHOLD) {
      enablePULPShield  = false;
      PULPRunning       = false;
      DEBUG_PRINT("EMERGENCY STOP!\n");
    }
#endif

    setpoint->mode.x            = PULPShieldGetSetpoint().mode.x;
    setpoint->mode.y            = PULPShieldGetSetpoint().mode.y;
    setpoint->mode.z            = PULPShieldGetSetpoint().mode.z;
    setpoint->mode.pitch        = modeDisable;
    setpoint->mode.roll         = modeDisable;
    setpoint->mode.yaw          = PULPShieldGetSetpoint().mode.yaw;
    setpoint->mode.quat         = modeDisable;
    setpoint->velocity_body     = true;
    setpoint->position.z        = PULPShieldGetSetpoint().position.z;
    setpoint->velocity.x        = PULPShieldGetSetpoint().velocity.x;
    setpoint->velocity.y        = PULPShieldGetSetpoint().velocity.y;
    setpoint->attitudeRate.yaw  = PULPShieldGetSetpoint().attitudeRate.yaw;
  }
  else {
    if(landing) {
      if(landingStep==PULP_STEPS_LANDING) {
        landing = false;
        landingStep = 1;
      }
      else {

        setpoint->mode.x              = modeDisable;
        setpoint->mode.y              = modeDisable;
        setpoint->mode.z              = modeAbs;
        setpoint->mode.roll           = modeAbs;
        setpoint->mode.pitch          = modeAbs;
        setpoint->mode.yaw            = modeVelocity;
        setpoint->mode.quat           = modeDisable;
        setpoint->velocity_body       = true;
        setpoint->position.x          = 0.0f;
        setpoint->position.y          = 0.0f;
        setpoint->position.z          = PULP_TARGET_H-((PULP_TARGET_H/PULP_STEPS_LANDING*1.0f)*(landingStep*1.0f));
        setpoint->velocity.x          = 0.0f;
        setpoint->velocity.y          = 0.0f;
        setpoint->velocity.z          = 0.0f;
        setpoint->acceleration.x      = 0.0f;
        setpoint->acceleration.y      = 0.0f;
        setpoint->acceleration.z      = 0.0f;
        setpoint->attitude.pitch      = 0.0f;
        setpoint->attitude.roll       = 0.0f;
        setpoint->attitude.yaw        = 0.0f;
        setpoint->attitudeRate.pitch  = 0.0f;
        setpoint->attitudeRate.roll   = 0.0f;
        setpoint->attitudeRate.yaw    = 0.0f;

        if((currentTime-lastLandingUpdate) > PULP_LANDING_RATE) {
          landingStep++;
          lastLandingUpdate = xTaskGetTickCount();
        }
      }
    }
    else {

      PULPShieldOff();

      setpoint->mode.x              = modeDisable;
      setpoint->mode.y              = modeDisable;
      setpoint->mode.z              = modeDisable;
      setpoint->mode.roll           = modeDisable;
      setpoint->mode.pitch          = modeDisable;
      setpoint->mode.yaw            = modeDisable;
      setpoint->mode.quat           = modeDisable;
      setpoint->velocity_body       = true;
      setpoint->position.x          = 0.0f;
      setpoint->position.y          = 0.0f;
      setpoint->position.z          = 0.0f;
      setpoint->velocity.x          = 0.0f;
      setpoint->velocity.y          = 0.0f;
      setpoint->velocity.z          = 0.0f;
      setpoint->acceleration.x      = 0.0f;
      setpoint->acceleration.y      = 0.0f;
      setpoint->acceleration.z      = 0.0f;
      setpoint->attitude.pitch      = 0.0f;
      setpoint->attitude.roll       = 0.0f;
      setpoint->attitude.yaw        = 0.0f;
      setpoint->attitudeRate.pitch  = 0.0f;
      setpoint->attitudeRate.roll   = 0.0f;
      setpoint->attitudeRate.yaw    = 0.0f;

      PULPRunning  = false;
    }
  }

  // only for logging
  PULP_vel_x          = setpoint->velocity.x;
  PULP_pos_z          = setpoint->position.z;
  PULP_att_yaw        = setpoint->attitudeRate.yaw/YAW_SCALING;

#endif // PULP_SHIELD

}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;
  xQueuePeek(priorityQueue, &priority, 0);
  return priority;
}

PARAM_GROUP_START(commander)
PARAM_ADD(PARAM_UINT8, enHighLevel, &enableHighLevel)
PARAM_ADD(PARAM_UINT8, enPULPShield, &enablePULPShield)
PARAM_GROUP_STOP(commander)

LOG_GROUP_START(PULP)
LOG_ADD(LOG_FLOAT, PULP_x, &PULP_vel_x)
LOG_ADD(LOG_FLOAT, PULP_z, &PULP_pos_z)
LOG_ADD(LOG_FLOAT, PULP_yaw, &PULP_att_yaw)
LOG_GROUP_STOP(PULP)