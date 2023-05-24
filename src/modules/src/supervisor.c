/*
*    ||          ____  _ __
* +------+      / __ )(_) /_______________ _____  ___
* | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
* +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
*  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
*
* Crazyflie control firmware
*
* Copyright (C) 2021 - 2023 Bitcraze AB
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
* supervisor.c - Keep track of system state
*/

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "log.h"
#include "param.h"
#include "motors.h"
#include "power_distribution.h"
#include "pm.h"
#include "supervisor.h"
#include "supervisor_state_machine.h"
#include "platform_defaults.h"
#include "crtp_localization_service.h"
#include "system.h"

#define DEBUG_MODULE "SUP"
#include "debug.h"


#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT (M2T(1000))

// The minimum time (in ms) we need to see tumble condition before acting on it
#define TUMBLE_HYSTERESIS_THRESHOLD M2T(30)

// The minimum time (in ms) we need to see low thrust before saying that we are not flying anymore
#define IS_FLYING_HYSTERESIS_THRESHOLD M2T(2000)

#define COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

typedef struct {
  bool canFly;
  bool isFlying;
  bool isTumbled;
  uint8_t paramEmergencyStop;

  // The time (in ticks) of the first tumble event. 0=no tumble
  uint32_t initialTumbleTick;

  // The time (in ticks) of the latest high thrust event. 0=no high thrust event yet
  uint32_t latestThrustTick;

  supervisorState_t state;
} SupervisorMem_t;

static SupervisorMem_t supervisorMem;

const static setpoint_t nullSetpoint;


bool supervisorCanFly() {
  return supervisorMem.canFly;
}

bool supervisorIsFlying() {
  return supervisorMem.isFlying;
}

bool supervisorIsTumbled() {
  return supervisorMem.isTumbled;
}

//
// We say we are flying if one or more motors are running over the idle thrust.
//
static bool isFlyingCheck(SupervisorMem_t* this, const uint32_t tick) {
  bool isThrustOverIdle = false;
  const uint32_t idleThrust = powerDistributionGetIdleThrust();
  for (int i = 0; i < NBR_OF_MOTORS; ++i) {
    const uint32_t ratio = powerDistributionMotorType(i) * motorsGetRatio(i);
    if (ratio > idleThrust) {
      isThrustOverIdle = true;
      break;
    }
  }

  if (isThrustOverIdle) {
    this->latestThrustTick = tick;
  }

  bool result = false;
  if (0 != this->latestThrustTick) {
    if ((tick - this->latestThrustTick) < IS_FLYING_HYSTERESIS_THRESHOLD) {
      result = true;
    }
  }

  return result;
}

//
// We say we are tumbled when the accelerometer reports negative values.
//
// Once a tumbled situation is identified, we can use this for instance to cut
// the thrust to the motors, avoiding the Crazyflie from running propellers at
// significant thrust when accidentally crashing into walls or the ground.
//
static bool isTumbledCheck(SupervisorMem_t* this, const sensorData_t *data, const uint32_t tick) {
  const float tolerance = -0.5;
  //
  // We need a TUMBLE_HYSTERESIS_THRESHOLD amount of readings that indicate
  // that we are tumbled before we act on it. This is to reduce false positives.
  //
  if (data->acc.z <= tolerance) {
    if (0 == this->initialTumbleTick) {
      this->initialTumbleTick = tick;
    }
    if ((tick - this->initialTumbleTick) > TUMBLE_HYSTERESIS_THRESHOLD) {
      return true;
    }
  } else {
    this->initialTumbleTick = 0;
  }

  return false;
}

static bool checkEmergencyStopWatchdog(const uint32_t tick) {
  bool isOk = true;

  const uint32_t latestNotification = locSrvGetEmergencyStopWatchdogNotificationTick();
  if (latestNotification > 0) {
    isOk = tick < (latestNotification + DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT);
  }

  return isOk;
}

static void transitionActions(const supervisorState_t currentState, const supervisorState_t newState) {
  if (newState == supervisorStateReadyToFly) {
    DEBUG_PRINT("Ready to fly\n");
  }

  if (newState == supervisorStateLocked) {
    DEBUG_PRINT("Locked, reboot required\n");
  }

  if ((currentState == supervisorStateNotInitialized || currentState == supervisorStateReadyToFly || currentState == supervisorStateFlying) &&
      newState != supervisorStateReadyToFly && newState != supervisorStateFlying) {
    DEBUG_PRINT("Can not fly\n");
  }
}

static supervisorConditionBits_t updateAndpopulateConditions(SupervisorMem_t* this, const sensorData_t *sensors, const setpoint_t* setpoint, const uint32_t currentTick) {
  supervisorConditionBits_t conditions = 0;

  if (systemIsArmed()) {
    conditions |= SUPERVISOR_CB_ARMED;
  }

  if (pmIsChargerConnected()) {
    conditions |= SUPERVISOR_CB_CHARGER_CONNECTED;
  }

  const bool isFlying = isFlyingCheck(this, currentTick);
  if (isFlying) {
    conditions |= SUPERVISOR_CB_IS_FLYING;
  }

  const bool isTumbled = isTumbledCheck(this, sensors, currentTick);
  if (isTumbled) {
    conditions |= SUPERVISOR_CB_IS_TUMBLED;
  }

  const uint32_t setpointAge = currentTick - setpoint->timestamp;
  if (setpointAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    conditions |= SUPERVISOR_CB_COMMANDER_WDT_WARNING;
  }
  if (setpointAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    conditions |= SUPERVISOR_CB_COMMANDER_WDT_TIMEOUT;
  }

  if (!checkEmergencyStopWatchdog(currentTick)) {
    conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
  }

  if (locSrvIsEmergencyStopRequested()) {
    conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
  }

  if (this->paramEmergencyStop) {
    conditions |= SUPERVISOR_CB_EMERGENCY_STOP;
  }

  return conditions;
}

static void updateLogData(SupervisorMem_t* this, const supervisorConditionBits_t conditions) {
  this->canFly = supervisorAreMotorsAllowedToRun();
  this->isFlying = (this->state == supervisorStateFlying) || (this->state == supervisorStateWarningLevelOut);
  this->isTumbled = (conditions & SUPERVISOR_CB_IS_TUMBLED) != 0;
}

void supervisorUpdate(const sensorData_t *sensors, const setpoint_t* setpoint, stabilizerStep_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
    return;
  }

  SupervisorMem_t* this = &supervisorMem;
  const uint32_t currentTick = xTaskGetTickCount();

  const supervisorConditionBits_t conditions = updateAndpopulateConditions(this, sensors, setpoint, currentTick);
  const supervisorState_t newState = supervisorStateUpdate(this->state, conditions);
  if (this->state != newState) {
    transitionActions(this->state, newState);
    this->state = newState;
  }

  updateLogData(this, conditions);
}

void supervisorOverrideSetpoint(setpoint_t* setpoint) {
  SupervisorMem_t* this = &supervisorMem;
  switch(this->state){
    case supervisorStateReadyToFly:
      // Fall through
    case supervisorStateFlying:
      // Do nothing
      break;

    case supervisorStateWarningLevelOut:
      setpoint->mode.x = modeDisable;
      setpoint->mode.y = modeDisable;
      setpoint->mode.roll = modeAbs;
      setpoint->mode.pitch = modeAbs;
      setpoint->mode.yaw = modeVelocity;
      setpoint->attitude.roll = 0;
      setpoint->attitude.pitch = 0;
      setpoint->attitudeRate.yaw = 0;
      // Keep Z as it is
      break;

    default:
      // Replace with null setpoint to stop motors
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
      break;
  }
}

bool supervisorAreMotorsAllowedToRun() {
  SupervisorMem_t* this = &supervisorMem;
  return (this->state == supervisorStateReadyToFly) ||
         (this->state == supervisorStateFlying) ||
         (this->state == supervisorStateWarningLevelOut);
}

/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
/**
 * @brief Nonzero if system is ready to fly.
 */
LOG_ADD_CORE(LOG_UINT8, canfly, &supervisorMem.canFly)
/**
 * @brief Nonzero if the system thinks it is flying
 */
LOG_ADD_CORE(LOG_UINT8, isFlying, &supervisorMem.isFlying)
/**
 * @brief Nonzero if the system thinks it is tumbled/crashed
 */
LOG_ADD_CORE(LOG_UINT8, isTumbled, &supervisorMem.isTumbled)
LOG_GROUP_STOP(sys)


PARAM_GROUP_START(stabilizer)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &supervisorMem.paramEmergencyStop)
PARAM_GROUP_STOP(stabilizer)
