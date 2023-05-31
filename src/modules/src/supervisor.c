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
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "motors.h"
#include "power_distribution.h"
#include "supervisor.h"
#include "supervisor_state_machine.h"
#include "platform_defaults.h"
#include "crtp_localization_service.h"
#include "system.h"
#include "autoconf.h"

#define DEBUG_MODULE "SUP"
#include "debug.h"


#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT (M2T(1000))

// The minimum time (in ms) we need to see low thrust before saying that we are not flying anymore
#define IS_FLYING_HYSTERESIS_THRESHOLD M2T(2000)

#define COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

#ifndef CONFIG_MOTORS_REQUIRE_ARMING
  #define AUTO_ARMING 1
#else
  #define AUTO_ARMING 0
#endif

typedef struct {
  bool canFly;
  bool isFlying;
  bool isTumbled;
  bool isArmingActivated;
  uint16_t infoBitfield;
  uint8_t paramEmergencyStop;

  // Deprecated, remove after 2024-06-01
  int8_t deprecatedArmParam;

  // The time (in ticks) of the first tumble event. 0=no tumble
  uint32_t initialTumbleTick;

  // The time (in ticks) of the latest high thrust event. 0=no high thrust event yet
  uint32_t latestThrustTick;

  supervisorState_t state;

  // Copy of latest conditions, for logging
  supervisorConditionBits_t latestConditions;
  uint8_t doinfodump;
} SupervisorMem_t;

static SupervisorMem_t supervisorMem;

const static setpoint_t nullSetpoint;

void infoDump(const SupervisorMem_t* this);

bool supervisorCanFly() {
  return supervisorMem.canFly;
}

bool supervisorIsFlying() {
  return supervisorMem.isFlying;
}

bool supervisorIsTumbled() {
  return supervisorMem.isTumbled;
}

bool supervisorCanArm() {
  return supervisorStatePreFlChecksPassed == supervisorMem.state;
}

bool supervisorIsArmed() {
  return supervisorMem.isArmingActivated || supervisorMem.deprecatedArmParam;
}

bool supervisorRequestArming(const bool doArm) {
  if (doArm == supervisorMem.isArmingActivated) {
    return true;
  }

  if (doArm && !supervisorCanArm()) {
    return false;
  }

  supervisorMem.isArmingActivated = doArm;
  return true;
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
// Tumbling is defined as being tilted more than 60 degrees for one second, or more than 90 degrees for 30 ms.
// Free falling is considered a valid flight mode.
//
// Once a tumbled situation is identified, we can use this for instance to cut
// the thrust to the motors, avoiding the Crazyflie from running propellers at
// significant thrust when accidentally crashing into walls or the ground.
//
static bool isTumbledCheck(SupervisorMem_t* this, const sensorData_t *data, const uint32_t tick) {
  const float freeFallThreshold = 0.1;

  const float acceptedTiltAccZ = 0.5;  // 60 degrees tilt (when stationary)
  const uint32_t maxTiltTime = M2T(1000);

  const float acceptedUpsideDownAccZ = -0.0;  // 90 degrees tilt
  const uint32_t maxUpsideDownTime = M2T(30);

  const bool isFreeFalling = (fabsf(data->acc.z) < freeFallThreshold && fabsf(data->acc.y) < freeFallThreshold && fabsf(data->acc.x) < freeFallThreshold);
  if (isFreeFalling) {
    // Falling is OK, reset
    this->initialTumbleTick = 0;
  }

  const bool isTilted = (data->acc.z < acceptedTiltAccZ);
  if(isTilted) {  // Will also be true for up side down
    if (0 == this->initialTumbleTick) {
      // Start the clock
      this->initialTumbleTick = tick;
    }

    const uint32_t ticksBeingTumbled = tick - this->initialTumbleTick;

    const bool isUpSideDown = (data->acc.z < acceptedUpsideDownAccZ);
    if (isUpSideDown && (ticksBeingTumbled > maxUpsideDownTime)) {
      return true;
    }

    if (ticksBeingTumbled > maxTiltTime) {
      return true;
    }
  } else {
    // We're OK, reset
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

static void postTransitionActions(SupervisorMem_t* this, const supervisorState_t previousState) {
  const supervisorState_t newState = this->state;

  if (newState == supervisorStateReadyToFly) {
    DEBUG_PRINT("Ready to fly\n");
  }

  if (newState == supervisorStateLocked) {
    DEBUG_PRINT("Locked, reboot required\n");
  }

  if ((previousState == supervisorStateNotInitialized || previousState == supervisorStateReadyToFly || previousState == supervisorStateFlying) &&
      newState != supervisorStateReadyToFly && newState != supervisorStateFlying) {
    DEBUG_PRINT("Can not fly\n");
  }

  if (newState != supervisorStateReadyToFly &&
      newState != supervisorStateFlying &&
      newState != supervisorStateWarningLevelOut) {
    supervisorRequestArming(false);
  }

  // We do not require an arming action by the user, auto arm
  if (AUTO_ARMING || this->deprecatedArmParam) {
    if (newState == supervisorStatePreFlChecksPassed) {
      supervisorRequestArming(true);
    }
  }
}

static supervisorConditionBits_t updateAndPopulateConditions(SupervisorMem_t* this, const sensorData_t *sensors, const setpoint_t* setpoint, const uint32_t currentTick) {
  supervisorConditionBits_t conditions = 0;

  if (supervisorIsArmed()) {
    conditions |= SUPERVISOR_CB_ARMED;
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

  this->infoBitfield = 0;
  if (supervisorCanArm()) {
    this->infoBitfield |= 0x0001;
  }
  if (supervisorIsArmed()) {
    this->infoBitfield |= 0x0002;
  }
  if(AUTO_ARMING || this->deprecatedArmParam) {
    this->infoBitfield |= 0x0004;
  }
  if (this->canFly) {
    this->infoBitfield |= 0x0008;
  }
  if (this->isFlying) {
    this->infoBitfield |= 0x0010;
  }
  if (this->isTumbled) {
    this->infoBitfield |= 0x0020;
  }
}

void supervisorUpdate(const sensorData_t *sensors, const setpoint_t* setpoint, stabilizerStep_t stabilizerStep) {
  if (!RATE_DO_EXECUTE(RATE_SUPERVISOR, stabilizerStep)) {
    return;
  }

  SupervisorMem_t* this = &supervisorMem;
  const uint32_t currentTick = xTaskGetTickCount();

  const supervisorConditionBits_t conditions = updateAndPopulateConditions(this, sensors, setpoint, currentTick);
  const supervisorState_t newState = supervisorStateUpdate(this->state, conditions);
  if (this->state != newState) {
    const supervisorState_t previousState = this->state;
    this->state = newState;
    postTransitionActions(this, previousState);
  }

  this->latestConditions = conditions;
  updateLogData(this, conditions);
  if (this->doinfodump) {
    this->doinfodump = 0;
    infoDump(this);
  }
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

void infoDump(const SupervisorMem_t* this) {
  DEBUG_PRINT("Supervisor info ---\n");
  DEBUG_PRINT("State: %s\n", supervisorGetStateName(this->state));
  DEBUG_PRINT("Conditions: (0x%lx)\n", this->latestConditions);
  for (supervisorConditions_t condition = 0; condition < supervisorCondition_NrOfConditions; condition++) {
    const supervisorConditionBits_t bit = 1 << condition;
    int bitValue = 0;
    if (this->latestConditions & bit) {
      bitValue = 1;
    }

    DEBUG_PRINT("  %s (0x%lx): %u\n", supervisorGetConditionName(condition), bit, bitValue);
  }
}



/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
/**
 * @brief Nonzero if system is ready to fly.
 *
 * Deprecated, will be removed after 2024-06-01. Use superv.info instead
 */
LOG_ADD_CORE(LOG_UINT8, canfly, &supervisorMem.canFly)
/**
 * @brief Nonzero if the system thinks it is flying
 *
 * Deprecated, will be removed after 2024-06-01. Use superv.info instead
 */
LOG_ADD_CORE(LOG_UINT8, isFlying, &supervisorMem.isFlying)
/**
 * @brief Nonzero if the system thinks it is tumbled/crashed
 *
 * Deprecated, will be removed after 2024-06-01. Use superv.info instead
 */
LOG_ADD_CORE(LOG_UINT8, isTumbled, &supervisorMem.isTumbled)
LOG_GROUP_STOP(sys)


PARAM_GROUP_START(stabilizer)
/**
 * @brief If set to nonzero will turn off motors
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &supervisorMem.paramEmergencyStop)
PARAM_GROUP_STOP(stabilizer)


PARAM_GROUP_START(system)

/**
 * @brief Set to nonzero to arm the system. A nonzero value enables the auto arm functionality
 *
 * Deprecated, will be removed after 2024-06-01. Use the CRTP `PlatformCommand` `armSystem` on the CRTP_PORT_PLATFORM port instead.
 */
PARAM_ADD_CORE(PARAM_INT8, arm, &supervisorMem.deprecatedArmParam)
PARAM_GROUP_STOP(system)


/**
 * The purpose of the supervisor is to monitor the system and its state. Depending on the situation, the supervisor
 * can enable/disable functionality as well as take action to protect the system or humans close by.
 */
LOG_GROUP_START(supervisor)
/**
 * @brief Bitfield containing information about the supervisor status
 * Bit 0 = Can be armed - the system can be armed and will accept an arming command
 * Bit 1 = is armed - the system is armed
 * Bit 2 = auto arm - the system is configured to automatically arm
 * Bit 3 = can fly - the Crazyflie is ready to fly
 * Bit 4 = is flying - the Crazyflie is flying.
 * Bit 5 = is tumbled - the Crazyflie is up side down.
 */
LOG_ADD(LOG_UINT16, info, &supervisorMem.infoBitfield)
LOG_GROUP_STOP(supervisor)


/**
 * The purpose of the supervisor is to monitor the system and its state. Depending on the situation, the supervisor
 * can enable/disable functionality as well as take action to protect the system or humans close by.
 */
PARAM_GROUP_START(supervisor)
/**
 * @brief Set to nonzero to dump information about the current supervisor state to the console log
 */
PARAM_ADD(PARAM_UINT8, infdmp, &supervisorMem.doinfodump)
PARAM_GROUP_STOP(supervisor)
