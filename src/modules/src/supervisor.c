/*
*    ||          ____  _ __
* +------+      / __ )(_) /_______________ _____  ___
* | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
* +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
*  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
*
* Crazyflie control firmware
*
* Copyright (C) 2021 Bitcraze AB
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
#include "platform_defaults.h"
#include "crtp_localization_service.h"
#include "system.h"


#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT (M2T(1000))

/* Minimum summed motor PWM that means we are flying */
#define SUPERVISOR_FLIGHT_THRESHOLD 1000

/* Number of times in a row we need to see a condition before acting upon it */
#define TUMBLE_HYSTERESIS_THRESHOLD 30

// TODO krri rename
#define COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

typedef enum {
  actionNone = 0,               // All is normal, we can fly
  actionMotorsDisabled,         // Motors are disabled for now, for instance when not armed
  actionLevelOut,               // Level out, all is not lost yet
  actionStopMotorsAndFreeFall,  // Stop motors permanently and free fall
} action_t;

typedef struct {
  bool canFly;
  bool isFlying;
  bool isTumbled;

  uint32_t tumbleHysteresis;

  action_t action;
} SupervisorState_t;

static SupervisorState_t supervisorState;

const static setpoint_t nullSetpoint;


// TODO krri
// * Add reset functionality

bool supervisorCanFly() {
  return supervisorState.canFly;
}

bool supervisorIsFlying() {
  return supervisorState.isFlying;
}

bool supervisorIsTumbled() {
  return supervisorState.isTumbled;
}

//
// We say we are flying if the sum of the ratios of all motors giving thrust
// is above a certain threshold.
//
static bool isFlyingCheck() {
  int sumRatio = 0;
  for (int i = 0; i < NBR_OF_MOTORS; ++i) {
    sumRatio += powerDistributionMotorType(i) * motorsGetRatio(i);
  }

  return sumRatio > SUPERVISOR_FLIGHT_THRESHOLD;
}

//
// We say we are tumbled when the accelerometer reports negative values.
//
// Once a tumbled situation is identified, we can use this for instance to cut
// the thrust to the motors, avoiding the Crazyflie from running propellers at
// significant thrust when accidentally crashing into walls or the ground.
//
static bool isTumbledCheck(SupervisorState_t* this, const sensorData_t *data) {
  const float tolerance = -0.5;
  //
  // We need a TUMBLE_HYSTERESIS_THRESHOLD amount of readings that indicate
  // that we are tumbled before we act on it. This is to reduce false positives.
  //
  if (data->acc.z <= tolerance) {
    this->tumbleHysteresis++;
    if (this->tumbleHysteresis > TUMBLE_HYSTERESIS_THRESHOLD) {
      return true;
    }
  } else {
    this->tumbleHysteresis = 0;
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


static action_t setAction(SupervisorState_t* this, const action_t newAction) {
  switch(this->action) {
    case actionNone:
      this->action = newAction;
      break;
    case actionMotorsDisabled:
      if (newAction == actionNone || newAction == actionStopMotorsAndFreeFall) {
        this->action = newAction;
      }
      break;
    case actionLevelOut:
      if (newAction == actionNone || newAction == actionStopMotorsAndFreeFall) {
        this->action = newAction;
      }
      break;
    case actionStopMotorsAndFreeFall:
      // Intentionally empty
      break;
    default:
      ASSERT_FAILED();
      break;
  }

  return this->action;
}

void supervisorUpdate(const sensorData_t *sensors, const setpoint_t* setpoint) {
  SupervisorState_t* this = &supervisorState;
  const uint32_t currentTick = xTaskGetTickCount();

  this->isFlying = isFlyingCheck();
  this->isTumbled = isTumbledCheck(this, sensors);

  // canFly is kept for backwards compatibility. TODO krri deprecate?
  this->canFly = true;

  // Reset action (if possible)
  setAction(this, actionNone);

  if (this->isTumbled) {
    #if SUPERVISOR_TUMBLE_CHECK_ENABLE
    if (this->isFlying) {
      setAction(this, actionStopMotorsAndFreeFall);
    } else {
      setAction(this, actionMotorsDisabled);
    }
    #endif

    this->canFly = false;
  }

  if (locSrvIsEmergencyStopRequested()) {
      setAction(this, actionStopMotorsAndFreeFall);
  }

  if (pmIsChargerConnected()) {
    setAction(this, actionMotorsDisabled);
    this->canFly = false;
  }

  if (! checkEmergencyStopWatchdog(currentTick)) {
    if (this->isFlying){
      setAction(this, actionStopMotorsAndFreeFall);
    } else {
      setAction(this, actionMotorsDisabled);
    }
  }

  if (! systemIsArmed()) {
      setAction(this, actionMotorsDisabled);
  }

  const uint32_t setpointAge = currentTick - setpoint->timestamp;
  if (this->isFlying) {
    if (setpointAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
      setAction(this, actionLevelOut);
    }

    if (setpointAge > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
      setAction(this, actionStopMotorsAndFreeFall);
    }
  } else {
    if (setpointAge > COMMANDER_WDT_TIMEOUT_STABILIZE) {
      setAction(this, actionMotorsDisabled);
    }
  }
}


void supervisorOverrideSetpoint(setpoint_t* setpoint) {
  SupervisorState_t* this = &supervisorState;
  switch(this->action){
    case actionNone:
      // Do nothing
      break;
    case actionLevelOut:
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
    case actionMotorsDisabled:
      // Fall through
    case actionStopMotorsAndFreeFall:
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
      break;
  }
}


/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
/**
 * @brief If nonzero if system is ready to fly.
 */
LOG_ADD_CORE(LOG_UINT8, canfly, &supervisorState.canFly)
/**
 * @brief Nonzero if the system thinks it is flying
 */
LOG_ADD_CORE(LOG_UINT8, isFlying, &supervisorState.isFlying)
/**
 * @brief Nonzero if the system thinks it is tumbled/crashed
 */
LOG_ADD_CORE(LOG_UINT8, isTumbled, &supervisorState.isTumbled)
LOG_GROUP_STOP(sys)


// TODO Krri rename? deprecate?
PARAM_GROUP_START(stabilizer)
/**
 * @brief If set to nonzero will turn off power
 */

// TODO krri How to handle?
// PARAM_ADD_CORE(PARAM_UINT8, stop, &supervisorState.areMotorsLocked)
PARAM_GROUP_STOP(stabilizer)
