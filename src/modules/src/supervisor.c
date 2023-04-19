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
#include "stabilizer.h"
#include "supervisor.h"
#include "platform_defaults.h"
#include "crtp_localization_service.h"
#include "system.h"


#define DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT (M2T(1000))


/* Minimum summed motor PWM that means we are flying */
#define SUPERVISOR_FLIGHT_THRESHOLD 1000

/* Number of times in a row we need to see a condition before acting upon it */
#define SUPERVISOR_HYSTERESIS_THRESHOLD 30

static bool canFly;
static bool isFlying;
static bool isTumbled;
static bool areMotorsLocked = false;

// TODO krri
// * Add reset functionality

bool supervisorCanFly()
{
  return canFly;
}

bool supervisorIsFlying()
{
  return isFlying;
}

bool supervisorIsTumbled()
{
  return isTumbled;
}

//
// We say we are flying if the sum of the ratios of all motors giving thrust
// is above a certain threshold.
//
static bool isFlyingCheck()
{
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
static bool isTumbledCheck(const sensorData_t *data)
{
  const float tolerance = -0.5;
  static uint32_t hysteresis = 0;
  //
  // We need a SUPERVISOR_HYSTERESIS_THRESHOLD amount of readings that indicate
  // that we are tumbled before we act on it. This is to reduce false positives.
  //
  if (data->acc.z <= tolerance) {
    hysteresis++;
    if (hysteresis > SUPERVISOR_HYSTERESIS_THRESHOLD) {
      return true;
    }
  } else {
    hysteresis = 0;
  }

  return false;
}

static bool checkEmergencyStopWatchdog() {
  bool isOk = true;
  const uint32_t latestNotification = locSrvGetEmergencyStopWatchdogNotificationTick();
  if (latestNotification > 0) {
    const uint32_t tick = xTaskGetTickCount();
    isOk = tick < (latestNotification + DEFAULT_EMERGENCY_STOP_WATCHDOG_TIMEOUT);
  }

  return isOk;
}

bool supervisorUpdate(const sensorData_t *sensors)
{
  isFlying = isFlyingCheck();
  isTumbled = isTumbledCheck(sensors);

  bool areMotorsAllowedToSpin = true;

  // canFly is kept for backwards compatibility. TODO krri deprecate?
  canFly = true;

  if (isTumbled) {
    #if SUPERVISOR_TUMBLE_CHECK_ENABLE
    // It is OK to tumble before flying
    if (isFlying) {
      areMotorsAllowedToSpin = false;
      areMotorsLocked = true;
    }
    #endif

    canFly = false;
  }

  if (locSrvIsEmergencyStopRequested()) {
    areMotorsAllowedToSpin = false;
    areMotorsLocked = true;
  }

  if (pmIsChargerConnected()) {
    areMotorsAllowedToSpin = false;
    canFly = false;
  }

  if (! checkEmergencyStopWatchdog()) {
    areMotorsAllowedToSpin = false;
  }

  if (! systemIsArmed()) {
    areMotorsAllowedToSpin = false;
  }

  if (areMotorsLocked) {
    areMotorsAllowedToSpin = false;
  }

  return areMotorsAllowedToSpin;
}

/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
/**
 * @brief If nonzero if system is ready to fly.
 */
LOG_ADD_CORE(LOG_UINT8, canfly, &canFly)
/**
 * @brief Nonzero if the system thinks it is flying
 */
LOG_ADD_CORE(LOG_UINT8, isFlying, &isFlying)
/**
 * @brief Nonzero if the system thinks it is tumbled/crashed
 */
LOG_ADD_CORE(LOG_UINT8, isTumbled, &isTumbled)
LOG_GROUP_STOP(sys)


// TODO Krri rename? deprecate?
PARAM_GROUP_START(stabilizer)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &areMotorsLocked)
PARAM_GROUP_STOP(stabilizer)
