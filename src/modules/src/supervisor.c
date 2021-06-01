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
#include "motors.h"
#include "pm.h"
#include "stabilizer.h"
#include "supervisor.h"

/* Minimum summed motor PWM that means we are flying */
#define SUPERVISOR_FLIGHT_THRESHOLD 1000

/* Number of times in a row we need to see a condition before acting upon it */
#define SUPERVISOR_HYSTERESIS_THRESHOLD 30

static bool canFly;
static bool isFlying;
static bool isTumbled;

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
// We cannot fly if the Crazyflie is tumbled and we cannot fly if the Crazyflie
// is connected to a charger.
//
static bool canFlyCheck()
{
  if (isTumbled) {
    return false;
  }
  return !pmIsChargerConnected();
}

//
// We say we are flying if the sum of the ratio of all motors are above
// a certain threshold.
//
static bool isFlyingCheck()
{
  int sumRatio = 0;
  for (int i = 0; i < NBR_OF_MOTORS; ++i) {
    sumRatio += motorsGetRatio(i);
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

void supervisorUpdate(const sensorData_t *data)
{
  isFlying = isFlyingCheck();

  isTumbled = isTumbledCheck(data);
  if (isTumbled && isFlying) {
    stabilizerSetEmergencyStop();
  }

  canFly = canFlyCheck();
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
