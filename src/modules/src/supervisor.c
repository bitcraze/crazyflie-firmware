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

#include <stdbool.h>
#include <stdlib.h>

#include "log.h"
#include "motors.h"
#include "pm.h"
#include "supervisor.h"

/* Minimum summed motor PWM that means we are flying */
#define SUPERVISOR_FLIGHT_THRESHOLD 1000

static bool canFly;
static bool isFlying;

bool supervisorCanFly()
{
  return canFly;
}

bool supervisorIsFlying()
{
  return isFlying;
}

//
// We cannot fly if the Crazyflie is tumbled and we cannot fly if the Crazyflie
// is connected to a charger.
//
static bool canFlyCheck()
{
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

void supervisorUpdate()
{
  isFlying = isFlyingCheck();
  canFly = canFlyCheck();
}

LOG_GROUP_START(sys)
LOG_ADD(LOG_UINT8, canfly, &canFly)
LOG_ADD(LOG_UINT8, isFlying, &isFlying)
LOG_GROUP_STOP(sys)
