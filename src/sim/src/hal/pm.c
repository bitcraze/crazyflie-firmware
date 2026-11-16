/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2026 Bitcraze AB
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
 * pm.c - Replacement of pm.h for the Simmyflie.
 *
 * The sim has an ideal battery: the voltage is a constant 4.2 V and it never
 * charges, runs low or shuts down. The mainline power management state
 * machine, driven by syslink messages from the nRF51, has nothing to do. The
 * pm.vbat and pm.state log variables are kept, with the values a Crazyflie
 * running on a full battery reports.
 */

#include <stdint.h>

#include "log.h"
#include "pm.h"

static float batteryVoltage = 4.2f;
/* PMStates value 0 (battery); the shadow pm.h does not declare the enum.
 * int8_t rather than an enum: the real pmState is 1 byte on ARM
 * (-fshort-enums), a host enum would be 4. */
static int8_t pmState = 0;

void pmInit(void)
{
}

bool pmTest(void)
{
  return true;
}

float pmGetBatteryVoltage(void)
{
  return batteryVoltage;
}

/**
 * Power management log variables.
 */
LOG_GROUP_START(pm)
/**
 * @brief Battery voltage [V] -- fixed at 4.2, Simmyflie has no battery drain.
 */
LOG_ADD_CORE(LOG_FLOAT, vbat, &batteryVoltage)
/**
 * @brief State of power management -- fixed at 0 (Battery), Simmyflie
 * never charges, runs low or shuts down. Same encoding as real hardware:
 *
 * | State | Meaning   | \n
 * | -     | -         | \n
 * | 0     | Battery   | \n
 * | 1     | Charging  | \n
 * | 2     | Charged   | \n
 * | 3     | Low power | \n
 * | 4     | Shutdown  | \n
 */
LOG_ADD_CORE(LOG_INT8, state, &pmState)
LOG_GROUP_STOP(pm)
