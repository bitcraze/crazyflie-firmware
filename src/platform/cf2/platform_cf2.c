/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
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
 * TODO: Add description
 */

#define DEBUG_MODULE "PLATFORM"

/* Personal configs */
/* Project includes */
#include "exti.h"
#include "nvic.h"
#include "debug.h"
#include "radiolink.h"

// Define to decrease the nRF51 Tx power to reduce interference
#define PLATFORM_NRF51_TX_POWER_DBM (-12)

// TODO: Implement!
int platformInit(void)
{
  //Low level init: Clock and Interrupt controller
  nvicInit();

  //EXTI interrupts
  extiInit();

  return 0;
}

void platformSetLowInterferenceRadioMode(void)
{
  // Decrease the nRF51 Tx power to reduce interference
  radiolinkSetPowerDbm(PLATFORM_NRF51_TX_POWER_DBM);
  DEBUG_PRINT("Low interference mode. NRF51 TX power reduced by %ddb.\r\n", PLATFORM_NRF51_TX_POWER_DBM);
}
