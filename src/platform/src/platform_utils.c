/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 * Platform utilities
 *
 */

#define DEBUG_MODULE "PLATFORM"

#include "platform.h"
#include "radiolink.h"
#include "debug.h"

// Define to decrease the nRF51 Tx power to reduce interference
#ifndef PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM
#define PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM (-12)
#endif


void platformSetLowInterferenceRadioMode(void) {
 // Decrease the nRF51 Tx power to reduce interference
 radiolinkSetPowerDbm(PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM);
 DEBUG_PRINT("Low interference mode. NRF51 TX power offset by %ddb.\r\n", PLATFORM_NRF51_LOW_INTERFERENCE_TX_POWER_DBM);
}

