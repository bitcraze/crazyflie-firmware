/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) Bitcraze AB
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
 * @file watchdog.c - Implementaion of hardware watchdog
 *
 */
#define DEBUG_MODULE "SYS"

#include "debug.h"

#include "watchdog.h"
#include "cfassert.h"


bool watchdogNormalStartTest(void)
{
  bool wasNormalStart = true;

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST)) {
		RCC_ClearFlag();
		wasNormalStart = false;
		DEBUG_PRINT("The system resumed after watchdog timeout [WARNING]\n");
		printAssertSnapshotData();
	}

	return wasNormalStart;
}


void watchdogInit(void)
{
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  // IWDG counter clock: LSI/32 = 1024Hz
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  IWDG_SetReload(WATCHDOG_TIMEOUT_CYCLES);

  watchdogReset();
	IWDG_Enable();
}
