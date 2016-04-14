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

  // The watchdog uses the LSI oscillator for checking the timeout. The LSI
  // oscillator frequency is not very exact and the range is fairly large
  // and also differs between the Crazyflie 1.0 and 2.0:
  //                 MIN  TYP  MAX
  // Crazyflie 1.0   30   40   60   (in kHz)
  // Crazyflie 2.0   17   32   47   (in kHz)
  //
  IWDG_SetPrescaler(IWDG_Prescaler_32);
  // Divide the clock with 32 which gives
  // an interval of 17kHz/32 (CF2 min) to 60kHz/32 (CF1 max) =>
  // 1875 Hz to 531 Hz for the watchdog timer.
  //
  // The goal timeout is >100 ms, but it's acceptable
  // that the max timeout is a bit higher. Scaling the
  // reload counter for the fastest LSI then gives a
  // timeout of 100ms, which in turn gives a timeout
  // of 353ms for the slowest LSI. So the watchdog timeout
  // will be between 100ms and 353ms on all platforms.
  //
  // At prescaler 32 each bit is 1 ms this gives:
  // 1875 Hz * 0.1 s / 1 => 188
  IWDG_SetReload(188);

  watchdogReset();
  IWDG_Enable();
}
