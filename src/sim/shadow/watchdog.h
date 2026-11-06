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
 * watchdog.h (sim shim) - shadows src/drivers/interface/watchdog.h for
 * CONFIG_PLATFORM_SIM (Simmyflie), which includes stm32fxxx.h and maps
 * watchdogReset() straight onto the IWDG register. Same include-path
 * shadowing as motors.h/pm.h/platform.h in this directory. The functions are
 * in watchdog_sim.c.
 */
#ifndef __WATCHDOG_HW_SHIM_H__
#define __WATCHDOG_HW_SHIM_H__

#include <stdbool.h>

#define WATCHDOG_RESET_PERIOD_MS 80

void watchdogInit(void);
bool watchdogNormalStartTest(void);
void watchdogReset(void);

#endif // __WATCHDOG_HW_SHIM_H__
