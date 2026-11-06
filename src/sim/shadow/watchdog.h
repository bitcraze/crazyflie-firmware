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
 * watchdog.h - Shadow header for the Simmyflie.
 *
 * The mainline watchdog.h defines watchdogReset() as a write to the IWDG
 * register. Here it is a function, in src/sim/src/drivers/watchdog.c, like
 * watchdogInit() and watchdogNormalStartTest().
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <stdbool.h>

#define WATCHDOG_RESET_PERIOD_MS 80

void watchdogInit(void);
bool watchdogNormalStartTest(void);
void watchdogReset(void);

#endif // WATCHDOG_H_
