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
 * pm.h - Shadow header for the Simmyflie.
 *
 * The mainline pm.h includes deck.h and syslink.h, and through them the STM32
 * device headers. This header keeps the part of its contract that the
 * compiled files use, implemented by src/sim/src/hal/pm.c. It includes adc.h
 * as mainline does; system.c gets adcInit() that way.
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef PM_H_
#define PM_H_

#include <stdbool.h>

/* The real pm.h includes adc.h, and system.c gets adcInit() that way. */
#include "adc.h"

void pmInit(void);
bool pmTest(void);
float pmGetBatteryVoltage(void);

#endif // PM_H_
