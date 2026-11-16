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
 * platform.h - Shadow header for the Simmyflie.
 *
 * The mainline platform.h includes motors.h and describes the hardware
 * configurations. This header keeps the part of its contract that the
 * compiled files use, implemented by src/sim/src/platform/platform.c.
 * platformConfigGetMotorMapping() returns `const void **` instead of
 * `const MotorPerifDef **`, matching motorsInit() in the shadow motors.h.
 *
 * See docs/development/simmyflie/index.md for the Shadow header rules.
 */
#ifndef PLATFORM_H_
#define PLATFORM_H_

#include <stdbool.h>

int platformInit(void);
const char* platformConfigGetDeviceTypeName(void);
const void** platformConfigGetMotorMapping(void);
bool platformConfigPhysicalLayoutAntennasAreClose(void);
void platformSetLowInterferenceRadioMode(void);

#endif // PLATFORM_H_
