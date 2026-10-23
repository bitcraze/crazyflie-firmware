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
 * platform.h (hw_shims) - Phase 4.8 shadow for src/platform/interface/
 * platform.h, CONFIG_PLATFORM_SIM only (Simmyflie).
 *
 * See motors.h in this same directory for the full rationale -- this file
 * is the platform.h counterpart, needed because stabilizer.c #include
 * "platform.h" unconditionally, and that header pulls in motors.h ->
 * stm32fxxx.h directly.
 *
 * Not a declaration set of its own -- platform_sim.h remains the single
 * source of truth (also included directly by main_sim.c/platformservice.c
 * and defined against by platform_sim.c).
 */
#ifndef __PLATFORM_HW_SHIM_H__
#define __PLATFORM_HW_SHIM_H__

#include "platform_sim.h"

#endif // __PLATFORM_HW_SHIM_H__
