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
 * motors.h (hw_shims) - Phase 4.8 shadow for src/drivers/interface/
 * motors.h, CONFIG_PLATFORM_SIM only (Simmyflie).
 *
 * stabilizer.c and health.c #include "motors.h" unconditionally -- code
 * this project doesn't control the includes of, unlike every _sim.c file so
 * far (see design-specification.md's "Header-shadow-avoidance" section for
 * the full rationale and the alternative considered). This directory is
 * placed ahead of src/drivers/interface on the sim build's -I search path
 * (see Makefile), so their #include "motors.h" resolves to this file
 * instead of the real one, which pulls in stm32fxxx.h.
 *
 * Not a declaration set of its own -- motors_sim.h remains the single
 * source of truth (also included directly by main_sim.c and defined
 * against by motors_sim.c).
 *
 * Also pulls in FreeRTOS.h/task.h/semphr.h/config.h: real motors.h includes
 * config.h directly (task names/priorities/stack sizes), which in turn
 * includes usec_time.h -- stabilizer.c's usecTimestamp() call relies on
 * that transitive chain rather than including either directly. The
 * FreeRTOS.h/task.h/semphr.h trio is the same story for
 * xSemaphoreGive()/vTaskDelay()/M2T() (confirmed empirically -- all compile
 * unmodified on the real ARM build; the exact real header responsible for
 * each was not traced beyond config.h, and does not need to be, since the
 * shim only needs to match the net effect). Not needed by
 * motors_sim.h/motors_sim.c themselves.
 */
#ifndef __MOTORS_HW_SHIM_H__
#define __MOTORS_HW_SHIM_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "config.h"

#include "motors_sim.h"

#endif // __MOTORS_HW_SHIM_H__
