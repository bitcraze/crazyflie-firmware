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
 * pm_sim.c - pm.h backend for CONFIG_PLATFORM_SIM (Simmyflie), Phase 4.6.
 *
 * The real pm_stm32f4.c drives a battery state machine off syslink messages
 * from the NRF companion chip (charge state, low-battery shutdown, LED/sound
 * feedback) and pulls in led.h/ledseq.h/commander.h/sound.h/deck.h to do it
 * -- deck.h alone drops into the STM32 hardware header chain via
 * deck_constants.h -> stm32fxxx.h, so pm.h can't be #include'd here any more
 * than platform.h could in platform_sim.c (see that file's header comment
 * for the same reasoning). None of that state machine has meaning under the
 * MVP's constant-voltage, infinite-battery requirement, so this is a full
 * HAL swap, not a narrowed real pm.c: just pmInit() and the pm.vbat log
 * variable real hardware exposes, fixed at 4.2 V forever, no drain, no
 * charge/low/shutdown states. Nothing in the sim build calls any other
 * pm.h entry point yet (deck drivers/motors.c/stabilizer.c/health.c/info.c
 * all reference pmGetBatteryVoltage() and friends, but none of those
 * translation units are built under PLATFORM_SIM until Phase 5+) -- add the
 * rest of pm.h's surface here if and when a real sim caller needs it,
 * rather than guessing at it now.
 */

#include "log.h"

static float batteryVoltage = 4.2f;

void pmInit(void)
{
}

/**
 * Power management log variables.
 */
LOG_GROUP_START(pm)
/**
 * @brief Battery voltage [V] -- fixed at 4.2, Simmyflie has no battery drain.
 */
LOG_ADD_CORE(LOG_FLOAT, vbat, &batteryVoltage)
LOG_GROUP_STOP(pm)
