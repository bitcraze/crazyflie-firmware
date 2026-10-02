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
 * watchdog_sim.c - watchdog.h backend for CONFIG_PLATFORM_SIM (Simmyflie).
 *
 * There is no hardware watchdog in the sim: the start is always a normal
 * one, and watchdogInit()/watchdogReset() do nothing. The declarations are
 * in src/config/sim/hw_shims/watchdog.h, which shadows the real watchdog.h
 * (that one includes stm32fxxx.h).
 */

#include "watchdog.h"

void watchdogInit(void)
{
}

bool watchdogNormalStartTest(void)
{
  return true;
}

void watchdogReset(void)
{
}
