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
 * watchdog_sim.c - watchdogNormalStartTest() backend for
 * CONFIG_PLATFORM_SIM (Simmyflie), Phase 4.0.
 *
 * Deliberately does not include drivers/interface/watchdog.h: that header
 * includes stm32fxxx.h directly (IWDG_* register access), an STM32
 * hardware chain with no meaning off-target -- same reasoning as
 * platform_sim.c bypassing platform.h. The signature reproduced below is
 * otherwise identical (bool, no arguments), so this links against the real
 * declaration wherever one is visible.
 *
 * watchdogInit()/watchdogReset() (the IWDG_* hardware watchdog itself) are
 * out of scope for this chunk -- nothing in Phase 4.0's own systemLaunch()
 * calls them yet; only system.c's systemStart()/vApplicationIdleHook() do,
 * and that's Phase 4.10's cutover to worry about.
 */

#include <stdbool.h>

bool watchdogNormalStartTest(void)
{
  return true;
}
