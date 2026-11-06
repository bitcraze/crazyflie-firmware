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
 * led_sim.c - led.h backend for CONFIG_PLATFORM_SIM (Simmyflie), Phase 4.0.
 *
 * The real led.c drives GPIO registers directly. There's no simulated
 * meaning for individual LED state yet (no client-visible behavior in this
 * chunk), so this just no-ops the whole surface -- enough for ledseq_sim.c
 * and system.c's own ledSet(CHG_LED, 1) call to link and run.
 */

#include "led.h"

void ledInit()
{
}

bool ledTest()
{
  return true;
}

void ledClearAll(void)
{
}

void ledSetAll(void)
{
}

void ledSet(led_t led, bool value)
{
  (void)led;
  (void)value;
}

void ledShowFaultPattern(void)
{
}
