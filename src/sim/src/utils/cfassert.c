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
 * cfassert_sim.c - ASSERT()/assertFail() backend for CONFIG_PLATFORM_SIM.
 *
 * The real cfassert.c (led.h/motors.h-driven blink-and-halt) is hardware
 * code. This is what crtp.c's ASSERT() macro links against instead, so
 * Communication (Phase 3) doesn't have to pull in the LED/motor HAL just to
 * satisfy the assert macro.
 */

#include "cfassert.h"

#include <stdio.h>
#include <stdlib.h>

void assertFail(char *exp, char *file, int line)
{
  fprintf(stderr, "Simmyflie: ASSERT failed: %s (%s:%d)\n", exp, file, line);
  fflush(stderr);
  abort();
}

void printAssertSnapshotData(void)
{
}

void storeAssertFileData(const char *file, int line)
{
  (void)file;
  (void)line;
}

void storeAssertHardfaultData(
    unsigned int r0,
    unsigned int r1,
    unsigned int r2,
    unsigned int r3,
    unsigned int r12,
    unsigned int lr,
    unsigned int pc,
    unsigned int psr)
{
  (void)r0; (void)r1; (void)r2; (void)r3;
  (void)r12; (void)lr; (void)pc; (void)psr;
}

void storeAssertTextData(const char *text)
{
  (void)text;
}

bool cfAssertNormalStartTest(void)
{
  return true;
}
