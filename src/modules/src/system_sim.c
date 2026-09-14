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
 * system_sim.c - Minimal system.h backend for CONFIG_PLATFORM_SIM
 * (Simmyflie), Phase 4.8.
 *
 * EXPLICITLY TEMPORARY (see dev/implementation-plan-phase-4.md's 4.8
 * section): stabilizer.c's stabilizerTask() calls systemWaitStart()
 * unconditionally before its main loop -- on real hardware this blocks
 * until system.c's self-test-gated boot sequence (systemTask()) signals
 * startup is complete. That whole sequence is Phase 4.10's cutover
 * (main_sim.c still runs its own local systemLaunch() placeholder, not the
 * real system.c). Returning immediately is the correct sim behavior for
 * now, not just an expedient stub: there is no self-test gate yet for it to
 * legitimately wait on. Phase 4.10 replaces this file wholesale with the
 * real system.c, not by extending it.
 */

#include "system.h"

void systemWaitStart(void)
{
}
