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
 * supervisor_sim.c - Minimal supervisor.h backend for CONFIG_PLATFORM_SIM
 * (Simmyflie), Phase 4.2.
 *
 * The real supervisor.c pulls in log.c/param.c/power_distribution.c/
 * planner.c/crtp_commander_high_level.c -- none of which exist yet at this
 * point in Phase 4 (see dev/implementation-plan-phase-4.md, chunk 4.2).
 * platformservice.c's deprecated armSystem/recoverSystem commands are the
 * only sim callers of supervisor.h right now, so this stub supplies just
 * those four entry points, inert (arming/recovery always denied). Phase
 * 4.2's plan explicitly carries this gap forward to 4.9, once Commander
 * lands and the real supervisor.c's dependents all exist -- swap this file
 * out for the real one then, don't extend it.
 */

#include "supervisor.h"

bool supervisorRequestArming(const bool doArm)
{
  (void)doArm;
  return false;
}

bool supervisorIsArmed(void)
{
  return false;
}

bool supervisorRequestCrashRecovery(const bool doRecover)
{
  (void)doRecover;
  return false;
}

bool supervisorIsCrashed(void)
{
  return false;
}
