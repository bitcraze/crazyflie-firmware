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
 * (Simmyflie), Phase 4.2 (extended Phase 4.8).
 *
 * The real supervisor.c pulls in log.c/param.c/power_distribution.c/
 * planner.c/crtp_commander_high_level.c -- planner.c/crtp_commander_high_level.c
 * still don't exist as of Phase 4.8 (planner.c needs arm_math.h/CMSIS, which
 * doesn't build under a 64-bit host; crtp_commander_high_level.c is 4.9's
 * scope), and real supervisor.c has its own new hardware chain besides
 * (deck_supervisor.h). platformservice.c's deprecated armSystem/
 * recoverSystem commands were the only sim callers of supervisor.h through
 * 4.2-4.7; Phase 4.8's stabilizer.c adds four more (supervisorCanFly/
 * supervisorUpdate/supervisorOverrideSetpoint/
 * supervisorAreMotorsAllowedToRun), stubbed here the same way, all inert
 * (user decision, see dev/implementation-plan-phase-4.md's 4.8 section --
 * chosen over pulling the real commander/crtp_commander_high_level/
 * planner/supervisor chain forward into 4.8). supervisorCanFly() always
 * false keeps this consistent with the pre-existing arming gap: nothing in
 * sim can fly until 4.9 gives arming real supervisor/commander context, so
 * stabilizerTask()'s motor-output branch stays dead code, matching 4.2's
 * "Known gap" exactly. Phase 4.9's plan already carries this gap forward --
 * swap this file out for the real one then, don't extend it further.
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

void supervisorUpdate(const sensorData_t *sensors, const setpoint_t* setpoint, stabilizerStep_t stabilizerStep)
{
  (void)sensors;
  (void)setpoint;
  (void)stabilizerStep;
}

void supervisorOverrideSetpoint(setpoint_t* setpoint)
{
  (void)setpoint;
}

bool supervisorAreMotorsAllowedToRun()
{
  return false;
}

bool supervisorCanFly(void)
{
  return false;
}
