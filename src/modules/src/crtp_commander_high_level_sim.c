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
 * crtp_commander_high_level_sim.c - Minimal crtp_commander_high_level.h
 * backend for CONFIG_PLATFORM_SIM (Simmyflie), Phase 4.8.
 *
 * EXPLICITLY TEMPORARY (see dev/implementation-plan-phase-4.md's 4.8
 * section -- user decision, same reasoning as commander_sim.c): the real
 * crtp_commander_high_level.c needs planner.c, which #includes arm_math.h
 * (CMSIS DSP) -- that pulls in ARM inline assembly (cmsis_gcc.h) that
 * doesn't compile under x86-64 GCC, a new build blocker unrelated to
 * anything else in this phase. It also self-registers CRTP port 8
 * (Setpoint HL) in its own Init, which is 4.9's scope, not 4.8's. This stub
 * supplies just the two entry points stabilizer.c's loop calls:
 * crtpCommanderHighLevelGetSetpoint() always returns false (never touches
 * its output setpoint -- matches "no active high-level trajectory"), and
 * crtpCommanderBlock() is a no-op. Safe because supervisor_sim.c's
 * supervisorCanFly() is unconditionally false in this phase, so
 * stabilizerTask() short-circuits past crtpCommanderHighLevelGetSetpoint()
 * entirely (`canFly && crtpCommanderHighLevelGetSetpoint(...)`) -- this
 * stub exists to satisfy the link, not to be exercised. Phase 4.9 replaces
 * this file wholesale with the real crtp_commander_high_level.c (+
 * planner.c/pptraj*.c, once the CMSIS problem is solved), not by extending
 * it.
 */

#include "crtp_commander_high_level.h"

bool crtpCommanderHighLevelGetSetpoint(setpoint_t* setpoint, const state_t *state, stabilizerStep_t stabilizerStep)
{
  (void)setpoint;
  (void)state;
  (void)stabilizerStep;
  return false;
}

int crtpCommanderBlock(bool doBlock)
{
  (void)doBlock;
  return 0;
}
