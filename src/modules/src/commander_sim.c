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
 * commander_sim.c - Minimal commander.h backend for CONFIG_PLATFORM_SIM
 * (Simmyflie), Phase 4.8.
 *
 * EXPLICITLY TEMPORARY (see dev/implementation-plan-phase-4.md's 4.8
 * section -- user decision): stabilizer.c calls commanderGetSetpoint()/
 * commanderSetSetpoint() unconditionally, but the real commander.c (queue-
 * based, priority-arbitrated setpoint source) is Phase 4.9's scope, which
 * also wires up its CRTP port (3). Rather than pull that chain forward,
 * this stub supplies just the two entry points stabilizer.c's loop needs,
 * always producing/accepting a zeroed setpoint -- safe because
 * supervisor_sim.c's supervisorCanFly() is unconditionally false in this
 * phase, so stabilizerTask() never executes the branch that would act on a
 * real commanded setpoint anyway. Phase 4.9 replaces this file wholesale
 * with the real commander.c, not by extending it.
 */

#include <string.h>

#include "commander.h"

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  (void)state;
  memset(setpoint, 0, sizeof(setpoint_t));
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  (void)setpoint;
  (void)priority;
}
