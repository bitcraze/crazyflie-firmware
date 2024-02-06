/**
 *    ||          ____  _ __  ______
 * +------+      / __ )(_) /_/ ____/_________ _____  ___
 * | 0xBC |     / __  / / __/ /    / ___/ __ `/_  / / _	\
 * +------+    / /_/ / / /_/ /___ / /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\____//_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 */

#ifndef COMMANDER_H_
#define COMMANDER_H_
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "stabilizer_types.h"

#define DEFAULT_YAW_MODE  XMODE

#define COMMANDER_PRIORITY_DISABLE   0
// Keep a macro for lowest non-disabled priority, regardless of source, in case
// some day there is a priority lower than the high-level commander.
#define COMMANDER_PRIORITY_LOWEST    1
#define COMMANDER_PRIORITY_HIGHLEVEL 1
#define COMMANDER_PRIORITY_CRTP      2
#define COMMANDER_PRIORITY_EXTRX     3

void commanderInit(void);
bool commanderTest(void);
uint32_t commanderGetInactivityTime(void);

// Arg `setpoint` cannot be const; the commander will mutate its timestamp.
void commanderSetSetpoint(setpoint_t *setpoint, int priority);
int commanderGetActivePriority(void);

// Sets the priority of the current setpoint to the lowest non-disabled value,
// so any new setpoint regardless of source will overwrite it.
void commanderRelaxPriority(void);

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);

#endif /* COMMANDER_H_ */
