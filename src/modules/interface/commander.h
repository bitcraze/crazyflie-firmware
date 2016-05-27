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

#ifdef PLATFORM_CF1
  #define DEFAULT_YAW_MODE  PLUSMODE
#else
  #define DEFAULT_YAW_MODE  XMODE
#endif

#define COMMANDER_WDT_TIMEOUT_STABILIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

/**
 * CRTP commander data struct
 */
struct CommanderCrtpValues
{
  float roll;       // deg
  float pitch;      // deg
  float yaw;        // deg
  uint16_t thrust;
} __attribute__((packed));

void commanderInit(void);
bool commanderTest(void);
uint32_t commanderGetInactivityTime(void);
void commanderExtrxSet(const struct CommanderCrtpValues* val);

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state);

#endif /* COMMANDER_H_ */
