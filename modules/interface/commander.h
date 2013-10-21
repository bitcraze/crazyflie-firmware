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

#define COMMANDER_WDT_TIMEOUT_STABALIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

typedef enum
{
  RATE,
  ANGLE
} RPYType;

void commanderInit(void);
bool commanderTest(void);
void commanderWatchdog(void);
uint32_t commanderGetInactivityTime(void);
void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired);
void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType);
void commanderGetThrust(uint16_t* thrust);
void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange);

#endif /* COMMANDER_H_ */
