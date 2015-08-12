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

#ifdef PLATFORM_CF1
  #define DEFUALT_YAW_MODE  PLUSMODE
#else
  #define DEFUALT_YAW_MODE  XMODE
#endif

#define COMMANDER_WDT_TIMEOUT_STABALIZE  M2T(500)
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   M2T(2000)

/**
 * Stabilization modes for Roll, Pitch, Yaw.
 */
typedef enum
{
  RATE    = 0,
  ANGLE   = 1,
} RPYType;

/**
 * Yaw flight Modes
 */
typedef enum
{
  CAREFREE  = 0, // Yaw is locked to world coordinates thus heading stays the same when yaw rotates
  PLUSMODE  = 1, // Plus-mode. Motor M1 is defined as front
  XMODE     = 2, // X-mode. M1 & M4 is defined as front
} YawModeType;

void commanderInit(void);
bool commanderTest(void);
void commanderWatchdog(void);
uint32_t commanderGetInactivityTime(void);
void commanderGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired);
void commanderGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType);
void commanderGetThrust(uint16_t* thrust);
void commanderGetAltHold(bool* altHold, bool* setAltHold, float* altHoldChange);
bool commanderGetAltHoldMode(void);
void commanderSetAltHoldMode(bool altHoldModeNew);
YawModeType commanderGetYawMode(void);
bool commanderGetYawModeCarefreeResetFront(void);

#endif /* COMMANDER_H_ */
