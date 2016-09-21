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

#ifndef _EXT_POSITION_H_
#define _EXT_POSITION_H_

#include "stabilizer_types.h"

/**
 * CRTP external position data struct
 */
struct CrtpExtPosition
{
  float x; // in m
  float y; // in m
  float z; // in m
} __attribute__((packed));

// Set up the callback for the CRTP_PORT_POSITION
void extPositionInit(void);

// Get the current position from the cache
bool getExtPosition(state_t *state);

#endif /* _EXT_POSITION_H_ */
