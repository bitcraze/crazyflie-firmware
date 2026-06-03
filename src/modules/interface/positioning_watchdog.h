/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * positioning_watchdog.h - Liveness monitoring of positioning decks
 */

#pragma once

#include <stdbool.h>

// A positioning source (a deck) that can be monitored for liveness. The deck
// registers itself when it has been detected and provides a function that
// reports whether it is currently delivering data.
typedef struct positioningSource_s {
  const char* name;
  bool (*isAlive)(void);
} positioningSource_t;

/**
 * @brief Register a positioning source to be monitored. Should be called by a
 * deck driver once the deck has been detected.
 *
 * @param source Pointer to a source description with static lifetime
 */
void positioningWatchdogRegister(const positioningSource_t* source);

/**
 * @brief Query if any registered positioning source has stopped delivering data
 *
 * @return true  At least one source is faulty (disconnected/malfunctioning)
 * @return false All registered sources are alive (or none are registered)
 */
bool positioningWatchdogHasFault(void);
