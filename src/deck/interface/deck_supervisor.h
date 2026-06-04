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
 * deck_supervisor.h - Monitor deck hardware health via the deck isAlive probe
 */

#pragma once

#include <stdbool.h>

/**
 * @brief Query if any installed deck reports a hardware fault.
 *
 * Iterates the detected decks and calls the isAlive probe of each deck that
 * provides one. A deck that has not initialized correctly reports not alive.
 *
 * @return true  At least one deck is not alive
 * @return false All probed decks are alive (or none provide a probe)
 */
bool deckSupervisorHasFault(void);
