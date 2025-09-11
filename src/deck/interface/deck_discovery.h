/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2025 Bitcraze AB
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
 * deck_discovery.h - Generic deck discovery backend interface
 */

#ifndef __DECK_DISCOVERY_H__
#define __DECK_DISCOVERY_H__

#include <stdint.h>
#include <stdbool.h>

// Forward declarations
typedef struct deckInfo_s DeckInfo;
typedef struct deck_driver DeckDriver;

/**
 * @brief Deck discovery backend interface
 */
typedef struct deckDiscoveryBackend_s {
    const char* name;                    ///< Backend name for debugging
    bool (*init)(void);                  ///< Initialize discovery hardware
    DeckInfo* (*getNextDeck)(void);      ///< Returns next deck or NULL when done
} DeckDiscoveryBackend_t;

/**
 * @brief Shared functions available to all backends
 */
const DeckDriver* findDriver(DeckInfo *deck);    ///< Find driver for deck
void printDeckInfo(DeckInfo *info);              ///< Print deck debug info

/**
 * @brief Dummy driver for decks without a driver
 */
extern const DeckDriver dummyDriver;

/**
 * @brief Registration macro for discovery backends  
 */
#define DECK_DISCOVERY_BACKEND(NAME) \
    const DeckDiscoveryBackend_t * backend_##NAME \
    __attribute__((section(".deckBackend." #NAME), used)) = &(NAME)

/**
 * @brief Access registered backends
 */
int deckDiscoveryBackendCount(void);                   ///< Get number of backends
const DeckDiscoveryBackend_t* deckDiscoveryGetBackend(int i);  ///< Get backend by index

#endif //__DECK_DISCOVERY_H__