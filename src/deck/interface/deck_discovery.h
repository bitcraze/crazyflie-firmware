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
 *
 * This interface defines how deck discovery backends work. Each backend
 * is responsible for discovering decks using a specific protocol or mechanism
 * (e.g., OneWire memory, I2C enumeration, compile-time forcing).
 *
 * Backends are registered using the DECK_DISCOVERY_BACKEND() macro and are
 * called sequentially during system initialization. Each backend's init()
 * function is called once, followed by repeated calls to getNextDeck() until
 * it returns NULL to indicate no more decks are available.
 *
 * Multiple backends can coexist, allowing discovery of different deck types
 * (e.g., legacy OneWire decks and new DeckCtrl decks) in a single system.
 */
typedef struct deckDiscoveryBackend_s {
    const char* name;                    ///< Backend name for debugging/logging
    bool (*init)(void);                  ///< Initialize discovery hardware (called once at startup)
    DeckInfo* (*getNextDeck)(void);      ///< Returns next deck info or NULL when done (called repeatedly)
} DeckDiscoveryBackend_t;

/**
 * @brief Shared functions available to all backends
 *
 * These functions are implemented by the deck core system and can be used
 * by any discovery backend during the enumeration process.
 */
const DeckDriver* findDriver(DeckInfo *deck);    ///< Find matching driver for a discovered deck (by VID/PID)
void printDeckInfo(DeckInfo *info);              ///< Print deck information for debugging

/**
 * @brief Dummy driver for decks without a driver
 *
 * This driver is used for decks that are detected but have no corresponding
 * driver implementation, or when operating in DEBUG mode with corrupt deck data.
 */
extern const DeckDriver dummyDriver;

/**
 * @brief Registration macro for discovery backends
 *
 * Use this macro to register a discovery backend with the system. The backend
 * will be automatically discovered at compile time using linker sections and
 * called during deck enumeration.
 *
 * Example usage:
 * @code
 * static const DeckDiscoveryBackend_t myBackend = {
 *     .name = "mybackend",
 *     .init = myBackendInit,
 *     .getNextDeck = myBackendGetNextDeck,
 * };
 * DECK_DISCOVERY_BACKEND(myBackend);
 * @endcode
 */
#define DECK_DISCOVERY_BACKEND(NAME) \
    const DeckDiscoveryBackend_t * backend_##NAME \
    __attribute__((section(".deckBackend." #NAME), used)) = &(NAME)

/**
 * @brief Access registered backends
 *
 * These functions allow querying the list of compiled-in discovery backends.
 * Used by the deck enumeration system to iterate through all available backends.
 */
int deckDiscoveryBackendCount(void);                          ///< Get total number of registered backends
const DeckDiscoveryBackend_t* deckDiscoveryGetBackend(int i); ///< Get backend by index (0 to count-1)

#endif //__DECK_DISCOVERY_H__