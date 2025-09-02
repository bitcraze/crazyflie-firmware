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
 * deck_backend_onewire.c - OneWire deck discovery backend
 */

#include "deck_discovery.h"
#include "deck.h"
#include "ow.h"

#define DEBUG_MODULE "DECK_BACKEND_OW"
#include "debug.h"

// Uncomment to enable debug prints for OneWire backend
// #define DEBUG_ONEWIRE_BACKEND

#ifdef DEBUG_ONEWIRE_BACKEND
#define OW_BACKEND_DEBUG(fmt, ...) DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
#define OW_BACKEND_DEBUG(...)
#endif

// OneWire backend state
static uint8_t totalDecks = 0;
static uint8_t currentDeck = 0;
static DeckInfo deckBuffer; // Buffer for returning deck info

// Dummy driver for decks that do not have a driver implemented
extern const DeckDriver dummyDriver;

static bool owBackendInit(void) {
    OW_BACKEND_DEBUG("Initializing OneWire backend\n");
    currentDeck = 0;
    totalDecks = 0;

    // Always initialize OneWire (needed for memory system)
    owInit();

#ifndef CONFIG_DEBUG_DECK_IGNORE_OWS
    if (!owScan(&totalDecks)) {
        OW_BACKEND_DEBUG("OneWire scan failed\n");
        return false;
    }

    OW_BACKEND_DEBUG("OneWire found %d deck(s)\n", totalDecks);
#else
    OW_BACKEND_DEBUG("Ignoring all OW decks because of compile flag.\n");
    totalDecks = 0;
#endif
    return true;
}

static DeckInfo* owBackendGetNextDeck(void) {
    if (currentDeck >= totalDecks) {
        return NULL; // No more decks
    }

#ifdef CONFIG_DEBUG_DECK_IGNORE_OWS
    // Skip all decks when ignore flag is set
    currentDeck = totalDecks; // Move to end
    return NULL;
#endif

    OW_BACKEND_DEBUG("Reading OneWire deck %d\n", currentDeck);

    // Read raw deck info
    if (!owRead(currentDeck, 0, sizeof(deckBuffer.raw), (uint8_t*)&deckBuffer)) {
        OW_BACKEND_DEBUG("Failed to read OneWire deck %d\n", currentDeck);
        currentDeck++;
        return NULL;
    }

    // Store backend reference
    deckBuffer.discoveryBackend = NULL; // Backend reference will be set after copying in enumeration code

    // Decode and validate deck info using shared function
    if (infoDecode(&deckBuffer)) {
        deckBuffer.driver = findDriver(&deckBuffer);
        printDeckInfo(&deckBuffer);
    } else {
#ifdef CONFIG_DEBUG
        OW_BACKEND_DEBUG("OneWire deck %d has corrupt memory. Using dummy driver in DEBUG mode.\n", currentDeck);
        deckBuffer.driver = &dummyDriver;
#else
        OW_BACKEND_DEBUG("OneWire deck %d has corrupt memory. Skipping.\n", currentDeck);
        currentDeck++;
        return NULL;
#endif
    }

    currentDeck++;
    return &deckBuffer;
}

static const DeckDiscoveryBackend_t owBackend = {
    .name = "onewire",
    .init = owBackendInit,
    .getNextDeck = owBackendGetNextDeck,
};

DECK_DISCOVERY_BACKEND(owBackend);