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
 * deck_discovery.c - Generic deck discovery backend management
 */

#include "deck_discovery.h"
#include <stddef.h>

#define DEBUG_MODULE "DECK_DISCOVERY"
#include "debug.h"

// Uncomment to enable debug prints for deck discovery
// #define DEBUG_DECK_DISCOVERY

#ifdef DEBUG_DECK_DISCOVERY
#define DECK_DISCOVERY_DEBUG(fmt, ...) DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
#define DECK_DISCOVERY_DEBUG(...)
#endif

// Linker symbols (need to be added to linker script)
extern const DeckDiscoveryBackend_t * _deckBackend_start;
extern const DeckDiscoveryBackend_t * _deckBackend_stop;

static const DeckDiscoveryBackend_t ** backends;
static int backendsLen;

// Initialize backend access (similar to deckdriversInit)
static void deckDiscoveryInit() {
    static bool init = false;
    if (!init) {
        backends = &_deckBackend_start;
        backendsLen = &_deckBackend_stop - &_deckBackend_start;
        init = true;

        DECK_DISCOVERY_DEBUG("Found %d discovery backends\n", backendsLen);
        for (int i = 0; i < backendsLen; i++) {
            DECK_DISCOVERY_DEBUG("Backend: %s\n", backends[i]->name ? backends[i]->name : "unnamed");
        }
    }
}

int deckDiscoveryBackendCount(void) {
    deckDiscoveryInit();
    return backendsLen;
}

const DeckDiscoveryBackend_t* deckDiscoveryGetBackend(int i) {
    deckDiscoveryInit();

    if (i < backendsLen) {
        return backends[i];
    }
    return NULL;
}