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
 * deck_backend_forced.c - Forced deck backend for CONFIG_DECK_FORCE
 */

#include <string.h>
#include <stdlib.h>

#include "deck_discovery.h"
#include "deck.h"
#include "autoconf.h"

#define DEBUG_MODULE "DECK_FORCED"
#include "debug.h"

// Uncomment to enable debug prints for forced backend
// #define DEBUG_FORCED_BACKEND

#ifdef DEBUG_FORCED_BACKEND
#define FORCED_BACKEND_DEBUG(fmt, ...) DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
#define FORCED_BACKEND_DEBUG(...)
#endif

static char* deck_force = CONFIG_DECK_FORCE;
static uint8_t forcedDeckCount = 0;
static uint8_t currentDeck = 0;
static const DeckDriver* forcedDrivers[DECK_MAX_COUNT];
static DeckInfo deckBuffer; // Buffer for returning deck info

static bool forcedBackendInit(void)
{
  forcedDeckCount = 0;
  currentDeck = 0;

  // Only proceed if we have forced decks configured
  if (strlen(deck_force) == 0 || strncmp(deck_force, "none", 4) == 0) {
    return true;  // No forced decks, but init successful
  }

  DEBUG_PRINT("CONFIG_DECK_FORCE=%s found\n", deck_force);

  // Parse colon-separated deck names
  char temp_deck_force[strlen(deck_force) + 1];
  strcpy(temp_deck_force, deck_force);

  char* token = strtok(temp_deck_force, ":");
  while (token && forcedDeckCount < DECK_MAX_COUNT) {
    const DeckDriver *driver = deckFindDriverByName(token);
    if (!driver) {
      DEBUG_PRINT("WARNING: compile-time forced driver %s not found\n", token);
    } else if (driver->init || driver->test) {
      forcedDrivers[forcedDeckCount] = driver;
      forcedDeckCount++;
      FORCED_BACKEND_DEBUG("compile-time forced driver %s prepared\n", token);
    }
    token = strtok(NULL, ":");
  }

  FORCED_BACKEND_DEBUG("Prepared %d forced deck(s)\n", forcedDeckCount);
  return true;
}

static DeckInfo* forcedBackendGetNextDeck(void)
{
  if (currentDeck >= forcedDeckCount) {
    return NULL; // No more forced decks
  }

  FORCED_BACKEND_DEBUG("Returning forced deck %d\n", currentDeck);

  // Clear the deck buffer
  memset(&deckBuffer, 0, sizeof(deckBuffer));

  // Set the driver (this is all forced decks need)
  deckBuffer.driver = forcedDrivers[currentDeck];
  deckBuffer.discoveryBackend = NULL; // Will be set by enumerateDecks

  currentDeck++;
  return &deckBuffer;
}

static const DeckDiscoveryBackend_t forcedBackend = {
    .name = "forced",
    .init = forcedBackendInit,
    .getNextDeck = forcedBackendGetNextDeck,
};

DECK_DISCOVERY_BACKEND(forcedBackend);
