/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 */

/**
 * @file deck.c 
 * @brief Deck subsystem main entry points
 *
 *
 * @ingroup decks
 */

#define DEBUG_MODULE "DECK_CORE"

#include <string.h>

#include "deck.h"
#include "debug.h"

#ifdef DEBUG
  #define DECK_CORE_DBG_PRINT(fmt, ...)  DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
  #define DECK_CORE_DBG_PRINT(...)
#endif

extern void deckInfoInit();

/**
 * @brief Initialize drivers for all decks found during startup
 *
 * Initializes all decks found via enumerateDecks().
 *
 * @see enumerateDecks()
 */
void deckInit()
{
  deckDriverCount();
  deckInfoInit();

  int nDecks;
  int i;

  nDecks = deckCount();

  DEBUG_PRINT("%d deck(s) found\n", nDecks);

  for (i=0; i<nDecks; i++) {
    DeckInfo *deck = deckInfo(i);

    if (deck->driver->init) {
      if (deck->driver->name) {
        DEBUG_PRINT("Calling INIT on driver %s for deck %i\n", deck->driver->name, i);
      } else {
        DEBUG_PRINT("Calling INIT for deck %i\n", i);
      }

      deck->driver->init(deck);
    }
  }
}

bool deckTest()
{
  bool pass = true;
  int nDecks;
  int i;

  nDecks = deckCount();

  for (i=0; i<nDecks; i++) {
    DeckInfo *deck = deckInfo(i);

    if (deck->driver->test) {
      if (deck->driver->test()) {
        DEBUG_PRINT("Deck %i test [OK].\n", i);
      } else {
        DEBUG_PRINT("Deck %i test [FAIL].\n", i);
        pass = false;
      }
    }
  }

  return pass;
}
