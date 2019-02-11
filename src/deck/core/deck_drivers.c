/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * deck_drivers.c - Deck drivers loading and handling
 */

#define DEBUG_MODULE "DECK_DRIVERS"

#include <stdlib.h>
#include <string.h>

#include "deck.h"
#include "debug.h"

#ifdef DEBUG
  #define DECK_DRV_DBG_PRINT(fmt, ...)  DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
  #define DECK_DRV_DBG_PRINT(...)
#endif

/* Symbols set by the linker script */
extern const struct deck_driver * _deckDriver_start;
extern const struct deck_driver * _deckDriver_stop;

static const struct deck_driver ** drivers;
static int driversLen;

// Init the toc access variables. Lazy initialisation: it is going to be done
// the first time any api function is called.
static void deckdriversInit() {
  static bool init = false;
  if (!init) {
    int i;

    drivers = &_deckDriver_start;
    driversLen = &_deckDriver_stop - &_deckDriver_start;
    init = true;

    DECK_DRV_DBG_PRINT("Found %d drivers\n", driversLen);
    for (i=0; i<driversLen; i++) {
      if (drivers[i]->name) {
        DECK_DRV_DBG_PRINT("VID:PID %02x:%02x (%s)\n", drivers[i]->vid, drivers[i]->pid, drivers[i]->name);
      } else {
        DECK_DRV_DBG_PRINT("VID:PID %02x:%02x\n", drivers[i]->vid, drivers[i]->pid);
      }

    }
  }
}

int deckDriverCount() {
  deckdriversInit();

  return driversLen;
}

const struct deck_driver* deckGetDriver(int i) {
  deckdriversInit();

  if (i<driversLen) {
    return drivers[i];
  }
  return NULL;
}

const DeckDriver* deckFindDriverByVidPid(uint8_t vid, uint8_t pid) {
  int i;

  deckdriversInit();

  for (i=0; i<driversLen; i++) {
    if ((vid == drivers[i]->vid) && (pid == drivers[i]->pid)) {
      return drivers[i];
    }
  }
  return NULL;
}

const DeckDriver* deckFindDriverByName(char* name) {
  int i;

  deckdriversInit();

  for (i=0; i<driversLen; i++) {
    if (!strcmp(name, drivers[i]->name)) {
      return drivers[i];
    }
  }
  return NULL;
}
