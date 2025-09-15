/**
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
 *
 * deck_ow.c - Functions to decode the decks oneWire memory content
 */

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define DEBUG_MODULE "DECK_INFO"

// Uncomment to enable verbose debug prints during deck enumeration
// #define DEBUG_DECK_ENUMERATION

#ifdef DEBUG_DECK_ENUMERATION
#define DECK_ENUM_DEBUG(fmt, ...) DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
#define DECK_ENUM_DEBUG(...)
#endif

#include "deck.h"

#include "deck_discovery.h"
#include "crc32.h"
#include "debug.h"
#include "static_mem.h"

#include "autoconf.h"

#ifdef CONFIG_DEBUG
  #define DECK_INFO_DBG_PRINT(fmt, ...)  DEBUG_PRINT(fmt, ## __VA_ARGS__)
#else
  #define DECK_INFO_DBG_PRINT(...)
#endif

static int count = 0;
NO_DMA_CCM_SAFE_ZERO_INIT static DeckInfo deckInfos[DECK_MAX_COUNT];

static void enumerateDecks(void);
static void checkPeriphAndGpioConflicts(void);

static void scanRequiredSystemProperties(void);
static StateEstimatorType requiredEstimator = StateEstimatorTypeAutoSelect;
static bool registerRequiredEstimator(StateEstimatorType estimator);
static bool requiredLowInterferenceRadioMode = false;
static bool requiredKalmanEstimatorAttitudeReversionOff = false;

void deckInfoInit()
{
  static bool isInit = false;

  if (isInit) return;

  enumerateDecks();
  checkPeriphAndGpioConflicts();
  scanRequiredSystemProperties();

  isInit = true;
}

int deckCount(void)
{
  return count;
}

DeckInfo * deckInfo(int i)
{
  if (i<count) {
    return &deckInfos[i];
  }

  return NULL;
}

// Dummy driver for decks that do not have a driver implemented
const DeckDriver dummyDriver;

const DeckDriver * findDriver(DeckInfo *deck)
{
  const DeckDriver *driver = &dummyDriver;

  if (deck->vid) {
    driver = deckFindDriverByVidPid(deck->vid, deck->pid);
  } else if (deck->productName && strlen(deck->productName) > 0) {
    driver = deckFindDriverByName(deck->productName);
  }

  if (driver == NULL)
    driver = &dummyDriver;

  return driver;
}

void printDeckInfo(DeckInfo *info)
{
#ifdef CONFIG_DEBUG
  const char *name = info->productName ? info->productName : "NoName";
  const char *rev = info->boardRevision ? info->boardRevision : "NoRev";
#endif

  DECK_INFO_DBG_PRINT("Deck %02x:%02x %s (Rev. %s)\n", info->vid, info->pid, name, rev);
  DECK_INFO_DBG_PRINT("Used pin: %08x\n", (unsigned int)info->usedPins);

  if (info->driver == &dummyDriver) {
    DEBUG_PRINT("Warning! No driver found for deck.\n");
  } else {
    DECK_INFO_DBG_PRINT("Driver implements: [ %s%s]\n",
                        info->driver->init?"init ":"", info->driver->test?"test ":"");
  }
}

static void enumerateDecks(void)
{
  uint8_t nDecks = 0;
  bool noError = true;

  // Get all available discovery backends
  int numBackends = deckDiscoveryBackendCount();
  DECK_ENUM_DEBUG("Found %d discovery backends\n", numBackends);

  for (int backendIdx = 0; backendIdx < numBackends; backendIdx++) {
    const DeckDiscoveryBackend_t* backend = deckDiscoveryGetBackend(backendIdx);

    if (!backend) {
      DECK_ENUM_DEBUG("Backend %d is NULL\n", backendIdx);
      continue;
    }

    DECK_ENUM_DEBUG("Trying backend: %s\n", backend->name);

    if (!backend->init || !backend->init()) {
      DECK_ENUM_DEBUG("Backend %s failed to initialize\n", backend->name);
      continue;
    }

    // Get decks from this backend
    DeckInfo* deckInfo;
    while ((deckInfo = backend->getNextDeck()) != NULL) {
      if (nDecks >= DECK_MAX_COUNT) {
        DECK_ENUM_DEBUG("Warning: Maximum deck count (%d) reached\n", DECK_MAX_COUNT);
        break;
      }

      // Copy deck info to our array and set backend reference
      deckInfos[nDecks] = *deckInfo;
      deckInfos[nDecks].discoveryBackend = backend;

      // Find appropriate driver for this deck
      deckInfos[nDecks].driver = findDriver(&deckInfos[nDecks]);
      printDeckInfo(&deckInfos[nDecks]);

      DECK_INFO_DBG_PRINT("Added deck from backend %s\n", backend->name);
      nDecks++;
    }

    if (nDecks >= DECK_MAX_COUNT) break;
  }

  if (noError) {
    count = nDecks;
  }

  return;
}

static void checkPeriphAndGpioConflicts(void)
{
  bool noError = true;
  uint32_t usedPeriph = 0;
  uint32_t usedGpio = 0;

  for (int i = 0; i < count; i++)
  {
    uint32_t matchPeriph = usedPeriph & deckInfos[i].driver->usedPeriph;
    if (matchPeriph != 0) {
      //
      // Here we know that two decks share a periph, that is only ok if it is a
      // bus. So, we check if the matching periphs contain a non-bus peripheral
      // by ANDing with the inverse of a mask made up with all bus peripherals.
      //
      uint32_t bus_mask = ~(DECK_USING_I2C | DECK_USING_SPI);
      if ((matchPeriph & bus_mask) != 0) {
        DEBUG_PRINT("ERROR: Driver Periph usage conflicts with a "
                    "previously enumerated deck driver. No decks will be "
                    "initialized!\n");
        noError = false;
      }
    }

    if (usedGpio & deckInfos[i].driver->usedGpio) {
      DEBUG_PRINT("ERROR: Driver Gpio usage conflicts with a "
                  "previously enumerated deck driver. No decks will be "
                  "initialized!\n");
      noError = false;
    }

    usedPeriph |= deckInfos[i].driver->usedPeriph;
    usedGpio |= deckInfos[i].driver->usedGpio;
  }

  if (!noError) {
    count = 0;
  }
}

static void scanRequiredSystemProperties(void)
{
  bool isError = false;

  for (int i = 0; i < count; i++)
  {
    isError = isError || registerRequiredEstimator(deckInfos[i].driver->requiredEstimator);
    requiredLowInterferenceRadioMode |= deckInfos[i].driver->requiredLowInterferenceRadioMode;
    requiredKalmanEstimatorAttitudeReversionOff |= deckInfos[i].driver->requiredKalmanEstimatorAttitudeReversionOff;
  }

  if (isError) {
    count = 0;
  }
}

static bool registerRequiredEstimator(StateEstimatorType estimator)
{
  bool isError = false;

  if (StateEstimatorTypeAutoSelect != estimator)
  {
    if (StateEstimatorTypeAutoSelect == requiredEstimator)
    {
      requiredEstimator = estimator;
    }
    else
    {
      if (requiredEstimator != estimator) {
        isError = true;
        DEBUG_PRINT("WARNING: Two decks require different estimators\n");
      }
    }
  }

  return isError;
}

StateEstimatorType deckGetRequiredEstimator()
{
  return requiredEstimator;
}

bool deckGetRequiredLowInterferenceRadioMode()
{
  return requiredLowInterferenceRadioMode;
}

bool deckGetRequiredKalmanEstimatorAttitudeReversionOff()
{
  return requiredKalmanEstimatorAttitudeReversionOff;
}
