/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
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
 *
 * activeMarkerDeck.c - Deck driver for the Active marker deck
 */

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "timers.h"

#include "deck.h"
#include "param.h"
#include "i2cdev.h"

#define DEBUG_MODULE "ACTIVE_MARKER"
#include "debug.h"

#define LED_COUNT 4

#define MEM_ADR_LED 0x0
#define MEM_ADR_VER 0x10

static bool isInit = false;
static bool isVerified = false;
// currentId != requestedID at startup to make sure all IDs are initialized in the deck
static uint8_t currentId[LED_COUNT] = {0xff, 0xff, 0xff, 0xff};
static uint8_t requestedId[LED_COUNT] = {1, 3, 4, 2}; // 1 to 4, clockwise

#define DECK_I2C_ADDRESS 0x2E
#define VERSION_STRING_LEN 12

static xTimerHandle timer;
static char versionString[VERSION_STRING_LEN + 1];

static void timerHandler(xTimerHandle timer);

static void activeMarkerDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  timer = xTimerCreate( "activeMarkerDeckTimer", M2T(1000), pdTRUE, NULL, timerHandler);
  xTimerStart(timer, 100);

  memset(versionString, 0, VERSION_STRING_LEN + 1);
  i2cdevReadReg8(I2C1_DEV, DECK_I2C_ADDRESS, MEM_ADR_VER, VERSION_STRING_LEN, (uint8_t*)versionString);
  DEBUG_PRINT("Deck FW %s\n", versionString);

  isInit = true;
}

static bool activeMarkerDeckTest() {
  if (!isInit) {
    return false;
  }

  isVerified = (0 == strcmp("Qualisys0.A", versionString));
  if (! isVerified) {
    DEBUG_PRINT("Incomaptible deck FW\n");
  }

  return isVerified;
}

static void timerHandler(xTimerHandle timer) {
  if (isVerified) {
    bool isDifferent = false;
    for (int led = 0; led < LED_COUNT; led++) {
      if (currentId[led] != requestedId[led]) {
        isDifferent = true;
        currentId[led] = requestedId[led];
      }
    }

    if (isDifferent) {
        i2cdevWriteReg8(I2C1_DEV, DECK_I2C_ADDRESS, MEM_ADR_LED, LED_COUNT, currentId);
    }
  }
}

static const DeckDriver deck_info = {
  .vid = 0xBC,
  .pid = 0x11,
  .name = "bcActiveM",

  .usedGpio = DECK_USING_SDA | DECK_USING_SCL,

  .init = activeMarkerDeckInit,
  .test = activeMarkerDeckTest,
};

DECK_DRIVER(deck_info);

PARAM_GROUP_START(activeMarker)
PARAM_ADD(PARAM_UINT8, front, &requestedId[0])
PARAM_ADD(PARAM_UINT8, back, &requestedId[1])
PARAM_ADD(PARAM_UINT8, left, &requestedId[2])
PARAM_ADD(PARAM_UINT8, right, &requestedId[3])
PARAM_GROUP_STOP(activeMarker)
