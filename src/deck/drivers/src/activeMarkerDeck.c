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
#include "task.h"

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "i2cdev.h"

#define DEBUG_MODULE "ACTIVE_MARKER"
#include "debug.h"

#define LED_COUNT 4

#define MEM_ADR_LED 0x00
#define MEM_ADR_MODE 0x01
#define MEM_ADR_BUTTON_SENSOR 0x02
#define MEM_ADR_VER 0x10

#define DEFAULT_UPDATE_PERIOD_MS 1000
#define POLL_UPDATE_PERIOD_MS 10

static bool isInit = false;
static bool isVerified = false;

// currentId != requestedID at startup to make sure all IDs are initialized in the deck
static uint8_t currentId[LED_COUNT] = {0xff, 0xff, 0xff, 0xff};
static uint8_t requestedId[LED_COUNT] = {1, 3, 4, 2}; // 1 to 4, clockwise

#define MODE_OFF 0
#define MODE_ON 1
#define MODE_MODULATED 2
#define MODE_QUALISYS 3
#define MODE_UART_TEST 0xff
#define MODE_BUTTON_RESET 0xff

static uint8_t currentDeckMode = MODE_QUALISYS;
static uint8_t requestedDeckMode = MODE_QUALISYS;

// The deck button and sensor data is polled when doPollDeckButtonSensor > 0. Used for production test.
static uint8_t doPollDeckButtonSensor = 0;
static uint8_t deckButtonSensorValue = 0;
static uint32_t nextPollTime = 0;
static const uint32_t pollIntervall = M2T(100);
static bool i2cOk = false;

#ifdef ACTIVE_MARKER_DECK_TEST
static bool activeMarkerDeckCanStart = false;
#endif

#define DECK_I2C_ADDRESS 0x2E
#define VERSION_STRING_LEN 12

enum version_e {
  versionUndefined = 0,
  version_0_A,
  version_1_0,
};

enum version_e deckFwVersion = versionUndefined;

static char versionString[VERSION_STRING_LEN + 1];

static void task(void* param);

static void activeMarkerDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  xTaskCreate(task, "activeMarkerDeck",
              configMINIMAL_STACK_SIZE, NULL, 3, NULL);

#ifndef ACTIVE_MARKER_DECK_TEST
  memset(versionString, 0, VERSION_STRING_LEN + 1);
  i2cOk = i2cdevReadReg8(I2C1_DEV, DECK_I2C_ADDRESS, MEM_ADR_VER, VERSION_STRING_LEN, (uint8_t*)versionString);
  DEBUG_PRINT("Deck FW %s\n", versionString);
#endif

  isInit = true;
}

static bool activeMarkerDeckTest() {
  if (!isInit) {
    return false;
  }

#ifndef ACTIVE_MARKER_DECK_TEST
  if (0 == strcmp("Qualisys0.A", versionString)) {
    deckFwVersion = version_0_A;
  } else if (0 == strcmp("Qualisys1.0", versionString)) {
    deckFwVersion = version_1_0;
  }

  isVerified = (versionUndefined != deckFwVersion);
  if (! isVerified) {
    DEBUG_PRINT("Incompatible deck FW\n");
  }
#else
  isVerified = true;
  deckFwVersion = version_1_0;
#endif

  return isVerified;
}

static void handleIdUpdate() {
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

static void handleModeUpdate() {
  if (currentDeckMode != requestedDeckMode) {
    currentDeckMode = requestedDeckMode;
    i2cdevWriteReg8(I2C1_DEV, DECK_I2C_ADDRESS, MEM_ADR_MODE, 1, &currentDeckMode);
  }
}

static void handleButtonSensorRead() {
  if (doPollDeckButtonSensor) {
    uint32_t now = xTaskGetTickCount();
    if (now > nextPollTime) {
      i2cdevReadReg8(I2C1_DEV, DECK_I2C_ADDRESS, MEM_ADR_BUTTON_SENSOR, 1, &deckButtonSensorValue);
      nextPollTime = now + pollIntervall;
    }
  }
}

static void task(void *param) {
  systemWaitStart();

#ifdef ACTIVE_MARKER_DECK_TEST
  while (!activeMarkerDeckCanStart) {
    vTaskDelay(100);
  }
  i2cOk = i2cdevReadReg8(I2C1_DEV, DECK_I2C_ADDRESS, MEM_ADR_VER, VERSION_STRING_LEN, (uint8_t*)versionString);
#endif

  while (1) {
    if (isVerified) {
      handleIdUpdate();

      if (deckFwVersion >= version_1_0) {
        handleModeUpdate();
        handleButtonSensorRead();
      }
    }

    int delay = DEFAULT_UPDATE_PERIOD_MS;
    if (doPollDeckButtonSensor) {
      delay = POLL_UPDATE_PERIOD_MS;
    }

    vTaskDelay(M2T(delay));
  }
  
}

static const DeckDriver deck_info = {
  .vid = 0xBC,
  .pid = 0x11,
  .name = "bcActiveM",

  .init = activeMarkerDeckInit,
  .test = activeMarkerDeckTest,
};

DECK_DRIVER(deck_info);

PARAM_GROUP_START(activeMarker)
PARAM_ADD(PARAM_UINT8, front, &requestedId[0])
PARAM_ADD(PARAM_UINT8, back, &requestedId[1])
PARAM_ADD(PARAM_UINT8, left, &requestedId[2])
PARAM_ADD(PARAM_UINT8, right, &requestedId[3])
PARAM_ADD(PARAM_UINT8, mode, &requestedDeckMode)
PARAM_ADD(PARAM_UINT8, poll, &doPollDeckButtonSensor)

#ifdef ACTIVE_MARKER_DECK_TEST
PARAM_ADD(PARAM_UINT8, canStart, &activeMarkerDeckCanStart)
#endif

PARAM_GROUP_STOP(activeMarker)

LOG_GROUP_START(activeMarker)
LOG_ADD(LOG_UINT8, btSns, &deckButtonSensorValue)
LOG_ADD(LOG_UINT8, i2cOk, &i2cOk)
LOG_GROUP_STOP(activeMarker)
