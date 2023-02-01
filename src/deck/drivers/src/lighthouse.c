/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018-2021 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouse.c: lighthouse tracking system receiver
 */

#include "deck.h"
#include "param.h"

#include "stm32fxxx.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "lighthouse.h"
#include "lighthouse_core.h"
#include "lighthouse_deck_flasher.h"

// LED timer
static StaticTimer_t timerBuffer;
#define FIFTH_SECOND 200
static void ledTimerHandle(xTimerHandle timer);

static bool isInit = false;
// lighthouseBaseStationsGeometry has been moved to lighthouse_core.c

static void lighthouseInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  lighthouseCoreInit();

  xTaskCreate(lighthouseCoreTask, LIGHTHOUSE_TASK_NAME,
              LIGHTHOUSE_TASK_STACKSIZE, NULL, LIGHTHOUSE_TASK_PRI, NULL);

  xTimerHandle timer;
  timer = xTimerCreateStatic("ledTimer", M2T(FIFTH_SECOND), pdTRUE,
    NULL, ledTimerHandle, &timerBuffer);
  xTimerStart(timer, M2T(0));

  isInit = true;
}

static void ledTimerHandle(xTimerHandle timer) {
  lighthouseCoreLedTimer();
}

static const DeckMemDef_t memoryDef = {
  .write = lighthouseDeckFlasherWrite,
  .read = lighthouseDeckFlasherRead,
  .properties = lighthouseDeckFlasherPropertiesQuery,
  .supportsUpgrade = true,

  .requiredSize = LIGHTHOUSE_BITSTREAM_SIZE,
  .requiredHash = LIGHTHOUSE_BITSTREAM_CRC,
};

static const DeckDriver lighthouse_deck = {
  .vid = 0xBC,
  .pid = 0x10,
  .name = "bcLighthouse4",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
  .requiredEstimator = StateEstimatorTypeKalman,

  .memoryDef = &memoryDef,

  .init = lighthouseInit,
};


DECK_DRIVER(lighthouse_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Lighthouse positioning deck](%https://store.bitcraze.io/collections/decks/products/lighthouse-positioning-deck) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse4, &isInit)

PARAM_GROUP_STOP(deck)
