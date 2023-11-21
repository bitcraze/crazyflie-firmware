/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2023 Bitcraze AB
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
 * lighthouse2.c: lighthouse tracking system receiver
 */

#include "deck.h"
#include "param.h"

#include "stm32fxxx.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "lighthouse2_core.h"

static bool isInit;

static void lighthouseInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

  xTaskCreate(lighthouse2CoreTask, LIGHTHOUSE2_TASK_NAME,
              LIGHTHOUSE2_TASK_STACKSIZE, NULL, LIGHTHOUSE2_TASK_PRI, NULL);

  isInit = true;
}

static const DeckMemDef_t memoryDef = {
  .supportsUpgrade = false,
};

static const DeckDriver deckDef = {
  .vid = 0xBC,
  .pid = 0x13,
  .name = "bcLighthouse2",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
  .requiredEstimator = StateEstimatorTypeKalman,

  .memoryDef = &memoryDef,

  .init = lighthouseInit,
};


DECK_DRIVER(deckDef);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if Lighthouse 2.0 deck is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouse2, &isInit)

PARAM_GROUP_STOP(deck)
