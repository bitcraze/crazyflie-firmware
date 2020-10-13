/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 * activeMarkerUartTest.c: Production test for Active marker deck UART
 *                               Uart
 */

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "param.h"
#include "debug.h"

#include "deck.h"
#include "uart1.h"

#define DEBUG_MODULE "ActiveMarkerDeckUartTest"

#include "log.h"

static bool isInit = false;

static bool trigger = false;
static bool passed = false;

static void task(void *param)
{
  while(1) {
    if (trigger) {
      trigger = false;

      DEBUG_PRINT("Sending...\n");
      uart1Putchar(0x55);

      uint8_t answer;
      DEBUG_PRINT("Waiting...\n");
      if (uart1GetDataWithDefaultTimeout(&answer)) {
        DEBUG_PRINT("Received %02x\n", (unsigned int)answer);

        if (answer == 0xaa) {
          passed = true;
        }
      } else {
        DEBUG_PRINT("Timeout!!!\n");
      }

    }
    vTaskDelay(M2T(100));
  }

}

static void init(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  DEBUG_PRINT("Initializing Active Marker deck Uart test driver...\n");

  uart1InitWithParity(115200, uart1ParityNone);

  xTaskCreate(task, "amarkUartTest",
              configMINIMAL_STACK_SIZE, NULL, 3, NULL);

  isInit = true;
}

static const DeckDriver deckDriver = {
  .name="activeMarkerUartTest",

  .init = init,
};

DECK_DRIVER(deckDriver);


PARAM_GROUP_START(amarkUartTest)
PARAM_ADD(PARAM_UINT8, trigger, &trigger)
PARAM_GROUP_STOP(amarkUartTest)

LOG_GROUP_START(amarkUartTest)
LOG_ADD(LOG_UINT8, passed, &passed)
LOG_GROUP_STOP(amarkUartTest)
