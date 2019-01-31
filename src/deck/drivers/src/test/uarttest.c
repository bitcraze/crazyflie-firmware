/*
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
 * uartTest.c - Testing of the deck uarts.
 *
 * Connect deck UART1_TX to UART2_RX and UART2_TX to UART1_RX
 *
 */
#define DEBUG_MODULE "UART-TEST"


#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32fxxx.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "uart1.h"
#include "uart2.h"


static const char testString[] = "ABC123";

static bool uarttestRun()
{
  bool status = true;
  char testChar;

  uart1Init(9600);
  uart2Init(9600);

  for (int i = 0; i < sizeof(testString) && status; i++)
  {
    uart1Putchar(testString[i]);
    uart2GetDataWithTimout(&testChar);
    if (testChar != testString[i])
    {
      DEBUG_PRINT(" Uart1->Uart2 [FAIL]\n");
      status = false;
    }

    uart2Putchar(testString[i]);
    uart1GetDataWithTimout(&testChar);
    if (testChar != testString[i])
    {
      DEBUG_PRINT(" Uart2->Uart1 [FAIL]\n");
      status = false;
    }
  }

  if (status)
  {
    DEBUG_PRINT("Read/write test [OK]\n");
  }

  return status;
}

static const DeckDriver uarttest_deck = {
  .name = "bcUartTest",

  .usedPeriph = 0,
  .usedGpio = 0,

  .test = uarttestRun,
};

DECK_DRIVER(uarttest_deck);

