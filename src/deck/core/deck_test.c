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
 *
 * deck_test.c - Test utility functions for testing decks.
 */
#include <string.h>

#include "stm32fxxx.h"

#include "deck.h"
#include "deck_test.h"
#include "debug.h"


#ifndef DECK_TEST_PRINT_ALL_FAILED
  #define STATUS_EVAL (*status)
#else
  #define STATUS_EVAL 1
#endif


void decktestEval(bool result, char *failString, bool *status)
{
  if (STATUS_EVAL)
  {
    if (!result)
    {
      consolePrintf("%s [FAIL]\n", failString);
      *status = false;
    }
    else
    {
      *status = true;
    }
  }
}

void decktestSaveGPIOStatesABC(GpioRegBuf *gpioRegBuf)
{
  // Save GPIO registers
  memcpy(&gpioRegBuf->gpioBuffA, GPIOA, sizeof(GPIO_TypeDef));
  memcpy(&gpioRegBuf->gpioBuffB, GPIOB, sizeof(GPIO_TypeDef));
  memcpy(&gpioRegBuf->gpioBuffC, GPIOC, sizeof(GPIO_TypeDef));
}

void decktestRestoreGPIOStatesABC(GpioRegBuf *gpioRegBuf)
{
  // Restore GPIO registers
  memcpy(GPIOA, &gpioRegBuf->gpioBuffA, sizeof(GPIO_TypeDef));
  memcpy(GPIOB, &gpioRegBuf->gpioBuffB, sizeof(GPIO_TypeDef));
  memcpy(GPIOC, &gpioRegBuf->gpioBuffC, sizeof(GPIO_TypeDef));
}
