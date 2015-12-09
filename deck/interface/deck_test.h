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
 * deck_test.h - Deck test utility functions
 */

#ifndef __DECK_TEST_H__
#define __DECK_TEST_H__

#include <stdint.h>
#include "stm32fxxx.h"

typedef struct
{
  GPIO_TypeDef gpioBuffA;
  GPIO_TypeDef gpioBuffB;
  GPIO_TypeDef gpioBuffC;
} GpioRegBuf;

/**
 * Deck test evaluation function which makes evaluation
 * a bit less messy. Outputs failstring on console if test
 * failed.
 *
 * param[in]   result      The result of the test.
 * param[in]   failstring  Pointer to fail string.
 * param[out]  status      Saves the test result.
 */
void decktestEval(bool result, char *failString, bool *status);

/**
 * Save GPIO registers into buffer so it can be restored later
 *
 * param[out] gpioRegBuf  Buffer to which registers will be copied
 */
void decktestSaveGPIOStatesABC(GpioRegBuf *gpioRegBuf);

/**
 * Restore saved GPIO registers
 *
 * param[in] gpioRegBuf Buffer of saved registers
 */
void decktestRestoreGPIOStatesABC(GpioRegBuf *gpioRegBuf);

#endif
