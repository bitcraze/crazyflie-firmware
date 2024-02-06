/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * uart2test.c - Uart echo implementation to test uart.
 */
#define DEBUG_MODULE "U1T"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32fxxx.h"
#include "system.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "uart2.h"

//Hardware configuration
static bool isInit;

void uart2testTask(void* arg);

static void uart2testInit(DeckInfo *info)
{
  if(isInit)
    return;

  uart2Init(115200);

  xTaskCreate(uart2testTask, UART2_TEST_TASK_NAME, UART2_TEST_TASK_STACKSIZE, NULL, UART2_TEST_TASK_PRI, NULL);

  isInit = true;
}

static bool uart2testTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

void uart2testTask(void* arg)
{
  systemWaitStart();

  while (1)
  {
    char c;
    uart2Getchar(&c);
    consolePutchar(c);
    uart2Putchar(c);
    //uart2SendDataDmaBlocking(36, (uint8_t *)"Testing UART2 DMA and it is working\n");
  }
}

static const DeckDriver uart2test_deck = {
//  .vid = 0xBC,
//  .pid = 0x08,
  .name = "bcUart2Test",

  .usedPeriph = DECK_USING_UART2,
  .usedGpio = 0,
  .init = uart2testInit,
  .test = uart2testTest,
};

DECK_DRIVER(uart2test_deck);
