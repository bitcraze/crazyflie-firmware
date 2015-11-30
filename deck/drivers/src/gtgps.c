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
 * exptest.c - Testing of expansion port.
 */
#define DEBUG_MODULE "GTGPS"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "config.h"
#include "console.h"
#include "uart1.h"
#include "debug.h"
#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>

//Hardware configuration
static bool isInit;

char a[] = "Hello";
char b[] = "World";

void gtgpsTask(void *param)
{
  vTaskDelay(2000);
  char ch;
  while(1)
  {
    uart1Getchar(&ch);
    consolePutchar(ch);
  }
}


static void gtgpsInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Enabling reading from GlobalTop GPS\n");
  //motorsInit(motorMapBigQuadDeck);
  uart1Init();

  xTaskCreate(gtgpsTask, "GTGPS",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);

  isInit = true;
}

static bool gtgpsTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver gtgps_deck = {
  .vid = 0xBC,
  .pid = 0x04,
  .name = "bcGTGPS",

  .usedPeriph = DECK_USING_TIMER3,
  .usedGpio = 0,               // FIXME: Edit the used GPIOs

  .init = gtgpsInit,
  .test = gtgpsTest,
};

DECK_DRIVER(gtgps_deck);
