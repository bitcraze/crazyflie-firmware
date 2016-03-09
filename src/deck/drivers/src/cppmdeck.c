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
#define DEBUG_MODULE "CPPM"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "extrx.h"

//Hardware configuration
static bool isInit;

static void cppmdeckInit(DeckInfo *info)
{
  if(isInit)
    return;

  extRxInit();

  isInit = true;
}

static bool cppmdeckTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver cppm_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcCPPM",

  .usedPeriph = DECK_USING_TIMER14,
  .usedGpio = DECK_USING_PA7,
  .init = cppmdeckInit,
  .test = cppmdeckTest,
};

DECK_DRIVER(cppm_deck);
