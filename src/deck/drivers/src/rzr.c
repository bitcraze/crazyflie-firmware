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
 * rzr.c - Crazyflie RZR board.
 */
#define DEBUG_MODULE "RZR"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "config.h"
#include "motors.h"
#include "debug.h"
#include "deck.h"
#include "extrx.h"
#include "pm.h"

//Hardware configuration
static bool isInit;

static void rzrInit(DeckInfo *info)
{
  if(isInit)
    return;

  DEBUG_PRINT("Switching to brushless.\n");
  motorsInit(motorMapRZRBrushless);

  isInit = true;
}

static bool rzrTest()
{
  bool status = true;

  if(!isInit)
    return false;

  status = motorsTest();

  return status;
}

static const DeckDriver rzr_deck = {
//  .vid = 0xBC,
//  .pid = 0x08,
  .name = "bcRZR",

  .usedPeriph = 0,
  .usedGpio = 0,
  .init = rzrInit,
  .test = rzrTest,
};

DECK_DRIVER(rzr_deck);
