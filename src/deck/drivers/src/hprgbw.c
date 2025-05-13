/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
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
 *
 * hpRGB.c - Deck driver for the High Power RGBW deck
 */

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "deck.h"
#include "log.h"
#include "param.h"
#include "eventtrigger.h"
#include "i2cdev.h"

#define DEBUG_MODULE "HP_RGBW"
#include "debug.h"

static bool isInit = false;
static uint32_t rgbw8888;
// static struct RGBW {
//   uint32_t bot;
//   uint32_t top;
// } rgbw8888;

#define DECK_I2C_ADDRESS 0x10

//static void task(void* param);

static void hprgbwDeckInit(DeckInfo *info) {
  if (isInit) {
    return;
  }

//  xTaskCreate(task, HPRGBW__TASK_NAME,
//              ACTIVEMARKER_TASK_STACKSIZE, NULL, HPRGBW__TASK_PRI, NULL);

  isInit = true;
}

static bool hprgbwDeckTest() {
  if (!isInit) {
    return false;
  }

  return true;
}

static void updateDeck(void)
{
  // FIXME: Don't do blocking i2c write here...
  i2cdevWrite(I2C1_DEV, DECK_I2C_ADDRESS, sizeof(rgbw8888), (uint8_t *)&rgbw8888);
}

//static void task(void *param) {
//  systemWaitStart();
//
//  while (1)
//  {
//
//  }
//}

static const DeckDriver hprgbw_deck = {
  .vid = 0xBC,
  .pid = 0x12,
  .name = "bcHPRGBW",

  .init = hprgbwDeckInit,
  .test = hprgbwDeckTest,
};

DECK_DRIVER(hprgbw_deck);

PARAM_GROUP_START(hprgbw)
// PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, topRGBW, &rgbw8888.top, &updateDeck)
// PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, botRGBW, &rgbw8888.bot, &updateDeck)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT32, rgbw8888, &rgbw8888, &updateDeck)
PARAM_GROUP_STOP(hprgbw)
