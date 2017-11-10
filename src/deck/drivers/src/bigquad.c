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
#define DEBUG_MODULE "BIGQUAD"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "config.h"
#include "motors.h"
#include "debug.h"
#include "deck.h"
#include "param.h"
#include "extrx.h"
#include "pm.h"
#include "uart1.h"
#include "msp.h"

#include "FreeRTOS.h"
#include "task.h"

#define BIGQUAD_BAT_VOLT_PIN       DECK_GPIO_MISO
#define BIGQUAD_BAT_VOLT_MULT      7.8f
#define BIGQUAD_BAT_CURR_PIN       DECK_GPIO_SCK
#define BIGQUAD_BAT_AMP_PER_VOLT   1.0f

#ifdef ENABLE_BQ_DECK
//Hardware configuration
static bool isInit;

#ifdef BQ_DECK_ENABLE_OSD
static MspObject s_MspObject;

static void osdTask(void *param)
{
  while(1)
  {
    char ch;
    uart1Getchar(&ch);

    mspProcessByte(&s_MspObject, (uint8_t)ch);
  }
}

static void osdResponseCallback(uint8_t* pBuffer, uint32_t bufferLen)
{
  uart1SendData(bufferLen, pBuffer);
}
#endif // BQ_DECK_ENABLE_OSD


static void bigquadInit(DeckInfo *info)
{
  if(isInit) {
    return;
  }

  DEBUG_PRINT("Switching to brushless.\n");
  motorsInit(motorMapBigQuadDeck);
  extRxInit();
#ifdef BQ_DECK_ENABLE_PM
  pmEnableExtBatteryVoltMeasuring(BIGQUAD_BAT_VOLT_PIN, BIGQUAD_BAT_VOLT_MULT);
  pmEnableExtBatteryCurrMeasuring(BIGQUAD_BAT_CURR_PIN, BIGQUAD_BAT_AMP_PER_VOLT);
#endif

#ifdef BQ_DECK_ENABLE_OSD
  uart1Init(115200);
  mspInit(&s_MspObject, osdResponseCallback);
  xTaskCreate(osdTask, "BQ_OSDTASK",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
#endif

  isInit = true;
}

static bool bigquadTest()
{
  bool status = true;

  if(!isInit)
    return false;

  status = motorsTest();

  return status;
}

static const DeckDriver bigquad_deck = {
  .vid = 0xBC,
  .pid = 0x05,
  .name = "bcBigQuad",

  .usedPeriph = DECK_USING_TIMER3 | DECK_USING_TIMER14,
  .usedGpio = DECK_USING_PA2 | DECK_USING_PA3 | DECK_USING_PB4 | DECK_USING_PB5 | DECK_USING_PA7,
  .init = bigquadInit,
  .test = bigquadTest,
};

DECK_DRIVER(bigquad_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcBigQuad, &isInit)
PARAM_GROUP_STOP(deck)
#endif // ENABLE_BQ_DECK
