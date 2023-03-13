/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
* Copyright (C) 2022 Bitcraze AB & Flapper Drones (https://flapper-drones.com)
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
 * flapperdeck.c: Flapper Nimble+ PCB driver
 */

#define DEBUG_MODULE "FLAPPER"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "extrx.h"
#include "flapperdeck.h"
#include "pm.h"
#include "autoconf.h"
#include "config.h"

static float reading_last = 0.0;
static float current_last = 0.0;
static float current = 0.0;
static float vbat = 0.0;
static float power = 0.0;

static float ampsPerVolt = 2.5; 
static float filter_alpha = 0.975;

static bool isInit;

void flapperDeckInit(DeckInfo* info)
{
  if (isInit)
    return;

  xTaskCreate(flapperDeckTask, FLAPPERDECK_TASK_NAME, FLAPPERDECK_TASK_STACKSIZE, NULL, FLAPPERDECK_TASK_PRI, NULL);

  #if CONFIG_DECK_FLAPPER_MEASURE_VBAT_ON_PA3
  pmEnableExtBatteryVoltMeasuring(DECK_GPIO_RX2, 11.0f);
  #endif

  #if CONFIG_DECK_FLAPPER_EXTRX_ENABLE
  extRxInit();
  #endif

  isInit = true;
}

bool flapperDeckTest(void)
{
  bool testStatus;
  testStatus = true;

  if (!isInit)
    return false;

  return testStatus;
}

void flapperDeckTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(1));

    reading_last = analogReadVoltage(DECK_GPIO_TX2);
    current_last = reading_last*ampsPerVolt;

    // simple low pass filter
    current = filter_alpha*current + (1.0f - filter_alpha)*current_last;

    #ifdef CONFIG_DECK_FLAPPER_MEASURE_VBAT_ON_PA3
    vbat = filter_alpha*vbat + (1.0f - filter_alpha)*pmMeasureExtBatteryVoltage();
    #else
    vbat = pmGetBatteryVoltage();

    #endif
    power = vbat * current;
  }
}

static const DeckDriver flapper_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcFlapperDeck",

  #if CONFIG_DECK_FLAPPER_MEASURE_VBAT_ON_PA3 || CONFIG_DECK_FLAPPER_EXTRX_ENABLE
  .usedGpio = DECK_USING_PA2 | DECK_USING_PA3,
  #else
  .usedGpio = DECK_USING_PA2,
  #endif
  #if CONFIG_DECK_FLAPPER_EXTRX_ENABLE
  .usedPeriph = DECK_USING_TIMER9,
  #endif
  .init = flapperDeckInit,
  .test = flapperDeckTest,
};

DECK_DRIVER(flapper_deck);



PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlapperDeck, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(flapper)
LOG_ADD(LOG_FLOAT, vbat, &vbat)
LOG_ADD(LOG_FLOAT, i_raw, &current_last)
LOG_ADD(LOG_FLOAT, current, &current)
LOG_ADD(LOG_FLOAT, power, &power)
LOG_GROUP_STOP(flapper)

/**
 *
 * Current sensor parameters
 */
PARAM_GROUP_START(flapper)
/**
 * @brief Current sensor constant (A/V)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, ampsPerVolt, &ampsPerVolt)
/**
 * @brief Current filter parameter <0; 1), set 0 to disable, 0.9999 for max effect 
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, filtAlpha, &filter_alpha)

PARAM_GROUP_STOP(flapper)