/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
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
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "VLX"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"
#include "zranger.h"
#include "vl53l0x.h"

#include "cf_math.h"

// Measurement noise model
static const float expPointA = 1.0f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 1.3f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;

static VL53L0xDev dev;

void zRangerInit(DeckInfo* info)
{
  if (isInit)
    return;

  vl53l0xInit(&dev, I2C1_DEV, true);

  xTaskCreate(zRangerTask, ZRANGER_TASK_NAME, ZRANGER_TASK_STACKSIZE, NULL, ZRANGER_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRangerTest(void)
{
  bool testStatus;

  if (!isInit)
    return false;

  testStatus  = vl53l0xTestConnection(&dev);

  return testStatus;
}

void zRangerTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodPreRange, 18);
  vl53l0xSetVcselPulsePeriod(&dev, VcselPeriodFinalRange, 14);
  vl53l0xStartContinuous(&dev, 0);

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(dev.measurement_timing_budget_ms));

    range_last = vl53l0xReadRangeContinuousMillimeters(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT) {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

static const DeckDriver zranger_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcZRanger",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_I2C,

  .init = zRangerInit,
  .test = zRangerTest,
};

DECK_DRIVER(zranger_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Z-ranger deck](%https://store.bitcraze.io/collections/decks/products/z-ranger-deck) is attached
*/
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)

PARAM_GROUP_STOP(deck)
