/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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

#include "i2cdev.h"
#include "zranger.h"
#include "vl53l1x.h"
#include "vl53l1_api.h"

#include "stabilizer_types.h"

#include "estimator.h"
#include "estimator_kalman.h"
#include "arm_math.h"

// Measurement noise model
static float expPointA = 1.5f;
static float expStdA = 0.0025f; // STD at elevation expPointA [m]
static float expPointB = 2.0f;
static float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 8000 // the measured range is in [mm]

static int16_t range_last = 0;

static bool isInit;

static VL53L1_Dev_t dev;

void zRangerInit(DeckInfo* info)
{
  if (isInit)
    return;

  vl53l1xInit(&dev, I2C1_DEV);

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

  testStatus = vl53l1xTestConnection(&dev);

  return testStatus;
}

void zRangerTask(void* arg)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint8_t dataReady = 0;
  VL53L1_RangingMeasurementData_t rangingData;
  // VL53L1_UserRoi_t roiSetting;

//  roiSetting.

  systemWaitStart();
  TickType_t xLastWakeTime;

  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&dev, 190);
  status = VL53L1_SetPresetMode(&dev, VL53L1_PRESETMODE_AUTONOMOUS);

  xLastWakeTime = xTaskGetTickCount();
  status = VL53L1_StopMeasurement(&dev);
  status = VL53L1_StartMeasurement(&dev);

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, M2T(1));

    dataReady = 0;
    while (dataReady == 0) {
      status = VL53L1_GetMeasurementDataReady(&dev, &dataReady);
      vTaskDelay(M2T(1));
    }
    if (1)
    {
      status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
      range_last = rangingData.RangeMilliMeter;

      // check if range is feasible and push into the kalman filter
      // the sensor should not be able to measure >3 [m], and outliers typically
      // occur as >8 [m] measurements
      if (getStateEstimator() == kalmanEstimator &&
          range_last < RANGE_OUTLIER_LIMIT) {
        // Form measurement
        tofMeasurement_t tofData;
        tofData.timestamp = xTaskGetTickCount();
        tofData.distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
        tofData.stdDev = expStdA * (1.0f  + expf( expCoeff * ( tofData.distance - expPointA)));
        estimatorKalmanEnqueueTOF(&tofData);
      }

//      status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
      VL53L1_StopMeasurement(&dev);
      status = VL53L1_StartMeasurement(&dev);
      status = status;
    }
  }
}

bool zRangerReadRange(zDistance_t* zrange, const uint32_t tick)
{
  bool updated = false;

  if (isInit) {
    if (range_last != 0 && range_last < RANGE_OUTLIER_LIMIT) {
      zrange->distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      zrange->timestamp = tick;
      updated = true;
    }
  }
  return updated;
}

static const DeckDriver zranger_deck = {
  .vid = 0xBC,
  .pid = 0x09,
  .name = "bcZRanger",
  .usedGpio = 0x0C,

  .init = zRangerInit,
  .test = zRangerTest,
};

DECK_DRIVER(zranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcZRanger, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(range)
LOG_ADD(LOG_UINT16, zrange, &range_last)
LOG_GROUP_STOP(range)
