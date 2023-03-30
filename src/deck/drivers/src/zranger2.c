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

#define DEBUG_MODULE "ZR2"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "i2cdev.h"
#include "zranger2.h"
#include "vl53lx_api.h"
#include "vl53lx_platform.h"

#include "cf_math.h"

// Measurement noise model
static const float expPointA = 2.5f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 4.0f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;

#define RANGE_OUTLIER_LIMIT 5000 // the measured range is in [mm]

static uint16_t range_last = 0;

static bool isInit;

static VL53LX_Dev_t dev;

uint16_t range = 8192;

static uint16_t zRanger2GetMeasurementAndRestart(VL53LX_Dev_t *dev)
{
    VL53LX_Error status = VL53LX_ERROR_NONE;
    VL53LX_MultiRangingData_t MultiRangingData;
	  VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t dataReady = 0;

    while (dataReady == 0)
    {
        status = VL53LX_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53LX_GetMultiRangingData(dev, pMultiRangingData);
    int no_of_object_found=pMultiRangingData->NumberOfObjectsFound;
				// DEBUG_PRINT("Count=%5d, ", pMultiRangingData->StreamCount);
				// DEBUG_PRINT("#Objs=%1d \n", no_of_object_found);
        uint16_t range_temp = 8192;
				for(int j=0;j<no_of_object_found;j++){
          // if(pMultiRangingData->RangeData[j].RangeStatus == 0 && pMultiRangingData->RangeData[j].RangeMilliMeter < range_temp && pMultiRangingData->RangeData[j].RangeMaxMilliMeter - pMultiRangingData->RangeData[j].RangeMinMilliMeter < 100)
          if(pMultiRangingData->RangeData[j].RangeStatus == 0 && pMultiRangingData->RangeData[j].RangeMaxMilliMeter < range_temp )
          {
            range = pMultiRangingData->RangeData[j].RangeMaxMilliMeter; 
            range_temp = range;
          }
					// DEBUG_PRINT("status=%d, D=%5dmm, Signal=%2.2f Mcps, Ambient=%2.2f Mcps",
					// 		pMultiRangingData->RangeData[j].RangeStatus,
					// 		pMultiRangingData->RangeData[j].RangeMilliMeter,
					// 		pMultiRangingData->RangeData[j].SignalRateRtnMegaCps/65536.0,
					// 		pMultiRangingData->RangeData[j].AmbientRateRtnMegaCps/65536.0);
          // DEBUG_PRINT("status=%d, D=%5dmm, D=%5dmm, D=%5dmm\n",
					// 		pMultiRangingData->RangeData[j].RangeStatus,
					// 		pMultiRangingData->RangeData[j].RangeMinMilliMeter,
					// 		pMultiRangingData->RangeData[j].RangeMilliMeter,
					// 		pMultiRangingData->RangeData[j].RangeMaxMilliMeter);
				}
        						status = VL53LX_ClearInterruptAndStartMeasurement(dev);


    // VL53LX_StopMeasurement(dev);
    // status = VL53LX_StartMeasurement(dev);
    status = status;

    return range;
}

void zRanger2Init(DeckInfo* info)
{
  if (isInit)
    return;

  if (VL53LX_Init(&dev, I2C1_DEV))
  {
      DEBUG_PRINT("Z-down sensor [OK]\n");
  }
  else
  {
    DEBUG_PRINT("Z-down sensor [FAIL]\n");
    return;
  }

  xTaskCreate(zRanger2Task, ZRANGER2_TASK_NAME, ZRANGER2_TASK_STACKSIZE, NULL, ZRANGER2_TASK_PRI, NULL);

  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  isInit = true;
}

bool zRanger2Test(void)
{
  if (!isInit)
    return false;

  return true;
}

void zRanger2Task(void* arg)
{
  TickType_t lastWakeTime;

  systemWaitStart();

  // Restart sensor
  VL53LX_StopMeasurement(&dev);
  VL53LX_SetDistanceMode(&dev, VL53LX_DISTANCEMODE_MEDIUM);
  VL53LX_SetMeasurementTimingBudgetMicroSeconds(&dev, 25000);

  VL53LX_StartMeasurement(&dev);

  lastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(25));

    range_last = zRanger2GetMeasurementAndRestart(&dev);
    rangeSet(rangeDown, range_last / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >5 [m], and outliers typically
    // occur as >8 [m] measurements
    if (range_last < RANGE_OUTLIER_LIMIT) {
      float distance = (float)range_last * 0.001f; // Scale from [mm] to [m]
      // float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      float stdDev = 0.08f;
      if (distance > 1.0f)
      {
        stdDev = distance*0.04f;
      }
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }
  }
}

static const DeckDriver zranger2_deck = {
  .vid = 0xBC,
  .pid = 0x0E,
  .name = "bcZRanger2",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_I2C,

  .init = zRanger2Init,
  .test = zRanger2Test,
};

DECK_DRIVER(zranger2_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Z-ranger deck v2](%https://store.bitcraze.io/collections/decks/products/z-ranger-deck-v2) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcZRanger2, &isInit)

PARAM_GROUP_STOP(deck)
