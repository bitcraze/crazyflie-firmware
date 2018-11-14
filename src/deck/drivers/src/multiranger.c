/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* multiranger.c: Multiranger deck driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "MR"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l1x.h"
#include "range.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;
static bool isPassed = false;

#define MR_PIN_UP     PCA95X4_P0
#define MR_PIN_FRONT  PCA95X4_P4
#define MR_PIN_BACK   PCA95X4_P1
#define MR_PIN_LEFT   PCA95X4_P6
#define MR_PIN_RIGHT  PCA95X4_P2

static VL53L1_Dev_t devFront;
static VL53L1_Dev_t devBack;
static VL53L1_Dev_t devUp;
static VL53L1_Dev_t devLeft;
static VL53L1_Dev_t devRight;

static uint16_t mrGetMeasurementAndRestart(VL53L1_Dev_t *dev)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;
    VL53L1_RangingMeasurementData_t rangingData;
    uint8_t dataReady = 0;
    uint16_t range;

    while (dataReady == 0)
    {
        status = VL53L1_GetMeasurementDataReady(dev, &dataReady);
        vTaskDelay(M2T(1));
    }

    status = VL53L1_GetRangingMeasurementData(dev, &rangingData);
    range = rangingData.RangeMilliMeter;

    VL53L1_StopMeasurement(dev);
    status = VL53L1_StartMeasurement(dev);
    status = status;

    return range;
}

static void mrTask(void *param)
{
    VL53L1_Error status = VL53L1_ERROR_NONE;

    systemWaitStart();

    // Restart all sensors
    status = VL53L1_StopMeasurement(&devFront);
    status = VL53L1_StartMeasurement(&devFront);
    status = VL53L1_StopMeasurement(&devBack);
    status = VL53L1_StartMeasurement(&devBack);
    status = VL53L1_StopMeasurement(&devUp);
    status = VL53L1_StartMeasurement(&devUp);
    status = VL53L1_StopMeasurement(&devLeft);
    status = VL53L1_StartMeasurement(&devLeft);
    status = VL53L1_StopMeasurement(&devRight);
    status = VL53L1_StartMeasurement(&devRight);
    status = status;

    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(100));

        rangeSet(rangeFront, mrGetMeasurementAndRestart(&devFront)/1000.0f);
        rangeSet(rangeBack, mrGetMeasurementAndRestart(&devBack)/1000.0f);
        rangeSet(rangeUp, mrGetMeasurementAndRestart(&devUp)/1000.0f);
        rangeSet(rangeLeft, mrGetMeasurementAndRestart(&devLeft)/1000.0f);
        rangeSet(rangeRight, mrGetMeasurementAndRestart(&devRight)/1000.0f);
    }
}

static void mrInit()
{
    if (isInit)
    {
        return;
    }

    pca95x4Init();

    pca95x4ConfigOutput(~(MR_PIN_UP |
                          MR_PIN_RIGHT |
                          MR_PIN_LEFT |
                          MR_PIN_FRONT |
                          MR_PIN_BACK));

    pca95x4ClearOutput(MR_PIN_UP |
                       MR_PIN_RIGHT |
                       MR_PIN_LEFT |
                       MR_PIN_FRONT |
                       MR_PIN_BACK);

    isInit = true;

    xTaskCreate(mrTask, MULTIRANGER_TASK_NAME, MULTIRANGER_TASK_STACKSIZE, NULL,
        MULTIRANGER_TASK_PRI, NULL);
}

static bool mrTest()
{
    if (isTested)
    {
        return isPassed;
    }

    isPassed = isInit;

    pca95x4SetOutput(MR_PIN_FRONT);
    if (vl53l1xInit(&devFront, I2C1_DEV))
    {
        DEBUG_PRINT("Init front sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init front sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_BACK);
    if (vl53l1xInit(&devBack, I2C1_DEV))
    {
        DEBUG_PRINT("Init back sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init back sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_UP);
    if (vl53l1xInit(&devUp, I2C1_DEV))
    {
        DEBUG_PRINT("Init up sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init up sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_LEFT);
    if (vl53l1xInit(&devLeft, I2C1_DEV))
    {
        DEBUG_PRINT("Init left sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init left sensor [FAIL]\n");
        isPassed = false;
    }

    pca95x4SetOutput(MR_PIN_RIGHT);
    if (vl53l1xInit(&devRight, I2C1_DEV))
    {
        DEBUG_PRINT("Init right sensor [OK]\n");
    }
    else
    {
        DEBUG_PRINT("Init right sensor [FAIL]\n");
        isPassed = false;
    }

    isTested = true;

    return isPassed;
}

static const DeckDriver multiranger_deck = {
    .vid = 0xBC,
    .pid = 0x0C,
    .name = "bcMultiranger",

    .usedGpio = 0, // FIXME: set the used pins

    .init = mrInit,
    .test = mrTest,
};

DECK_DRIVER(multiranger_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcMultiranger, &isInit)
PARAM_GROUP_STOP(deck)
