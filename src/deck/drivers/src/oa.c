/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2017, Bitcraze AB
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
/* oa.c: Codename Obstacle Avoidance driver */
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "OA"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "pca95x4.h"
#include "vl53l0x.h"

#include "i2cdev.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

static bool isInit = false;
static bool isTested = false;

#define OA_PIN_UP     PCA95X4_P0
#define OA_PIN_FRONT  PCA95X4_P4
#define OA_PIN_BACK   PCA95X4_P1
#define OA_PIN_LEFT   PCA95X4_P6
#define OA_PIN_RIGHT  PCA95X4_P2

static VL53L0xDev devFront;
static VL53L0xDev devBack;
static VL53L0xDev devUp;
static VL53L0xDev devLeft;
static VL53L0xDev devRight;

static uint16_t rangeFront;
static uint16_t rangeBack;
static uint16_t rangeUp;
static uint16_t rangeLeft;
static uint16_t rangeRight;

static void oaTask(void *param)
{
  systemWaitStart();

  vl53l0xStartContinuous(&devFront, 0);
  vl53l0xStartContinuous(&devBack, 0);
  vl53l0xStartContinuous(&devUp, 0);
  vl53l0xStartContinuous(&devLeft, 0);
  vl53l0xStartContinuous(&devRight, 0);

  TickType_t lastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&lastWakeTime, M2T(50));

    rangeFront = vl53l0xReadRangeContinuousMillimeters(&devFront);
    rangeBack = vl53l0xReadRangeContinuousMillimeters(&devBack);
    rangeUp = vl53l0xReadRangeContinuousMillimeters(&devUp);
    rangeLeft = vl53l0xReadRangeContinuousMillimeters(&devLeft);
    rangeRight = vl53l0xReadRangeContinuousMillimeters(&devRight);
  }
}

static void oaInit()
{
  if (isInit) {
    return;
  }

  pca95x4Init();

  pca95x4ConfigOutput(~(OA_PIN_UP |
                        OA_PIN_RIGHT |
                        OA_PIN_LEFT |
                        OA_PIN_FRONT |
                        OA_PIN_BACK));

  pca95x4ClearOutput(OA_PIN_UP |
                     OA_PIN_RIGHT |
                     OA_PIN_LEFT |
                     OA_PIN_FRONT |
                     OA_PIN_BACK);

  isInit = true;

  xTaskCreate(oaTask, OA_DECK_TASK_NAME, 2*configMINIMAL_STACK_SIZE, NULL,
              OA_DECK_TASK_PRI, NULL);
}

static bool oaTest()
{
  bool pass = isInit;

  if (isTested) {
    DEBUG_PRINT("Cannot test OA deck a second time\n");
    return false;
  }

  pca95x4SetOutput(OA_PIN_FRONT);
  if (vl53l0xInit(&devFront, I2C1_DEV, true)) {
    DEBUG_PRINT("Init front sensor [OK]\n");
  } else {
    DEBUG_PRINT("Init front sensor [FAIL]\n");
    pass = false;
  }

  pca95x4SetOutput(OA_PIN_BACK);
  if (vl53l0xInit(&devBack, I2C1_DEV, true)) {
    DEBUG_PRINT("Init back sensor [OK]\n");
  } else {
    DEBUG_PRINT("Init back sensor [FAIL]\n");
    pass = false;
  }

  pca95x4SetOutput(OA_PIN_UP);
  if (vl53l0xInit(&devUp, I2C1_DEV, true)) {
    DEBUG_PRINT("Init up sensor [OK]\n");
  } else {
    DEBUG_PRINT("Init up sensor [FAIL]\n");
    pass = false;
  }

  pca95x4SetOutput(OA_PIN_LEFT);
  if (vl53l0xInit(&devLeft, I2C1_DEV, true)) {
    DEBUG_PRINT("Init left sensor [OK]\n");
  } else {
    DEBUG_PRINT("Init left sensor [FAIL]\n");
    pass = false;
  }

  pca95x4SetOutput(OA_PIN_RIGHT);
  if (vl53l0xInit(&devRight, I2C1_DEV, true)) {
    DEBUG_PRINT("Init right sensor [OK]\n");
  } else {
    DEBUG_PRINT("Init right sensor [FAIL]\n");
    pass = false;
  }

  isTested = true;

  return pass;
}

static const DeckDriver oa_deck = {
  .vid = 0xBC,
  .pid = 0x0B,
  .name = "bcOA",

  .usedGpio = 0,  // FIXME: set the used pins

  .init = oaInit,
  .test = oaTest,
};

DECK_DRIVER(oa_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcOA, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(oa)
LOG_ADD(LOG_UINT16, front, &rangeFront)
LOG_ADD(LOG_UINT16, back, &rangeBack)
LOG_ADD(LOG_UINT16, up, &rangeUp)
LOG_ADD(LOG_UINT16, left, &rangeLeft)
LOG_ADD(LOG_UINT16, right, &rangeRight)
LOG_GROUP_STOP(oa)
