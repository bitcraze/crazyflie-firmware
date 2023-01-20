/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2021, Bitcraze AB
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
/* flowdeck.c: Flow deck driver */
#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "pmw3901.h"
#include "paa3905e1.h"
#include "sleepus.h"

#include "stabilizer_types.h"
#include "estimator.h"
#include "estimator.h"

#include "cf_math.h"

#include "usec_time.h"
#include <stdlib.h>

#define AVERAGE_HISTORY_LENGTH 4
#define OULIER_LIMIT 100
#define LP_CONSTANT 0.8f
// #define USE_LP_FILTER
// #define USE_MA_SMOOTHING

#if defined(USE_MA_SMOOTHING)
static struct {
  float32_t averageX[AVERAGE_HISTORY_LENGTH];
  float32_t averageY[AVERAGE_HISTORY_LENGTH];
  size_t ptr;
} pixelAverages;
#endif

float dpixelx_previous = 0;
float dpixely_previous = 0;

static uint8_t outlierCount = 0;
static float stdFlow = 2.0f;
static uint32_t shutter;
static int16_t flowDeltaX;
static int16_t flowDeltaY;

static bool isInit1 = false;
static bool isInit2 = false;

motionBurst3905_t currentMotion;

// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

// Turn on adaptive standard deviation for the kalman filter
static bool useAdaptiveStd = false;

// Set standard deviation flow
// (will not work if useAdaptiveStd is on)
static float flowStdFixed = 2.0f;

#define NCS_PIN DECK_GPIO_IO3


static void flowdeckTask(void *param)
{
  systemWaitStart();

  uint64_t lastTime  = usecTimestamp();
  while(1) {
    vTaskDelay(10);

    paa3905ReadMotion(NCS_PIN, &currentMotion);
    flowMeasurement_t flowData;
    flowData.dt = (float)(usecTimestamp()-lastTime)/1000000.0f;
    lastTime = usecTimestamp();

    // Flip motion information to comply with sensor mounting
    // (might need to be changed if mounted differently)
    int16_t accpx = -currentMotion.deltaY;
    int16_t accpy = -currentMotion.deltaX;
    shutter = currentMotion.shutter_h << 16 |
              currentMotion.shutter_m << 8 |
              currentMotion.shutter_l;

    // Outlier removal
    if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {
      if (useAdaptiveStd)
      {
        // The standard deviation is fitted by measurements flying over low and high texture
        //   and looking at the shutter time
        stdFlow=0.0007984f *shutter + 0.4335f;

        // The formula with the amount of features instead
        /*float squal_f = (float)currentMotion.squal;
        stdFlow =  -0.01257f * squal_f + 4.406f; */
        if (stdFlow < 0.1f) stdFlow=0.1f;
      } else {
        stdFlow = flowStdFixed;
      }

      // Form flow measurement struct and push into the EKF
      flowData.stdDevX = stdFlow;
      flowData.stdDevY = stdFlow;
      flowData.dt = 0.01;

  #if defined(USE_MA_SMOOTHING)
        // Use MA Smoothing
        pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpx;
        pixelAverages.averageY[pixelAverages.ptr] = (float32_t)accpy;

        float32_t meanX;
        float32_t meanY;

        arm_mean_f32(pixelAverages.averageX, AVERAGE_HISTORY_LENGTH, &meanX);
        arm_mean_f32(pixelAverages.averageY, AVERAGE_HISTORY_LENGTH, &meanY);

        pixelAverages.ptr = (pixelAverages.ptr + 1) % AVERAGE_HISTORY_LENGTH;

        flowData.dpixelx = (float)meanX;   // [pixels]
        flowData.dpixely = (float)meanY;   // [pixels]
  #elif defined(USE_LP_FILTER)
        // Use LP filter measurements
        flowData.dpixelx = LP_CONSTANT * dpixelx_previous + (1.0f - LP_CONSTANT) * (float)accpx;
        flowData.dpixely = LP_CONSTANT * dpixely_previous + (1.0f - LP_CONSTANT) * (float)accpy;
        dpixelx_previous = flowData.dpixelx;
        dpixely_previous = flowData.dpixely;
  #else
        // Use raw measurements
        flowData.dpixelx = (float)accpx;
        flowData.dpixely = (float)accpy;
  #endif
        // Push measurements into the estimator if flow is not disabled
        //    and the PMW flow sensor indicates motion detection
        if (!useFlowDisabled && (currentMotion.motion & 0x80)) {
          flowDeltaX = accpx;
          flowDeltaY = accpy;
          estimatorEnqueueFlow(&flowData);
        } else {
          flowDeltaX = 0;
          flowDeltaY = 0;
        }
      } else {
      outlierCount++;
    }
  }
}

static void flowdeck1Init()
{
  if (isInit1 || isInit2) {
    return;
  }

  // Initialize the VL53L0 sensor using the zRanger deck driver
  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");
  zRanger->init(NULL);

  if (pmw3901Init(NCS_PIN))
  {
    xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL,
                FLOW_TASK_PRI, NULL);

    isInit1 = true;
  }
}

static bool flowdeck1Test()
{
  if (!isInit1) {
    DEBUG_PRINT("Error while initializing the PMW3901 sensor\n");
    return false;
  }

  // Test the VL53L0 driver
  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");

  return zRanger->test();
}

static const DeckDriver flowdeck1_deck = {
  .vid = 0xBC,
  .pid = 0x0A,
  .name = "bcFlow",
  .usedGpio = DECK_USING_IO_3,
  .usedPeriph = DECK_USING_I2C | DECK_USING_SPI,
  .requiredEstimator = kalmanEstimator,

  .init = flowdeck1Init,
  .test = flowdeck1Test,
};

DECK_DRIVER(flowdeck1_deck);

static void flowdeck2Init()
{
  if (isInit1 || isInit2) {
    return;
  }

  // Initialize the VL53L1 sensor using the zRanger deck driver
  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger2");
  zRanger->init(NULL);

  if (paa3905Init(NCS_PIN))
  {
    xTaskCreate(flowdeckTask, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL,
                FLOW_TASK_PRI, NULL);

    isInit2 = true;
  }
}

static bool flowdeck2Test()
{
  if (!isInit2) {
    DEBUG_PRINT("Error while initializing the PMW3901 sensor\n");
    return false;
  }

  // Test the VL53L1 driver
  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger2");

  return zRanger->test();
}

static const DeckDriver flowdeck2_deck = {
  .vid = 0xBC,
  .pid = 0x0F,
  .name = "bcFlow2",

  .usedGpio = DECK_USING_IO_3,
  .usedPeriph = DECK_USING_I2C | DECK_USING_SPI,
  .requiredEstimator = kalmanEstimator,

  .init = flowdeck2Init,
  .test = flowdeck2Test,
};

DECK_DRIVER(flowdeck2_deck);

/**
 * Logging variables of the flow deck
 */
LOG_GROUP_START(flow)
/**
 * @brief Flow X measurement after filtering  [flow/fr]
 */
LOG_ADD(LOG_INT16, deltaX, &flowDeltaX)
/**
 * @brief Flow Y measurement after filtering [flow/fr]
 */
LOG_ADD(LOG_INT16, deltaY, &flowDeltaY)
/**
 * @brief Shutter time [clock cycles]
 */
LOG_ADD(LOG_UINT32, shutter, &shutter)
/**
 * @brief Counted flow outliers excluded from the estimator
 */
LOG_ADD(LOG_UINT8, outlierCount, &outlierCount)
/**
 * @brief Standard deviation of flow measurement
 */
LOG_ADD(LOG_FLOAT, std, &stdFlow)
LOG_GROUP_STOP(flow)

/**
 * Logging variables of the motion sensor of the flowdeck
 */
LOG_GROUP_START(motion)
/**
 * @brief True if motion occurred since the last measurement
 */
LOG_ADD(LOG_UINT8, motion, &currentMotion.motion)
/**
 * @brief Flow X  measurement as from sensor  [flow/fr]
 */
LOG_ADD(LOG_INT16, deltaX, &currentMotion.deltaX)
/**
 * @brief Flow Y measurement as from sensor [flow/fr]
 */
LOG_ADD(LOG_INT16, deltaY, &currentMotion.deltaY)
/**
 * @brief Maximum raw data value in frame
 */
LOG_ADD(LOG_UINT8, maxRaw, &currentMotion.maxRawData)
/**
 * @brief Minimum raw data value in frame
 */
LOG_ADD(LOG_UINT8, minRaw, &currentMotion.minRawData)
/**
 * @brief Average raw data value
 */
LOG_ADD(LOG_UINT8, Rawsum, &currentMotion.rawDataSum)
/**
 * @brief Count of surface feature
 */
LOG_ADD(LOG_UINT8, squal, &currentMotion.squal)
LOG_GROUP_STOP(motion)

/**
 * Settings and parameters for handling of the flowdecks
 * measurments
 */
PARAM_GROUP_START(motion)
/**
 * @brief Nonzero to not push the flow measurement in the EKF (default: 0)
 */
PARAM_ADD(PARAM_UINT8, disable, &useFlowDisabled)
/**
 * @brief Nonzero to turn on adaptive standard deviation estimation (default: 0)
 */
PARAM_ADD(PARAM_UINT8, adaptive, &useAdaptiveStd)
/**
 * @brief Set standard deviation flow measurement (default: 2.0f)
 */
PARAM_ADD_CORE(PARAM_FLOAT, flowStdFixed, &flowStdFixed)
PARAM_GROUP_STOP(motion)

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if Flow deck v1 is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow, &isInit1)

/**
 * @brief Nonzero if [Flow deck v2](%https://store.bitcraze.io/collections/decks/products/flow-deck-v2) is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow2, &isInit2)

PARAM_GROUP_STOP(deck)
