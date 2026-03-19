/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright 2026, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
/* flowdeck_v3.c: UART template for Flow deck V3 */

#define DEBUG_MODULE "FlowDeckV3"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "deck.h"
#include "debug.h"
#include "param.h"
#include "log.h"
#include "system.h"
#include "uart1.h"
#include "range.h"

#include "cf_math.h"

#include "flowdeck_v3.h"
#include "stabilizer_types.h"


#define OULIER_LIMIT 100
#define RANGE_OUTLIER_LIMIT 3000 // the measured range is in [mm]


static uint8_t resolution = 0x4c;
static flowMeasurement_t flowData;

static uint8_t outlierCount = 0;
static float stdFlow = 2.0f;

static bool isInit = false;
static flowdeckV3UartFrame_t rxFrame;

// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

// Turn on adaptive standard deviation for the kalman filter
static bool useAdaptiveStd = false;

// Set standard deviation flow
// (will not work if useAdaptiveStd is on)
static float flowStdFixed = 2.0f;

// Range sensor measurement noise model
static const float expPointA = 1.0f;
static const float expStdA = 0.0025f; // STD at elevation expPointA [m]
static const float expPointB = 1.3f;
static const float expStdB = 0.2f;    // STD at elevation expPointB [m]
static float expCoeff;


static void flowdeckV3ReadData(flowdeckV3UartFrame_t *frame) {
  uint8_t *raw = (uint8_t *)frame;
  uint16_t header = 0;

  // the deck sends a sync header before the start of each frame.
  // Wait for it.
  while (header != FLOWDECK_V3_UART_SYNC_HEADER) {
    while (uart1bytesAvailable() < 1) {
      vTaskDelay(M2T(1));
    }
    uint8_t byte;
    uart1Getchar((char *) &byte);
    header = (header << 8) | byte;
  }

  while (uart1bytesAvailable() < sizeof(*frame)) {
    vTaskDelay(M2T(1));
  }

  for (uint32_t i = 0; i < sizeof(*frame); i++) {
    uart1Getchar((char *)&raw[i]);
  }
}

static void flowdeckV3Task(void *param) {
  (void)param;

  uart1Init(FLOWDECK_V3_UART_BAUDRATE);
  systemWaitStart();

  ASSERT(uart1QueueMaxLength() >= sizeof(flowdeckV3UartFrame_t));

  uint32_t frameCount = 0;
  while (1) {
    flowdeckV3ReadData(&rxFrame);

    // Flow -------------------------------------------------------
    // Flip motion information to comply with sensor mounting
    // (might need to be changed if mounted differently)
    int16_t accpx = (int16_t) -((int32_t) rxFrame.deltaY + INT16_MIN);
    int16_t accpy = (int16_t) -((int32_t) rxFrame.deltaX + INT16_MIN);

    // Outlier removal
    if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {
       if (useAdaptiveStd) {
        // The standard deviation is fitted by measurements flying over low and high texture
        //   and looking at the shutter time
        float shutter_f = (float)rxFrame.shutter;
        stdFlow=0.0007984f *shutter_f + 0.4335f;

        // The formula with the amount of features instead
        /*float squal_f = (float)currentMotion.squal;
        stdFlow =  -0.01257f * squal_f + 4.406f; */
        if (stdFlow < 0.1f) stdFlow=0.1f;
      } else {
        stdFlow = flowStdFixed;
      }
    
      flowData.stdDevX = stdFlow * 0.1f;
      flowData.stdDevY = stdFlow * 0.1f;
      flowData.dt = 1.0f / 126.0f;
      frameCount++;

      flowData.dpixelx = (float) accpx;
      flowData.dpixely = (float) accpy;
      
      // Push measurements into the estimator if flow is not disabled
      // and the PMW flow sensor indicates motion detection
      if (!useFlowDisabled && rxFrame.motion & 0x80) {
        estimatorEnqueueFlow(&flowData);
      }
    } else {
      outlierCount++;
    }

    // Z-range -------------------------------------------------------
    rangeSet(rangeDown, rxFrame.rangeMm / 1000.0f);

    // check if range is feasible and push into the estimator
    // the sensor should not be able to measure >3 [m], and outliers typically
    // occur as >8 [m] measurements
    if (rxFrame.rangeMm < RANGE_OUTLIER_LIMIT) {
      float distance = (float) rxFrame.rangeMm * 0.001f; // Scale from [mm] to [m]
      float stdDev = expStdA * (1.0f  + expf( expCoeff * (distance - expPointA)));
      rangeEnqueueDownRangeInEstimator(distance, stdDev, xTaskGetTickCount());
    }

  }
}


static void flowdeck3Init(DeckInfo *info) {
  (void)info;

  if (isInit) {
    return;
  }
  
  // pre-compute constant in the measurement noise model for kalman
  expCoeff = logf(expStdB / expStdA) / (expPointB - expPointA);

  xTaskCreate(flowdeckV3Task, FLOW_TASK_NAME, FLOW_TASK_STACKSIZE, NULL,
              FLOW_TASK_PRI, NULL);

  isInit = true;
}

static bool flowdeck3Test(void) {
  if (!isInit) {
    DEBUG_PRINT("Error while initializing the flow V3 deck\n");
    return false;
  }

  return uart1Test();
}

static const DeckDriver flowdeck3_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcFlow3",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
  .requiredEstimator = StateEstimatorTypeKalman,

  .init = flowdeck3Init,
  .test = flowdeck3Test,
};

DECK_DRIVER(flowdeck3_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if the Flow deck V3 template driver is attached
 */
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcFlow3, &isInit)

PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(flow)
PARAM_ADD(PARAM_UINT8, resolution, &resolution)
PARAM_GROUP_STOP(flow)
