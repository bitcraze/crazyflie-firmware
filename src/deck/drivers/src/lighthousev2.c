/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018-2021 Bitcraze AB
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * lighthouse.c: lighthouse tracking system receiver
 */

#include "deck.h"
#include "param.h"
#include "log.h"
#include "debug.h"

#include "stm32fxxx.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "uart1.h"

#include "lighthouse.h"
#include "lighthouse_core.h"

struct measurement_frame {
    uint32_t sensor_id:2;
    uint32_t polynomial_id:6;
    uint32_t dummy:24;
    uint32_t lfsr_location;
    uint32_t timestamp;
} __attribute__((packed));
typedef struct measurement_frame measurement_frame_t;

#define LIGHTHOUSE_UART_BAUDRATE 230400
#define LIGHTHOUSE_UART_FRAME_LENGTH 12

struct lh2_measurement {
    uint8_t sensor;
    uint32_t timestamp;
    uint8_t selected_polynomial;
    uint32_t lfsr_location;
};

static bool isInit = false;

static void waitForSyncFrame(void)
{
  // A sync frame is 12 0xff in a row
  char c;
  int syncCounter = 0;
  bool synchronized = false;

  while (!synchronized) {
    uart1Getchar(&c);
    if ((unsigned char)c == 0xff) {
      syncCounter += 1;
    } else {
      syncCounter = 0;
    }
    synchronized = (syncCounter == LIGHTHOUSE_UART_FRAME_LENGTH);
  }

  DEBUG_PRINT("Lighthouse V2: Synchronized\n");
}

static bool receiveFrame(struct lh2_measurement *frame)
{
  static uint8_t buffer[LIGHTHOUSE_UART_FRAME_LENGTH];
  // Read the frame from the UART
  uart1GetBytesWithDefaultTimeout(LIGHTHOUSE_UART_FRAME_LENGTH, buffer);

  if (!(  buffer[1] == 0 && buffer[2] == 0 && buffer[3] == 0)) {
    // Not a valid frame, check if it is a sync frame
    if (buffer[0] == 0xff && buffer[1] == 0xff &&
        buffer[2] == 0xff && buffer[3] == 0xff &&
        buffer[4] == 0xff && buffer[5] == 0xff &&
        buffer[6] == 0xff && buffer[7] == 0xff &&
        buffer[8] == 0xff && buffer[9] == 0xff &&
        buffer[10] == 0xff && buffer[11] == 0xff) {
      // Get next frame
      uart1GetBytesWithDefaultTimeout(LIGHTHOUSE_UART_FRAME_LENGTH, buffer);
    } else {
      // Not a valid frame and not a sync frame, we are out of sync
      return false;
    }
  }

  // Decode the frame
  measurement_frame_t *m = (measurement_frame_t *)buffer;
  frame->sensor = m->sensor_id;
  frame->lfsr_location = m->lfsr_location;
  frame->timestamp = m->timestamp;
  frame->selected_polynomial = m->polynomial_id;

  return true;
}

static volatile uint32_t sensor1_pos = 0;

static void lighthousev2Task(void *param)
{
  static struct lh2_measurement frame;
  uart1Init(LIGHTHOUSE_UART_BAUDRATE);

  waitForSyncFrame();

  while(1) {
    
    if (receiveFrame(&frame)) {
      if (frame.sensor == 0 && frame.lfsr_location < 50000) {
        sensor1_pos = frame.lfsr_location;
      }
    } else {
      // We are out of sync, wait for a sync frame
      DEBUG_PRINT("Lighthouse V2: Out of sync\n");
      waitForSyncFrame();
    }
  }
}

static void lighthousev2Init(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  xTaskCreate(lighthousev2Task, LIGHTHOUSE_TASK_NAME,
              LIGHTHOUSE_TASK_STACKSIZE, NULL, LIGHTHOUSE_TASK_PRI, NULL);

  isInit = true;
}

static const DeckDriver lighthousev2_deck = {
  .name = "bcLighthouseV2",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
  .requiredEstimator = StateEstimatorTypeKalman,

  .init = lighthousev2Init,
};

DECK_DRIVER(lighthousev2_deck);

PARAM_GROUP_START(deck)

/**
 * @brief Nonzero if [Lighthouse V2 positioning deck](%https://store.bitcraze.io/collections/decks/products/lighthouse-positioning-deck) is attached
 */ 
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthousev2, &isInit)

PARAM_GROUP_STOP(deck)


LOG_GROUP_START(lighthousev2)
LOG_ADD(LOG_UINT32, position, &sensor1_pos)
LOG_GROUP_STOP(lighthousev2)