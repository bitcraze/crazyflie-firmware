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

static volatile uint32_t sensor1_pos = 0;

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
}

static bool receiveFrame(lighthouseUartFrame_t *frame)
{
  static uint8_t buffer[LIGHTHOUSE_UART_FRAME_LENGTH];

  memset(frame, 0, sizeof(*frame));

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
      frame->isSyncFrame = true;
      return true;
    } else {
      // Not a valid frame and not a sync frame, we are out of sync
      return false;
    }
  }

  // Decode the frame
  measurement_frame_t *m = (measurement_frame_t *)buffer;
  frame->data.sensor = m->sensor_id;
  frame->data.offset = m->lfsr_location;
  // The offset is expressed in a 6 MHz clock, convert to the 24 MHz that is used for timestamps
  frame->data.offset *= 4;

  frame->data.timestamp = (m->timestamp*24) & 0x00FFFFFF;
  frame->data.channelFound = true;
  frame->data.channel = m->polynomial_id/2;
  frame->data.slowBit = m->polynomial_id&0x01;

  frame->data.width = 1000;

  if (frame->data.sensor == 0) {
    // The first sensor is the one that is used for the position
    sensor1_pos = frame->data.slowBit;
  }

  return true;
}


static void lighthousev2Task(void *param)
{
  uart1Init(LIGHTHOUSE_UART_BAUDRATE);

  lighthouseCoreInit();

  lighthouseCoreMainLoop(waitForSyncFrame, receiveFrame);
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
PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcLighthouseV2, &isInit)

PARAM_GROUP_STOP(deck)


LOG_GROUP_START(lighthousev2)
LOG_ADD(LOG_UINT32, position, &sensor1_pos)
LOG_GROUP_STOP(lighthousev2)