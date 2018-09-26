/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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

#include "system.h"
#include "deck.h"
#include "log.h"

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "uart1.h"

#include "pulseProcessor.h"
#include "lighthouseGeometry.h"

#include "estimator_kalman.h"


static float angles[2][2];

typedef union frame_u {
  struct {
    uint32_t timestamp:29;
    uint32_t sensor:3;
    uint16_t width;
    uint8_t sync;
  } __attribute__((packed));
  char data[7];
} __attribute__((packed)) frame_t;

static bool getFrame(frame_t *frame)
{
  int syncCounter = 0;
  for(int i=0; i<7; i++) {
    uart1Getchar(&frame->data[i]);
    if (frame->data[i] != 0) {
      syncCounter += 1;
    }
  }
  return (frame->sync == 0 || (syncCounter==7));
}

static vec3d position;

baseStationGeometry_t baseStationsGeometry[] = {
  {.origin = {-2.029516, 2.391417, -1.356382, }, .mat = {{-0.718327, 0.285313, -0.634511, }, {0.066982, 0.936164, 0.345125, }, {0.692474, 0.205412, -0.691582, }, }},
  {.origin = {1.027486, 2.587440, 1.884445, }, .mat = {{0.846093, -0.256320, 0.467361, }, {-0.021730, 0.859477, 0.510712, }, {-0.532592, -0.442266, 0.721628, }, }},
};

static positionMeasurement_t ext_pos;

static void lighthouseTask(void *param)
{
  bool synchronized = false;
  int syncCounter = 0;
  char c;
  static frame_t frame;
  static pulseProcessor_t ppState = {};

  float angle;
  int basestation;
  int axis;

  float delta;

  systemWaitStart();

  while(1) {
    // Synchronize
    // DEBUG_PRINT("Resynchronizing ...!\n");
    syncCounter = 0;
    while (!synchronized) {
      
      uart1Getchar(&c);
      if (c != 0) {
        syncCounter += 1;
      } else {
        syncCounter = 0;
      }
      synchronized = syncCounter == 7;
    }

    // DEBUG_PRINT("Sync!\n");


    // Receive data until being desynchronized
    synchronized = getFrame(&frame);
    while(synchronized) {
      if (frame.sync != 0) {
        // DEBUG_PRINT("Sync!\n");
        synchronized = getFrame(&frame);
        continue;
      }

      if (frame.sensor == 0) {
        // DEBUG_PRINT("Reading %08X:%04X\n", frame.timestamp, frame.width);
        if (processPulse(&ppState, frame.timestamp, frame.width, &angle, &basestation, &axis)) {
          angles[basestation][axis] = angle;
          
          if (basestation == 1 && axis == 1) {
            lhgeometryGetPosition(baseStationsGeometry, (void*)angles, position, &delta);

            ext_pos.x = position[0];
            ext_pos.y = -position[2];
            ext_pos.z = position[1];
            ext_pos.stdDev = 0.01;
            estimatorKalmanEnqueuePosition(&ext_pos);
          }
        }
      } else {
        DEBUG_PRINT("Receiving from wrong sensor?!?!?!\n");
      }

      synchronized = getFrame(&frame);
      if (frame.sync != 0) {
        // DEBUG_PRINT("Sync!\n");
        synchronized = getFrame(&frame);
        continue;
      }
    }
  }
}


static void lighthouseInit(DeckInfo *info)
{
  uart1Init(115200);

  xTaskCreate(lighthouseTask, "LH",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
}


static const DeckDriver lighthouse_deck = {
  .name = "bcLH8",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = lighthouseInit,
};

DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, angle0x, &angles[0][0])
LOG_ADD(LOG_FLOAT, angle0y, &angles[1][0])
LOG_ADD(LOG_FLOAT, angle1x, &angles[0][1])
LOG_ADD(LOG_FLOAT, angle1y, &angles[1][1])
LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])
LOG_GROUP_STOP(lighthouse)
