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

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "deck.h"
#include "log.h"

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "LH"
#include "debug.h"
#include "uart1.h"
#include "lh_bootloader.h"

#include "pulse_processor.h"
#include "lighthouse_geometry.h"

#include "estimator.h"

#ifndef DISABLE_LIGHTHOUSE_DRIVER
  #define DISABLE_LIGHTHOUSE_DRIVER 1
#endif

baseStationGeometry_t baseStationsGeometry[] = {
{.origin = {-1.866722, 2.229666, 1.521226, }, .mat = {{0.710527, 0.378001, -0.593521, }, {0.027454, 0.827931, 0.560158, }, {0.703134, -0.414302, 0.577889, }, }},
{.origin = {2.158244, 2.287099, -1.858179, }, .mat = {{-0.645509, -0.380170, 0.662412, }, {0.017664, 0.859648, 0.510581, }, {-0.763548, 0.341286, -0.548195, }, }},
};

#if DISABLE_LIGHTHOUSE_DRIVER == 0

#define STR2(x) #x
#define STR(x) STR2(x)

#define INCBIN(name, file) \
    __asm__(".section .rodata\n" \
            ".global incbin_" STR(name) "_start\n" \
            ".align 4\n" \
            "incbin_" STR(name) "_start:\n" \
            ".incbin \"" file "\"\n" \
            \
            ".global incbin_" STR(name) "_end\n" \
            ".align 1\n" \
            "incbin_" STR(name) "_end:\n" \
            ".byte 0\n" \
            ".align 4\n" \
            STR(name) "Size:\n" \
            ".int incbin_" STR(name) "_end - incbin_" STR(name) "_start\n" \
    ); \
    extern const __attribute__((aligned(4))) void* incbin_ ## name ## _start; \
    extern const void* incbin_ ## name ## _end; \
    extern const int name ## Size; \
    static const __attribute__((used)) unsigned char* name = (unsigned char*) & incbin_ ## name ## _start; \

INCBIN(bitstream, "lighthouse.bin");

static void checkVersionAndBoot();

static bool isInit = false;

static pulseProcessorResult_t angles[PULSE_PROCESSOR_N_SENSORS];

// Stats
static int serialFrameCount = 0;
static int frameCount = 0;
static int cycleCount = 0;
static int positionCount = 0;

static float serialFrameRate = 0.0;
static float frameRate = 0.0;
static float cycleRate = 0.0;
static float positionRate = 0.0;

static uint16_t pulseWidth[PULSE_PROCESSOR_N_SENSORS];

static uint32_t latestStatsTimeMs = 0;

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

static void resetStats() {
  serialFrameCount = 0;
  frameCount = 0;
  cycleCount = 0;
  positionCount = 0;
}

static void calculateStats(uint32_t nowMs) {
  double time = (nowMs - latestStatsTimeMs) / 1000.0;
  serialFrameRate = serialFrameCount / time;
  frameRate = frameCount / time;
  cycleRate = cycleCount / time;
  positionRate = positionCount / time;

  resetStats();
}

static vec3d position;
static positionMeasurement_t ext_pos;
static void estimatePosition(pulseProcessorResult_t angles[]) {
  memset(&ext_pos, 0, sizeof(ext_pos));
  int sensorsUsed = 0;
  float delta;

  // Average over all sensors with valid data
  for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
      if (angles[sensor].validCount == 4) {
        lighthouseGeometryGetPosition(baseStationsGeometry, (void*)angles[sensor].angles, position, &delta);

        ext_pos.x -= position[2];
        ext_pos.y -= position[0];
        ext_pos.z += position[1];
        sensorsUsed++;

        positionCount++;
      }
  }

  ext_pos.x /= sensorsUsed;
  ext_pos.y /= sensorsUsed;
  ext_pos.z /= sensorsUsed;

  // Make sure we feed sane data into the estimator
  if (!isfinite(ext_pos.pos[0]) || !isfinite(ext_pos.pos[1]) || !isfinite(ext_pos.pos[2])) {
    return;
  }
  ext_pos.stdDev = 0.01;
  estimatorEnqueuePosition(&ext_pos);
}

static void lighthouseTask(void *param)
{
  bool synchronized = false;
  int syncCounter = 0;
  char c;
  static frame_t frame;
  static pulseProcessor_t ppState = {};

  int basestation;
  int axis;

  systemWaitStart();

  // Boot the deck firmware
  checkVersionAndBoot();

  while(1) {
    // Synchronize
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

    DEBUG_PRINT("Synchronized!\n");

    // Receive data until being desynchronized
    synchronized = getFrame(&frame);
    while(synchronized) {
      if (frame.sync != 0) {
        synchronized = getFrame(&frame);
        memset(pulseWidth, 0, sizeof(pulseWidth[0])*PULSE_PROCESSOR_N_SENSORS);
        continue;
      }

      serialFrameCount++;

      pulseWidth[frame.sensor] = frame.width;

      if (pulseProcessorProcessPulse(&ppState, frame.sensor, frame.timestamp, frame.width, angles, &basestation, &axis)) {
        frameCount++;
        if (basestation == 1 && axis == 1) {
          cycleCount++;
          estimatePosition(angles);
          for (size_t sensor = 0; sensor < PULSE_PROCESSOR_N_SENSORS; sensor++) {
            angles[sensor].validCount = 0;
          }
        }
      }

      uint32_t nowMs = T2M(xTaskGetTickCount());
      if ((nowMs - latestStatsTimeMs) > 1000) {
        calculateStats(nowMs);
        latestStatsTimeMs = nowMs;
      }

      synchronized = getFrame(&frame);
      if (frame.sync != 0) {
        synchronized = getFrame(&frame);
        continue;
      }
    }
  }
}

static void checkVersionAndBoot()
{

  uint8_t bootloaderVersion = 0;
  lhblGetVersion(&bootloaderVersion);
  DEBUG_PRINT("Lighthouse bootloader version: %d\n", bootloaderVersion);

  // Wakeup mem
  lhblFlashWakeup();
  vTaskDelay(M2T(1));

  // Checking if first and last 64 bytes of bitstream are identical
  // Also decoding bitstream version for console
  static char deckBitstream[65];
  lhblFlashRead(LH_FW_ADDR, 64, (uint8_t*)deckBitstream);
  deckBitstream[64] = 0;
  int deckVersion = strtol(&deckBitstream[2], NULL, 10);
  int embeddedVersion = strtol((char*)&bitstream[2], NULL, 10);

  bool identical = true;
  if (memcmp(deckBitstream, bitstream, 64)) {
    DEBUG_PRINT("Fail comparing begining\n");
    identical = false;
  }

  lhblFlashRead(LH_FW_ADDR + (bitstreamSize - 64), 64, (uint8_t*)deckBitstream);
  if (memcmp(deckBitstream, &bitstream[(bitstreamSize - 64)], 64)) {
    DEBUG_PRINT("Fail comparing end\n");
    identical = false;
  }

  if (identical == false) {
    DEBUG_PRINT("Deck has version %d and we embeed version %d\n", deckVersion, embeddedVersion);
    DEBUG_PRINT("Updating deck with embedded version!\n");

    // Erase LH deck FW
    lhblFlashEraseFirmware();

    // Flash LH deck FW
    if (lhblFlashWriteFW((uint8_t*)bitstream, bitstreamSize)) {
      DEBUG_PRINT("FW updated [OK]\n");
    } else {
      DEBUG_PRINT("FW updated [FAILED]\n");
    }
  }

  // Launch LH deck FW
  DEBUG_PRINT("Firmware version %d verified, booting deck!\n", deckVersion);
  lhblBootToFW();
}

static void lighthouseInit(DeckInfo *info)
{
  if (isInit) return;

  uart1Init(230400);
  lhblInit(I2C1_DEV);
  
  xTaskCreate(lighthouseTask, "LH",
              configMINIMAL_STACK_SIZE, NULL, /*priority*/1, NULL);
  
  isInit = true;
}


static const DeckDriver lighthouse_deck = {
  .vid = 0xBC,
  .pid = 0x10,
  .name = "bcLighthouse4",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = lighthouseInit,
};

DECK_DRIVER(lighthouse_deck);

LOG_GROUP_START(lighthouse)
LOG_ADD(LOG_FLOAT, angle0x, &angles[0].angles[0][0])
LOG_ADD(LOG_FLOAT, angle0y, &angles[0].angles[1][0])
LOG_ADD(LOG_FLOAT, angle1x, &angles[0].angles[0][1])
LOG_ADD(LOG_FLOAT, angle1y, &angles[0].angles[1][1])
LOG_ADD(LOG_FLOAT, x, &position[0])
LOG_ADD(LOG_FLOAT, y, &position[1])
LOG_ADD(LOG_FLOAT, z, &position[2])

LOG_ADD(LOG_FLOAT, serRt, &serialFrameRate)
LOG_ADD(LOG_FLOAT, frmRt, &frameRate)
LOG_ADD(LOG_FLOAT, cycleRt, &cycleRate)
LOG_ADD(LOG_FLOAT, posRt, &positionRate)

LOG_ADD(LOG_UINT16, width0, &pulseWidth[0])
#if PULSE_PROCESSOR_N_SENSORS > 1
LOG_ADD(LOG_UINT16, width1, &pulseWidth[1])
#endif
#if PULSE_PROCESSOR_N_SENSORS > 2
LOG_ADD(LOG_UINT16, width2, &pulseWidth[2])
#endif
#if PULSE_PROCESSOR_N_SENSORS > 3
LOG_ADD(LOG_UINT16, width3, &pulseWidth[3])
#endif
LOG_GROUP_STOP(lighthouse)


#endif // DISABLE_LIGHTHOUSE_DRIVER
