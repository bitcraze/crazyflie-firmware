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
/* flowdeck.c: Flow deck driver */
#include "deck.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"

#include "FreeRTOS.h"
#include "task.h"

#include "sleepus.h"

#include "stabilizer_types.h"
#include "estimator.h"
#include "estimator_kalman.h"

#include "arm_math.h"

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

static bool isInit = false;

// Disables pushing the flow measurement in the EKF
static bool useFlowDisabled = false;

#define NCS_PIN DECK_GPIO_IO3

typedef struct motionBurst_s {
  union {
    uint8_t motion;
    struct {
      uint8_t frameFrom0    : 1;
      uint8_t runMode       : 2;
      uint8_t reserved1     : 1;
      uint8_t rawFrom0      : 1;
      uint8_t reserved2     : 2;
      uint8_t motionOccured : 1;
    };
  };

  uint8_t observation;
  int16_t deltaX;
  int16_t deltaY;

  uint8_t squal;

  uint8_t rawDataSum;
  uint8_t maxRawData;
  uint8_t minRawData;

  uint16_t shutter;
} __attribute__((packed)) motionBurst_t;

motionBurst_t currentMotion;

static void InitRegisters();



static void registerWrite(uint8_t reg, uint8_t value)
{
  // Set MSB to 1 for write
  reg |= 0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(NCS_PIN, LOW);

  sleepus(50);

  spiExchange(1, &reg, &reg);
  sleepus(50);
  spiExchange(1, &value, &value);

  sleepus(50);

  digitalWrite(NCS_PIN, HIGH);
  spiEndTransaction();
  sleepus(200);
}

static uint8_t registerRead(uint8_t reg)
{
  uint8_t data = 0;
  uint8_t dummy = 0;

  // Set MSB to 0 for read
  reg &= ~0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(NCS_PIN, LOW);

  sleepus(50);

  spiExchange(1, &reg, &reg);
  sleepus(500);
  spiExchange(1, &dummy, &data);

  sleepus(50);

  digitalWrite(NCS_PIN, HIGH);
  spiEndTransaction();
  sleepus(200);

  return data;
}

static void readMotion(motionBurst_t * motion)
{
  uint8_t address = 0x16;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(NCS_PIN,LOW);
  sleepus(50);
  spiExchange(1, &address, &address);
  sleepus(50);
  spiExchange(sizeof(motionBurst_t), (uint8_t*)motion, (uint8_t*)motion);
  sleepus(50);
  digitalWrite(NCS_PIN, HIGH);
  spiEndTransaction();
  sleepus(50);

  uint16_t realShutter = (motion->shutter >> 8) & 0x0FF;
  realShutter |= (motion->shutter & 0x0ff) << 8;
  motion->shutter = realShutter;
}

static void InitRegisters()
{
  registerWrite(0x7F, 0x00);
  registerWrite(0x61, 0xAD);
  registerWrite(0x7F, 0x03);
  registerWrite(0x40, 0x00);
  registerWrite(0x7F, 0x05);
  registerWrite(0x41, 0xB3);
  registerWrite(0x43, 0xF1);
  registerWrite(0x45, 0x14);
  registerWrite(0x5B, 0x32);
  registerWrite(0x5F, 0x34);
  registerWrite(0x7B, 0x08);
  registerWrite(0x7F, 0x06);
  registerWrite(0x44, 0x1B);
  registerWrite(0x40, 0xBF);
  registerWrite(0x4E, 0x3F);
  registerWrite(0x7F, 0x08);
  registerWrite(0x65, 0x20);
  registerWrite(0x6A, 0x18);
  registerWrite(0x7F, 0x09);
  registerWrite(0x4F, 0xAF);
  registerWrite(0x5F, 0x40);
  registerWrite(0x48, 0x80);
  registerWrite(0x49, 0x80);
  registerWrite(0x57, 0x77);
  registerWrite(0x60, 0x78);
  registerWrite(0x61, 0x78);
  registerWrite(0x62, 0x08);
  registerWrite(0x63, 0x50);
  registerWrite(0x7F, 0x0A);
  registerWrite(0x45, 0x60);
  registerWrite(0x7F, 0x00);
  registerWrite(0x4D, 0x11);
  registerWrite(0x55, 0x80);
  registerWrite(0x74, 0x1F);
  registerWrite(0x75, 0x1F);
  registerWrite(0x4A, 0x78);
  registerWrite(0x4B, 0x78);
  registerWrite(0x44, 0x08);
  registerWrite(0x45, 0x50);
  registerWrite(0x64, 0xFF);
  registerWrite(0x65, 0x1F);
  registerWrite(0x7F, 0x14);
  registerWrite(0x65, 0x67);
  registerWrite(0x66, 0x08);
  registerWrite(0x63, 0x70);
  registerWrite(0x7F, 0x15);
  registerWrite(0x48, 0x48);
  registerWrite(0x7F, 0x07);
  registerWrite(0x41, 0x0D);
  registerWrite(0x43, 0x14);
  registerWrite(0x4B, 0x0E);
  registerWrite(0x45, 0x0F);
  registerWrite(0x44, 0x42);
  registerWrite(0x4C, 0x80);
  registerWrite(0x7F, 0x10);
  registerWrite(0x5B, 0x02);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x41);
  registerWrite(0x70, 0x00);

  vTaskDelay(M2T(10)); // delay 10ms

  registerWrite(0x32, 0x44);
  registerWrite(0x7F, 0x07);
  registerWrite(0x40, 0x40);
  registerWrite(0x7F, 0x06);
  registerWrite(0x62, 0xF0);
  registerWrite(0x63, 0x00);
  registerWrite(0x7F, 0x0D);
  registerWrite(0x48, 0xC0);
  registerWrite(0x6F, 0xD5);
  registerWrite(0x7F, 0x00);
  registerWrite(0x5B, 0xA0);
  registerWrite(0x4E, 0xA8);
  registerWrite(0x5A, 0x50);
  registerWrite(0x40, 0x80);

  registerWrite(0x7F, 0x00);
  registerWrite(0x5A, 0x10);
  registerWrite(0x54, 0x00);
}

static void pamotionTask(void *param)
{
  systemWaitStart();

  while(1) {
    vTaskDelay(10);

    readMotion(&currentMotion);

    // Flip motion information to comply with sensor mounting
    // (might need to be changed if mounted differently)
    int16_t accpx = -currentMotion.deltaY;
    int16_t accpy = -currentMotion.deltaX;

    // Outlier removal
    if (abs(accpx) < OULIER_LIMIT && abs(accpy) < OULIER_LIMIT) {

      // Form flow measurement struct and push into the EKF
      flowMeasurement_t flowData;
      flowData.stdDevX = 0.25;    // [pixels] should perhaps be made larger?
      flowData.stdDevY = 0.25;    // [pixels] should perhaps be made larger?
      flowData.dt = 0.01;

#if defined(USE_MA_SMOOTHING)
      // Use MA Smoothing
      pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpx;
      pixelAverages.averageX[pixelAverages.ptr] = (float32_t)accpy;

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
      // Push measurements into the Kalman filter
      if (!useFlowDisabled) {
        estimatorKalmanEnqueueFlow(&flowData);
      }
    } else {
      outlierCount++;
    }
  }
}

static void pamotionInit()
{
  if (isInit) {
    return;
  }

  // Initialize the VL53 sensor using the zRanger deck driver
  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");
  zRanger->init(NULL);

  // Initialize CS Pin
  pinMode(NCS_PIN, OUTPUT);
  digitalWrite(NCS_PIN, HIGH);

  spiBegin();
  vTaskDelay(M2T(40));

  digitalWrite(NCS_PIN, HIGH);
  vTaskDelay(M2T(2));
  digitalWrite(NCS_PIN, LOW);
  vTaskDelay(M2T(2));
  digitalWrite(NCS_PIN, HIGH);
  vTaskDelay(M2T(2));


  uint8_t chipId = registerRead(0);
  uint8_t invChipId = registerRead(0x5f);

  consolePrintf("Motion chip is: 0x%x\n", chipId);
  consolePrintf("si pihc noitoM: 0x%x\n", invChipId);

  // Power on reset
  registerWrite(0x3a, 0x5a);
  vTaskDelay(M2T(5));

  // Reading the motion registers one time
  registerRead(0x02);
  registerRead(0x03);
  registerRead(0x04);
  registerRead(0x05);
  registerRead(0x06);
  vTaskDelay(M2T(1));

  InitRegisters();

  isInit = true;

  xTaskCreate(pamotionTask, "pamotion", 2*configMINIMAL_STACK_SIZE, NULL,
              /*priority*/3, NULL);
}

static bool pamotionTest()
{
  if (!isInit) {
    DEBUG_PRINT("Error while initializing the motion sensor\n");
  }

  // Test the VL53 driver
  const DeckDriver *zRanger = deckFindDriverByName("bcZRanger");

  return zRanger->test();
}

static const DeckDriver pamotion_deck = {
  .vid = 0xBC,
  .pid = 0x0A,
  .name = "bcFlow",

  .usedGpio = 0,  // FIXME: set the used pins
  .requiredEstimator = kalmanEstimator,

  .init = pamotionInit,
  .test = pamotionTest,
};

DECK_DRIVER(pamotion_deck);

LOG_GROUP_START(motion)
LOG_ADD(LOG_UINT8, motion, &currentMotion.motion)
LOG_ADD(LOG_INT16, deltaX, &currentMotion.deltaX)
LOG_ADD(LOG_INT16, deltaY, &currentMotion.deltaY)
LOG_ADD(LOG_UINT16, shutter, &currentMotion.shutter)
LOG_ADD(LOG_UINT8, maxRaw, &currentMotion.maxRawData)
LOG_ADD(LOG_UINT8, minRaw, &currentMotion.minRawData)
LOG_ADD(LOG_UINT8, Rawsum, &currentMotion.rawDataSum)
LOG_ADD(LOG_UINT8, outlierCount, &outlierCount)
LOG_GROUP_STOP(motion)

PARAM_GROUP_START(motion)
PARAM_ADD(PARAM_UINT8, disable, &useFlowDisabled)
PARAM_GROUP_STOP(motion)

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcFlow, &isInit)
PARAM_GROUP_STOP(deck)
