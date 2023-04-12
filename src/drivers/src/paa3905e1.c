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
/* paa3905e1.c: PAA3905E1 driver */
#define DEBUG_MODULE "PAA"

#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

#include "paa3905e1.h"
#include "deck.h"
#include "debug.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "sleepus.h"
#include "usec_time.h"


#define TURN_ON_FLOW_DECK_LED


static bool isInit = false;
static uint8_t mode = 0; //0: Bright mode, 1: Low light mode, 2: Super low light mode (read only currently)
static uint8_t resolution = 0x4c;//0x26 corresponds to 0.1pixel, which was the resolution in flowV2, 0xFF is the max;
static uint8_t old_resolution = 0x4c;

static volatile uint8_t rawDataArray[1225];

static float dt = 0.0f;
static uint64_t lastTime = 0;

static void registerWrite(const deckPin_t csPin, uint8_t reg, uint8_t value)
{
  // Set MSB to 1 for write
  reg |= 0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin, LOW);

  sleepus(10);

  spiExchange(1, &reg, &reg);
  sleepus(10);
  spiExchange(1, &value, &value);

  digitalWrite(csPin, HIGH);
  spiEndTransaction();
}

static uint8_t registerRead(const deckPin_t csPin, uint8_t reg)
{
  uint8_t data = 0;
  uint8_t dummy = 0;

  // Set MSB to 0 for read
  reg &= ~0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin, LOW);

  sleepus(10);

  spiExchange(1, &reg, &reg);
  sleepus(10);
  spiExchange(1, &dummy, &data);

  digitalWrite(csPin, HIGH);
  spiEndTransaction();

  return data;
}

static uint8_t registerReadFast(const deckPin_t csPin, uint8_t reg)
{
  uint8_t data = 0;
  uint8_t dummy = 0;

  // Set MSB to 0 for read
  reg &= ~0x80u;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin, LOW);

  sleepus(2);

  spiExchange(1, &reg, &reg);
  sleepus(2);
  spiExchange(1, &dummy, &data);

  digitalWrite(csPin, HIGH);
  spiEndTransaction();

  return data;
}



static void InitRegisters(const deckPin_t csPin)
{

  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x51, 0xFF);
  registerWrite(csPin, 0x4E, 0x2A);
  registerWrite(csPin, 0x66, 0x3E);
  registerWrite(csPin, 0x7F, 0x14);
  registerWrite(csPin, 0x7E, 0x71);
  registerWrite(csPin, 0x55, 0x00);
  registerWrite(csPin, 0x59, 0x00);
  registerWrite(csPin, 0x6F, 0x2C);
  registerWrite(csPin, 0x7F, 0x05);
  registerWrite(csPin, 0x4D, 0xAC);
  registerWrite(csPin, 0x4E, 0x32);
  registerWrite(csPin, 0x7F, 0x09);
  registerWrite(csPin, 0x5C, 0xAF);
  registerWrite(csPin, 0x5F, 0xAF);
  registerWrite(csPin, 0x70, 0x08);
  registerWrite(csPin, 0x71, 0x04);
  registerWrite(csPin, 0x72, 0x06);
  registerWrite(csPin, 0x74, 0x3C);
  registerWrite(csPin, 0x75, 0x28);
  registerWrite(csPin, 0x76, 0x20);
  registerWrite(csPin, 0x4E, 0xBF);
  registerWrite(csPin, 0x7F, 0x03);
  registerWrite(csPin, 0x64, 0x14);
  registerWrite(csPin, 0x65, 0x0A);
  registerWrite(csPin, 0x66, 0x10);
  registerWrite(csPin, 0x55, 0x3C);
  registerWrite(csPin, 0x56, 0x28);
  registerWrite(csPin, 0x57, 0x20);
  registerWrite(csPin, 0x4A, 0x2D);
  registerWrite(csPin, 0x4B, 0x2D);
  registerWrite(csPin, 0x4E, 0x4B);
  registerWrite(csPin, 0x69, 0xFA);
  registerWrite(csPin, 0x7F, 0x05);
  registerWrite(csPin, 0x69, 0x1F);
  registerWrite(csPin, 0x47, 0x1F);
  registerWrite(csPin, 0x48, 0x0C);
  registerWrite(csPin, 0x5A, 0x20);
  registerWrite(csPin, 0x75, 0x0F);
  registerWrite(csPin, 0x4A, 0x0F);
  registerWrite(csPin, 0x42, 0x02);
  registerWrite(csPin, 0x45, 0x03);
  registerWrite(csPin, 0x65, 0x00);
  registerWrite(csPin, 0x67, 0x76);
  registerWrite(csPin, 0x68, 0x76);
  registerWrite(csPin, 0x6A, 0xC5);
  registerWrite(csPin, 0x43, 0x00);
  registerWrite(csPin, 0x7F, 0x06);
  registerWrite(csPin, 0x4A, 0x18);
  registerWrite(csPin, 0x4B, 0x0C);
  registerWrite(csPin, 0x4C, 0x0C);
  registerWrite(csPin, 0x4D, 0x0C);
  registerWrite(csPin, 0x46, 0x0A);
  registerWrite(csPin, 0x59, 0xCD);
  registerWrite(csPin, 0x7F, 0x0A);
  registerWrite(csPin, 0x4A, 0x2A);
  registerWrite(csPin, 0x48, 0x96);
  registerWrite(csPin, 0x52, 0xB4);
  registerWrite(csPin, 0x7F, 0x00);
  // registerWrite(csPin, 0x5B, 0xA0);

#ifdef TURN_ON_FLOW_DECK_LED
  // turn on LED
  registerWrite(csPin, 0x7F, 0x14);
  registerWrite(csPin, 0x6F, 0x0c);
  registerWrite(csPin, 0x7F, 0x00);
  DEBUG_PRINT("Turned on LED\n");
#endif


}

bool paa3905Init(const deckPin_t csPin)
{
  if (isInit) {
    return true;
  }

  // Initialize CS Pin
  pinMode(csPin, OUTPUT);
  digitalWrite(csPin, HIGH);

  spiBegin();
  vTaskDelay(M2T(40));

  digitalWrite(csPin, HIGH);
  vTaskDelay(M2T(2));
  digitalWrite(csPin, LOW);
  vTaskDelay(M2T(2));
  digitalWrite(csPin, HIGH);
  vTaskDelay(M2T(2));

  uint8_t chipId    = registerRead(csPin, 0x00);
  uint8_t invChipId = registerRead(csPin, 0x5f);

  DEBUG_PRINT("Motion chip id: 0x%x:0x%x\n", chipId, invChipId);

  if (chipId == 0xA2 && invChipId == 0x5D)
  {
    // Power on reset
    registerWrite(csPin, 0x3a, 0x5a);
    vTaskDelay(M2T(5));

    // Reading the motion registers one time
    registerRead(csPin, 0x02);
    registerRead(csPin, 0x03);
    registerRead(csPin, 0x04);
    registerRead(csPin, 0x05);
    registerRead(csPin, 0x06);
    vTaskDelay(M2T(1));

    InitRegisters(csPin);

    DEBUG_PRINT("0x5B: 0x%x\n", registerRead(csPin, 0x5B));
    // Fix orientation for PAA3905
    registerWrite(csPin, 0x5B, 0xA0);
    DEBUG_PRINT("0x5B: 0x%x\n", registerRead(csPin, 0x5B));
    registerWrite(csPin, 0x4E, resolution);

    isInit = true;
  }

  return isInit;
}

void paa3905ReadMotion(const deckPin_t csPin, motionBurst3905_t * motion)
{
  uint8_t address = 0x16;

  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  digitalWrite(csPin,LOW);
  sleepus(10);
  spiExchange(1, &address, &address);
  sleepus(10);
  spiExchange(sizeof(motionBurst3905_t), (uint8_t*)motion, (uint8_t*)motion);
  sleepus(10);
  digitalWrite(csPin, HIGH);
  spiEndTransaction();

  uint32_t shutter = motion->shutter_h << 16 |
                     motion->shutter_m << 8 |
                     motion->shutter_l;

  mode = (motion->observation >> 6);
  switch(motion->observation >> 6)
  {
    case 0x00:
      motion->motion = ((motion->motion & 0x80) && (motion->squal > 0x19) && (shutter < 0x00FF80)) ? 0x80 : 0x00;
      break;
    case 0x01:
      motion->motion = ((motion->motion & 0x80) && (motion->squal > 0x46) && (shutter < 0x00FF80)) ? 0x80 : 0x00;
      break;
    case 0x02:
      motion->motion = ((motion->motion & 0x80) && (motion->squal > 0x55) && (shutter < 0x025998)) ? 0x80 : 0x00;
      break;
    default:
      motion->motion = 0x40;
      //motion->motion = motion->motion & 0x80;
      break;
  }

  if (old_resolution != resolution)
  {
    registerWrite(csPin, 0x4E, resolution);
    resolution = registerRead(csPin, 0x4E);
    old_resolution = resolution;
  }
}

void paa3905ReadRaw(const deckPin_t csPin)
{
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x67, 0x25);
  registerWrite(csPin, 0x55, 0x20);
  registerWrite(csPin, 0x7F, 0x13);
  registerWrite(csPin, 0x42, 0x01);
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x0F, 0x11);
  registerWrite(csPin, 0x0F, 0x13);
  registerWrite(csPin, 0x0F, 0x11);
  
  uint8_t tempStatus = 0;
  while( !(tempStatus & 0x01) ) {
    tempStatus = registerRead(csPin, 0x10) & 0x01; // wait for grab status bit 0 to equal 1
    vTaskDelay(M2T(1));
  }
  registerWrite(csPin, 0x13, 0xFF); //dummy value
  for (int i = 0; i < 49; ++i)
  {
    for (int j = 0; j < 25; ++j)
    {
      rawDataArray[i*25+j] = registerReadFast(csPin, 0x13);
      // DEBUG_PRINT("%d %d %d %d %d\n", registerRead(csPin, 0x13), registerRead(csPin, 0x13), registerRead(csPin, 0x13), registerRead(csPin, 0x13), registerRead(csPin, 0x13));
    }
    vTaskDelay(M2T(1));
  }
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x55, 0x00);
  registerWrite(csPin, 0x7F, 0x13);
  registerWrite(csPin, 0x42, 0x00);
  registerWrite(csPin, 0x7F, 0x00);
  registerWrite(csPin, 0x67, 0xA5);

  // Power on reset
  registerWrite(csPin, 0x3a, 0x5a);
  vTaskDelay(M2T(5));

  // Reading the motion registers one time
  registerRead(csPin, 0x02);
  registerRead(csPin, 0x03);
  registerRead(csPin, 0x04);
  registerRead(csPin, 0x05);
  registerRead(csPin, 0x06);
  vTaskDelay(M2T(1));

  InitRegisters(csPin);
  dt = (float)(usecTimestamp()-lastTime)/1000000.0f;
  lastTime = usecTimestamp();
}



PARAM_GROUP_START(flow)
PARAM_ADD(PARAM_UINT8, resolution, &resolution)
PARAM_GROUP_STOP(flow)

LOG_GROUP_START(flow)
LOG_ADD(LOG_UINT8, mode, &mode)
LOG_GROUP_STOP(flow)

