/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * loadcell.c - Deck driver for NAU7802 load cell
 * See 
 *  * https://learn.sparkfun.com/tutorials/qwiic-scale-hookup-guide
 *  * Code based on https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library
 */

#include <stdint.h>
#include <stdlib.h>
#include "nvicconf.h"
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "deck.h"
#include "param.h"
#include "log.h"
#include "i2cdev.h"
#include "sleepus.h"
#include "debug.h"

#define DECK_I2C_ADDRESS 0x2A

static bool isInit;
static int32_t rawWeight;
static float weight;
static bool enable = true;

static float a = 4.22852802e-05;
static float b = -2.21784688e+01;


// static void loadcellTask(void* prm);

////////////////////////////

//Register Map
typedef enum
{
  NAU7802_PU_CTRL = 0x00,
  NAU7802_CTRL1,
  NAU7802_CTRL2,
  NAU7802_OCAL1_B2,
  NAU7802_OCAL1_B1,
  NAU7802_OCAL1_B0,
  NAU7802_GCAL1_B3,
  NAU7802_GCAL1_B2,
  NAU7802_GCAL1_B1,
  NAU7802_GCAL1_B0,
  NAU7802_OCAL2_B2,
  NAU7802_OCAL2_B1,
  NAU7802_OCAL2_B0,
  NAU7802_GCAL2_B3,
  NAU7802_GCAL2_B2,
  NAU7802_GCAL2_B1,
  NAU7802_GCAL2_B0,
  NAU7802_I2C_CONTROL,
  NAU7802_ADCO_B2,
  NAU7802_ADCO_B1,
  NAU7802_ADCO_B0,
  NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
  NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
  NAU7802_OTP_B0,     //OTP 15:8
  NAU7802_PGA = 0x1B,
  NAU7802_PGA_PWR = 0x1C,
  NAU7802_DEVICE_REV = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
  NAU7802_PU_CTRL_RR = 0,
  NAU7802_PU_CTRL_PUD,
  NAU7802_PU_CTRL_PUA,
  NAU7802_PU_CTRL_PUR,
  NAU7802_PU_CTRL_CS,
  NAU7802_PU_CTRL_CR,
  NAU7802_PU_CTRL_OSCS,
  NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

//Bits within the CTRL1 register
typedef enum
{
  NAU7802_CTRL1_GAIN = 2,
  NAU7802_CTRL1_VLDO = 5,
  NAU7802_CTRL1_DRDY_SEL = 6,
  NAU7802_CTRL1_CRP = 7,
} CTRL1_Bits;

//Bits within the CTRL2 register
typedef enum
{
  NAU7802_CTRL2_CALMOD = 0,
  NAU7802_CTRL2_CALS = 2,
  NAU7802_CTRL2_CAL_ERROR = 3,
  NAU7802_CTRL2_CRS = 4,
  NAU7802_CTRL2_CHS = 7,
} CTRL2_Bits;

//Bits within the PGA register
typedef enum
{
  NAU7802_PGA_CHP_DIS = 0,
  NAU7802_PGA_INV = 3,
  NAU7802_PGA_BYPASS_EN,
  NAU7802_PGA_OUT_EN,
  NAU7802_PGA_LDOMODE,
  NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

//Bits within the PGA PWR register
typedef enum
{
  NAU7802_PGA_PWR_PGA_CURR = 0,
  NAU7802_PGA_PWR_ADC_CURR = 2,
  NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
  NAU7802_PGA_PWR_PGA_CAP_EN = 7,
} PGA_PWR_Bits;

//Allowed Low drop out regulator voltages
typedef enum
{
  NAU7802_LDO_2V4 = 0b111,
  NAU7802_LDO_2V7 = 0b110,
  NAU7802_LDO_3V0 = 0b101,
  NAU7802_LDO_3V3 = 0b100,
  NAU7802_LDO_3V6 = 0b011,
  NAU7802_LDO_3V9 = 0b010,
  NAU7802_LDO_4V2 = 0b001,
  NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
  NAU7802_GAIN_128 = 0b111,
  NAU7802_GAIN_64 = 0b110,
  NAU7802_GAIN_32 = 0b101,
  NAU7802_GAIN_16 = 0b100,
  NAU7802_GAIN_8 = 0b011,
  NAU7802_GAIN_4 = 0b010,
  NAU7802_GAIN_2 = 0b001,
  NAU7802_GAIN_1 = 0b000,
} NAU7802_Gain_Values;

//Allowed samples per second
typedef enum
{
  NAU7802_SPS_320 = 0b111,
  NAU7802_SPS_80 = 0b011,
  NAU7802_SPS_40 = 0b010,
  NAU7802_SPS_20 = 0b001,
  NAU7802_SPS_10 = 0b000,
} NAU7802_SPS_Values;

//Select between channel values
typedef enum
{
  NAU7802_CHANNEL_1 = 0,
  NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

//Calibration state
typedef enum
{
  NAU7802_CAL_SUCCESS = 0,
  NAU7802_CAL_IN_PROGRESS = 1,
  NAU7802_CAL_FAILURE = 2,
} NAU7802_Cal_Status;

bool nau7802_reset()
{
  bool result = true;
  result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR, 1);
  sleepus(1);
  result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR, 0);
  return result;
}

bool nau7802_revision(uint8_t* revision)
{
  return i2cdevReadByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_DEVICE_REV, revision);
}

/////////////////////////////


static void loadcellInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  isInit = true;

  i2cdevInit(I2C1_DEV);

  // isInit &= nau7802_reset();
  uint8_t revision;
  isInit &= nau7802_revision(&revision);
  DEBUG_PRINT("bla %d %d\n", isInit, revision);

  // // Create a task with very high priority to reduce risk that we get
  // // invalid data
  // xTaskCreate(loadcellTask, "LOADCELL",
  //             configMINIMAL_STACK_SIZE, NULL,
  //             /*priority*/6, NULL);

  // isInit = true;
}

// static void loadcellTask(void* prm)
// {
//   TickType_t lastWakeTime = xTaskGetTickCount();

//   while(1) {
//     vTaskDelayUntil(&lastWakeTime, F2T(100));

//     if (enable && hx711_is_ready()) {
//       rawWeight = hx711_read();
//       weight = a * rawWeight + b;
//     }
//   }
// }

static const DeckDriver loadcell_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcLoadcell",

  .init = loadcellInit,
};

DECK_DRIVER(loadcell_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(loadcell)
PARAM_ADD(PARAM_UINT8, enable, &enable)
PARAM_ADD(PARAM_FLOAT, a, &a)
PARAM_ADD(PARAM_FLOAT, b, &b)
PARAM_GROUP_STOP(loadcell)

LOG_GROUP_START(loadcell)
LOG_ADD(LOG_INT32, rawWeight, &rawWeight)
LOG_ADD(LOG_FLOAT, weight, &weight)
LOG_GROUP_STOP(loadcell)
