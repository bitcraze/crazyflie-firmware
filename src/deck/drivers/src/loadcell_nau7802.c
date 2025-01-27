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
#include "statsCnt.h"

// Hardware defines (also update deck driver below!)
#define DECK_I2C_ADDRESS 0x2A
#define DATA_READY_PIN DECK_GPIO_IO1

static bool isInit;
static int32_t rawWeight;
static float weight;

static float a[2];
static float b[2];
// static uint8_t currentChannel = 0;

static uint8_t sampleRateDesired = 0;
static uint8_t sampleRate = 0;
static uint8_t channelDesired = 0;
static uint8_t channel = 0;

static STATS_CNT_RATE_DEFINE(rate, 1000);

static xSemaphoreHandle dataReady;
static StaticSemaphore_t dataReadyBuffer;

static void loadcellTask(void* prm);

////////////////////////////

typedef struct
{
  uint8_t pu_ctrl;
  uint8_t ctrl1;
  uint8_t ctrl2;
} nau7802_t;
static nau7802_t nau7802;

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

void nau7802_init(nau7802_t* ctx)
{
  ctx->pu_ctrl = 0;
  ctx->ctrl1 = 0;
  ctx->ctrl2 = 0;
}

// //Resets all registers to Power Of Defaults
// bool nau7802_reset(nau7802_t *ctx)
// {
//   bool result = true;
//   result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR, 1);
//   sleepus(1);
//   result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_RR, 0);
//   return result;
// }

//Power up digital and analog sections of scale
bool nau7802_powerUp(nau7802_t *ctx)
{
  bool result = true;

  ctx->pu_ctrl = 0x06; //(PU analog and PU digital)
  result &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, ctx->pu_ctrl);
  DEBUG_PRINT("pu %d\n", result);

//  result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUD, 1);
//  result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUA, 1);

  //Wait for Power Up bit to be set - takes approximately 200us
  for (int i = 0; i < 200; ++i) {
    uint8_t bit;
    result &= i2cdevReadBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_PUR, &bit);
    if (result && bit) {
      return true;
    }
    sleepus(1);
  }
  return false;
}

//Set the onboard Low-Drop-Out voltage regulator to a given value
bool nau7802_setLDO(nau7802_t *ctx, NAU7802_LDO_Values ldoValue)
{
  bool result = true;
  //Set the value of the LDO
  ctx->ctrl1 &= 0b11000111;    //Clear LDO bits
  ctx->ctrl1 |= ldoValue << 3; //Mask in new LDO bits
  result &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL1, ctx->ctrl1);
  //Enable the internal LDO
  DEBUG_PRINT("l1 %d\n", ctx->pu_ctrl);
  ctx->pu_ctrl |= (1 << NAU7802_PU_CTRL_AVDDS);
  result &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, ctx->pu_ctrl);
  DEBUG_PRINT("l2 %d\n", ctx->pu_ctrl);
  return result;
}

//Set the gain
bool nau7802_setGain(nau7802_t *ctx, NAU7802_Gain_Values gainValue)
{
  bool result = true;
  ctx->ctrl1 &= 0b11111000; //Clear gain bits
  ctx->ctrl1 |= gainValue;  //Mask in new bits
  result &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL1, ctx->ctrl1);
  return result;
}

//Set the readings per second
bool nau7802_setSampleRate(nau7802_t* ctx, NAU7802_SPS_Values rate)
{
  bool result = true;
  ctx->ctrl2 &= 0b10001111; // Clear CRS bits
  ctx->ctrl2 |= rate << 4;  //Mask in new CRS bits
  result &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL2, ctx->ctrl2);
  return result;
}

//Set channel
bool nau7802_setChannel(nau7802_t *ctx, NAU7802_Channels channel)
{
  bool result = true;
  ctx->ctrl2 &= 0b01111111; // Clear CHS bits
  ctx->ctrl2 |= channel << 7;  //Mask in new CHS bits
  result &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL2, ctx->ctrl2);
  return result;
}

//Check calibration status.
NAU7802_Cal_Status nau7802_calAFEStatus(nau7802_t *ctx)
{
  bool result = true;
  uint8_t bit;
  result &= i2cdevReadBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL2, NAU7802_CTRL2_CALS, &bit);
  if (result && bit) {
    return NAU7802_CAL_IN_PROGRESS;
  }
  result &= i2cdevReadBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL2, NAU7802_CTRL2_CAL_ERROR, &bit);
  if (result && bit) {
    return NAU7802_CAL_FAILURE;
  }

  if (result) {
    // Calibration passed
    return NAU7802_CAL_SUCCESS;
  }
  return NAU7802_CAL_FAILURE;
}

//Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
//Takes approximately 344ms to calibrate; wait up to 1000ms.
//It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
bool nau7802_calibrateAFE(nau7802_t *ctx)
{
  bool result = true;
  //Begin asynchronous calibration of the analog front end.
  result &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL2, NAU7802_CTRL2_CALS, 1);

  for (int i = 0; i < 1000; ++i) {
    if (nau7802_calAFEStatus(ctx) == NAU7802_CAL_SUCCESS) {
      return true;
    }
    vTaskDelay(M2T(1));
  }
  return false;
}

//Returns true if Cycle Ready bit is set (conversion is complete)
bool nau7802_hasMeasurement(nau7802_t *ctx)
{
  bool result = true;
  uint8_t bit;
  result &= i2cdevReadBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, NAU7802_PU_CTRL_CR, &bit);
  return result && bit;
}

//Returns 24-bit reading
//Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
bool nau7802_getMeasurement(nau7802_t *ctx, int32_t *measurement)
{
  bool result = true;

  uint8_t data[3];
  result &= i2cdevReadReg8(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_ADCO_B2, 3, (uint8_t*)&data);
  // DEBUG_PRINT("M: %d %d, %d, %d\n", result, data[0], data[1], data[2]);

  int32_t valueRaw = ((int32_t)data[0] << 16) |
                      ((int32_t)data[1] << 8) |
                      ((int32_t)data[2] << 0);
  // recover the sign
  int32_t valueShifted = (valueRaw << 8);
  int32_t value = (valueShifted >> 8);

  *measurement = value;

  return result;
}

bool nau7802_revision(nau7802_t *ctx, uint8_t *revision)
{
  bool result = true;
  result &= i2cdevReadByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_DEVICE_REV, revision);
  *revision = *revision & 0x0F;
  return result;
}

/////////////////////////////

static void setupDataReadyInterrupt(void)
{
  pinMode(DATA_READY_PIN, INPUT);

  dataReady = xSemaphoreCreateBinaryStatic(&dataReadyBuffer);

  // TODO: EXTI_PortSourceGPIOB and EXTI_PinSource8 should be part of deckGPIOMapping!
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource8);

  EXTI_InitTypeDef EXTI_InitStructure;
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  // portDISABLE_INTERRUPTS();
  EXTI_Init(&EXTI_InitStructure);
  // EXTI_ClearITPendingBit(EXTI_Line15);
  // portENABLE_INTERRUPTS();
}

void __attribute__((used)) EXTI8_Callback(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(dataReady, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD();
  }
}

static void loadcellInit(DeckInfo *info)
{
  // if (isInit) {
    // return;
  // }

  isInit = true;

  // sleepus(1000 * 1000);
  // isInit &= nau7802_reset();

  if (true) {

    nau7802_init(&nau7802);
    // isInit &= nau7802_reset();
    // DEBUG_PRINT("Reset [%d]\n", isInit);

    isInit &= nau7802_powerUp(&nau7802);
    DEBUG_PRINT("Power up [%d]\n", isInit);

    uint8_t revision;
    isInit &= nau7802_revision(&nau7802, & revision);
    DEBUG_PRINT("revision [%d]: %d\n", isInit, revision);

    isInit &= nau7802_setLDO(&nau7802, NAU7802_LDO_2V7);
    DEBUG_PRINT("LDO [%d]\n", isInit);
    isInit &= nau7802_setGain(&nau7802, NAU7802_GAIN_128);
    DEBUG_PRINT("Gain [%d]\n", isInit);
    isInit &= nau7802_setSampleRate(&nau7802, NAU7802_SPS_10);
    DEBUG_PRINT("Sample rate [%d]\n", isInit);
    // Turn off CLK_CHP. From 9.1 power on sequencing.
    isInit &= i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_ADC, 0x30);
    DEBUG_PRINT("CLK_CHP [%d]\n", isInit);
    // Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.
    isInit &= i2cdevWriteBit(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PGA_PWR, NAU7802_PGA_PWR_PGA_CAP_EN, 1);
    DEBUG_PRINT("CAP [%d]\n", isInit);
    // calibrate
    isInit &= nau7802_calibrateAFE(&nau7802);
    DEBUG_PRINT("Cal [%d]\n", isInit);

    uint8_t v;
    isInit &= i2cdevReadByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, &v);
    DEBUG_PRINT("puctrl: %d\n", v);
    isInit &= i2cdevReadByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL1, &v);
    DEBUG_PRINT("ctrl1: %d\n", v);
    isInit &= i2cdevReadByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_CTRL2, &v);
    DEBUG_PRINT("ctrl2: %d\n", v);
  }

  // Set up Interrupt
  setupDataReadyInterrupt();

  if (1) {
    // Create a task
    xTaskCreate(loadcellTask, "LOADCELL",
                configMINIMAL_STACK_SIZE, NULL,
                /*priority*/1, NULL);
  }
}

static void loadcellTask(void* prm)
{
  while (1) {
    BaseType_t semResult = xSemaphoreTake(dataReady, M2T(500));
    if (semResult == pdTRUE || digitalRead(DATA_READY_PIN))
    {
      int32_t measurement;
      bool result = nau7802_getMeasurement(&nau7802, &measurement);
      if (result) {
        rawWeight = measurement;
        weight = a[channel] * rawWeight + b[channel];
        // currentChannel = (currentChannel + 1) % 2;
        // nau7802_setChannel(&nau7802, currentChannel);

        // nau7802.pu_ctrl |= (1 << NAU7802_PU_CTRL_CS);
        // i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, nau7802.pu_ctrl);

        // nau7802.ctrl2 &= 0b11101111; // Clear CS bit
        // i2cdevWriteByte(I2C1_DEV, DECK_I2C_ADDRESS, NAU7802_PU_CTRL, nau7802.pu_ctrl);

        STATS_CNT_RATE_EVENT(&rate);
      }
    }
    if (sampleRate != sampleRateDesired) {
      switch (sampleRateDesired) {
        case NAU7802_SPS_320:
        case NAU7802_SPS_80:
        case NAU7802_SPS_40:
        case NAU7802_SPS_20:
        case NAU7802_SPS_10:
          {
            bool result = nau7802_setSampleRate(&nau7802, sampleRateDesired);
            if (result) {
              DEBUG_PRINT("switched sample rate!\n");
            }
          }
          break;
        default:
          DEBUG_PRINT("Unknown sample rate!\n");
          break;
      }
      sampleRate = sampleRateDesired;
    }
    if (channel != channelDesired) {
      switch (channelDesired) {
        case NAU7802_CHANNEL_1:
        case NAU7802_CHANNEL_2:
          {
            bool result = nau7802_setChannel(&nau7802, channelDesired);
            result &= nau7802_calibrateAFE(&nau7802);
            if (result)
            {
              DEBUG_PRINT("switched and calibrated channel!\n");
            }
          }
          break;
        default:
          DEBUG_PRINT("Unknown channel!\n");
          break;
      }
      channel = channelDesired;
    }


  }
}

static const DeckDriver loadcell_deck = {
    .vid = 0x00,
    .pid = 0x00,
    .name = "bcLoadcell",

    .usedGpio = DECK_USING_IO_1,

    .init = loadcellInit,
};

DECK_DRIVER(loadcell_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcLoadcell, &isInit)
PARAM_GROUP_STOP(deck)

PARAM_GROUP_START(loadcell)
PARAM_ADD(PARAM_FLOAT, a, &a[0])
PARAM_ADD(PARAM_FLOAT, b, &b[0])
PARAM_ADD(PARAM_UINT8, sampleRate, &sampleRateDesired)
PARAM_ADD(PARAM_UINT8, channel, &channelDesired)
PARAM_GROUP_STOP(loadcell)

LOG_GROUP_START(loadcell)
LOG_ADD(LOG_INT32, rawWeight, &rawWeight)
LOG_ADD(LOG_FLOAT, weight, &weight)

STATS_CNT_RATE_LOG_ADD(rate, &rate)
LOG_GROUP_STOP(loadcell)
