/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2024 Bitcraze AB
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
 * radiotest.c - Use this deck driver to test the radio
 */
#define DEBUG_MODULE "RADIOTEST"

#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32fxxx.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "radiolink.h"
#include "syslink.h"
#include "param.h"
#include "platform_defaults.h"


#define SHOULD_SPIN_MOTORS 0

#ifdef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#define MOTOR_SPEED CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#else
#define MOTOR_SPEED  7000
#endif


//Hardware configuration
static bool isInit;
static uint8_t channel = 80;
static uint8_t datarate = RADIO_RATE_2M;
static int8_t power = -16;
static uint8_t contwave = 0;
static uint8_t old_channel;
static uint8_t old_datarate;
static int8_t old_power;
static uint8_t old_contwave;

static void spinMotorsTask(void *param)
{
  vTaskDelay(M2T(4000)); // Wait for the ESCs to be ready to recieve signals

  paramVarId_t motorPowerSetEnableParam = paramGetVarId("motorPowerSet", "enable");
  paramVarId_t motorParams = paramGetVarId("motorPowerSet", "m1");
  paramSetInt(motorPowerSetEnableParam, 2);
  paramSetInt(motorParams, MOTOR_SPEED);

  vTaskDelete(NULL);
}

static void radiotestTask(void *param)
{
  SyslinkPacket slp;
  bool configSent = false;

  old_channel = channel;
  old_datarate = datarate;
  old_power = power;
  old_contwave = contwave;

  while (1)
  {
    vTaskDelay(M2T(1000));

    if (!configSent || channel != old_channel)
    {
      radiolinkSetChannel(channel);
      old_channel = channel;
    }
    if (!configSent || datarate != old_datarate)
    {
      radiolinkSetDatarate(datarate);
      old_datarate = datarate;
    }
    if (!configSent || power != old_power)
    {
      radiolinkSetPowerDbm(power);
      old_power = power;
    }
    configSent = true;

    if (contwave != old_contwave)
    {
      slp.type = SYSLINK_RADIO_TEST;
      slp.length = 1;
      slp.data[0] = contwave;
      syslinkSendPacket(&slp);
      old_contwave = contwave;
    }
  }
}


static void radiotestInit(DeckInfo *info)
{
  if(isInit)
    return;

  xTaskCreate(radiotestTask, "RADIOTEST", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  if (SHOULD_SPIN_MOTORS) {
    xTaskCreate(spinMotorsTask, "spinMotors", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  }
  isInit = true;
}

static bool radiotestTest()
{
  bool status = true;

  if(!isInit)
    return false;

  return status;
}

static const DeckDriver radiotest_deck = {
  .vid = 0,
  .pid = 0,
  .name = "bcRadioTest",

  .init = radiotestInit,
  .test = radiotestTest,
};

DECK_DRIVER(radiotest_deck);

/**
 * Radio test configuration for lab use.
 *
 * The bcRadioTest deck driver must be initialized for changes to be sent to
 * the nRF51. Configure the channel, data rate and power before enabling a test
 * mode. Use USB to retain control while a test is active.
 */
PARAM_GROUP_START(radiotest)

/**
 * @brief Radio channel in the range 0 to 125. Default is 80.
 */
PARAM_ADD(PARAM_UINT8, channel, &channel)

/**
 * @brief Radio data rate. 0=250 Kbit/s, 1=1 Mbit/s, 2=2 Mbit/s (default), 3=BLE 1M PHY (test only).
 */
PARAM_ADD(PARAM_UINT8, datarate, &datarate)

/**
 * @brief Radio transmit power in dBm. Default is -16.
 */
PARAM_ADD(PARAM_INT8,  power, &power)

/**
 * @brief Radio test mode. 0=off, 1=unmodulated carrier, 2=modulated/whitened carrier.
 *
 * Only change this parameter over USB. Starting a test disables radio and BLE
 * communication. Mode 0 returns to ESB radio operation, but the nRF51 must be
 * rebooted to resume BLE advertising.
 */
PARAM_ADD(PARAM_UINT8, contwave, &contwave)

PARAM_GROUP_STOP(radiotest)
