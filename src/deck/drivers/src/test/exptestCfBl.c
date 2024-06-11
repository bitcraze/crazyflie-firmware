/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2021 Bitcraze AB
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
 * exptest.c - Testing of expansion port.
 */
#define DEBUG_MODULE "ET"

#include <stdint.h>

#include "stm32fxxx.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "deck_test.h"
#include "motors.h"
#include "platform.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "param.h"
#include "sensors.h"

//Hardware configuration
#define ET_GPIO_PERIF   (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC)

#define ET_GPIO_PORT_TX1  GPIOC
#define ET_GPIO_PIN_TX1   GPIO_Pin_10
#define ET_GPIO_PORT_RX1  GPIOC
#define ET_GPIO_PIN_RX1   GPIO_Pin_11

#define ET_GPIO_PORT_SCK  GPIOA
#define ET_GPIO_PIN_SCK   GPIO_Pin_5
#define ET_GPIO_PORT_MOSI GPIOA
#define ET_GPIO_PIN_MOSI  GPIO_Pin_6
#define ET_GPIO_PORT_MISO GPIOA
#define ET_GPIO_PIN_MISO  GPIO_Pin_7

#define ET_GPIO_PORT_SDA  GPIOB
#define ET_GPIO_PIN_SDA   GPIO_Pin_7
#define ET_GPIO_PORT_SCL  GPIOB
#define ET_GPIO_PIN_SCL   GPIO_Pin_6

#define ET_GPIO_PORT_IO1  GPIOB
#define ET_GPIO_PIN_IO1   GPIO_Pin_8
#define ET_GPIO_PORT_IO4  GPIOC
#define ET_GPIO_PIN_IO4   GPIO_Pin_12

#define ET_NBR_PINS         7

#define RPM_TEST_LOWER_LIMIT 12000
#define MOTOR_TEST_PWM (UINT16_MAX/2)
#define MOTOR_TEST_TIME_MILLIS 2000
#define MOTOR_FEED_SIGNAL_INTVL 1
#define MOTOR_RPM_NBR_SAMPLES (MOTOR_TEST_TIME_MILLIS/MOTOR_FEED_SIGNAL_INTVL)

typedef struct _etGpio {
  GPIO_TypeDef     *port;
  uint16_t          pin;
  char              name[6];
} EtGpio;

typedef struct {
  logVarId_t m1;
  logVarId_t m2;
  logVarId_t m3;
  logVarId_t m4;
} motorRpmParams_t;

static EtGpio etGpioIn[ET_NBR_PINS] = {
  {ET_GPIO_PORT_TX1,  ET_GPIO_PIN_TX1, "TX1"},
  {ET_GPIO_PORT_RX1,  ET_GPIO_PIN_RX1, "RX1"},
  {ET_GPIO_PORT_SCK,  ET_GPIO_PIN_SCK, "SCK"},
  {ET_GPIO_PORT_MOSI, ET_GPIO_PIN_MOSI, "MOSI"},
  {ET_GPIO_PORT_MISO, ET_GPIO_PIN_MISO, "MISO"},
  {ET_GPIO_PORT_IO1,  ET_GPIO_PIN_IO1, "IO1"},
  {ET_GPIO_PORT_IO4,  ET_GPIO_PIN_IO4, "IO4"}
};

static EtGpio etGpioSDA = {ET_GPIO_PORT_SDA,  ET_GPIO_PIN_SDA, "SDA"};
static EtGpio etGpioSCL = {ET_GPIO_PORT_SCL,  ET_GPIO_PIN_SCL, "SCL"};

static bool isInit;
const DeckDriver *bcRpm = NULL;
static motorRpmParams_t motorRpm = {0};

static void expCfBlTestInit(DeckInfo *info)
{
  (void)info;
  if (isInit) {
    return;
  }

  // Initialize the VL53L0 sensor using the zRanger deck driver
  bcRpm = deckFindDriverByName("bcRpm");
  motorsInit(platformConfigGetMotorMapping());
  motorRpm.m1 = logGetVarId("rpm", "m1");
  motorRpm.m2 = logGetVarId("rpm", "m2");
  motorRpm.m3 = logGetVarId("rpm", "m3");
  motorRpm.m4 = logGetVarId("rpm", "m4");
  isInit = true;
}

static int getMotorRpm(uint16_t motorIdx)
{
  uint16_t ret = 0xFF;
  switch (motorIdx) {
    case MOTOR_M1:
      ret =  logGetInt(motorRpm.m1);
      break;

    case MOTOR_M2:
      ret =  logGetInt(motorRpm.m2);
      break;

    case MOTOR_M3:
      ret = logGetInt(motorRpm.m3);
      break;

    case MOTOR_M4:
      ret = logGetInt(motorRpm.m4);
      break;

    default:
      break;
    }
  return ret;
}

static void setMotorsPwm(uint32_t pwm)
{
  for (int i = 0; i<NBR_OF_MOTORS; i++) {
    motorsSetRatio(i, pwm);
  }
}

static void runMotors()
{
    #ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
    #endif
    vTaskDelay(MOTOR_FEED_SIGNAL_INTVL);
}



static bool rpmTestRun(void)
{
  bool passed = true;
  uint16_t testTime = MOTOR_TEST_TIME_MILLIS;
  int32_t waitTime = MOTOR_TEST_TIME_MILLIS;
  int32_t rpmSamples[] = {0,0,0,0};
  setMotorsPwm(0);
  while (waitTime) { //We need to wait until all ESCs are started. We need to feed a signal continuosly so they dont go to sleep
    runMotors();
    waitTime -= MOTOR_FEED_SIGNAL_INTVL;
  }

  setMotorsPwm(MOTOR_TEST_PWM);
  while(testTime) {
    runMotors();
    testTime -= MOTOR_FEED_SIGNAL_INTVL;
    for (int i = 0; i<NBR_OF_MOTORS; i++) {
      rpmSamples[i] += getMotorRpm(i);
    }
  }

  setMotorsPwm(0);
  for (int i = 0; i<NBR_OF_MOTORS; i++) {
    int rpmAvg = rpmSamples[i] / MOTOR_RPM_NBR_SAMPLES;
    DEBUG_PRINT("Motor; %d RPM; %d\n", i, rpmAvg);
    passed &= (rpmAvg > RPM_TEST_LOWER_LIMIT);
  }
  return passed;
}

static bool expCfBlTestRun(void)
{
  int i;
  volatile int delay;
  bool status = true;
  GPIO_InitTypeDef GPIO_InitStructure;
  GpioRegBuf gpioSaved;

  isInit = true;

  status &= sensorsManufacturingTest();


  // Enable GPIOs
  RCC_AHB1PeriphClockCmd(ET_GPIO_PERIF, ENABLE);

  decktestSaveGPIOStatesABC(&gpioSaved);

  for (i = 0; i < ET_NBR_PINS; i++) {
    //Initialize the pins as inputs
    GPIO_InitStructure.GPIO_Pin = etGpioIn[i].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(etGpioIn[i].port, &GPIO_InitStructure);
  }

  for (i = 0; i < ET_NBR_PINS && status; i++) {
    // Configure pin as output to poke others
    GPIO_InitStructure.GPIO_Pin = etGpioIn[i].pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_Init(etGpioIn[i].port, &GPIO_InitStructure);

    // Test high
    GPIO_SetBits(etGpioIn[i].port, etGpioIn[i].pin);
    for (delay = 0; delay < 1000; delay++);


    // Test low
    GPIO_ResetBits(etGpioIn[i].port, etGpioIn[i].pin);
    for (delay = 0; delay < 1000; delay++);
  

    // Restore
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(etGpioIn[i].port, &GPIO_InitStructure);
  }

  decktestRestoreGPIOStatesABC(&gpioSaved);

  //Run rpm test
  if(status) {
    if(bcRpm->init) {
    bcRpm->init(NULL);
    }

    rpmTestRun();
  }

  if (status) {
    // Configure SDA & SCL to turn on OK leds
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed;
    GPIO_InitStructure.GPIO_Pin = etGpioSDA.pin;
    GPIO_Init(etGpioSDA.port, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = etGpioSCL.pin;
    GPIO_Init(etGpioSCL.port, &GPIO_InitStructure);
    // Turn on OK LEDs.
    GPIO_ResetBits(etGpioSDA.port, etGpioSDA.pin);
    GPIO_ResetBits(etGpioSCL.port, etGpioSCL.pin);
  }

  return status;
}




static const DeckDriver expCfBltest_deck = {
  .vid = 0xFF,
  .pid = 0xFF,
  .name = "bcExpTestCfBl",

  .usedGpio = 0xFFFFFFFF,

  .init = expCfBlTestInit,
  .test = expCfBlTestRun
};

DECK_DRIVER(expCfBltest_deck);
