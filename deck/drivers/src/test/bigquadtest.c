/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * bigquadtest.c - Testing of the Big Quad deck.
 */
#define DEBUG_MODULE "BQ-TEST"


#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32fxxx.h"
#include "config.h"
#include "debug.h"
#include "deck.h"
#include "deck_test.h"
#include "param.h"

#define VBAT_TEST_VOLTAGE_LOW      (3.0 / ((1.0 + 69.0 + 10.0) / 10.0) * 0.95) /* 0.35625 */
#define VBAT_TEST_VOLTAGE_HIGH     (3.0 / ((1.0 + 69.0 + 10.0) / 10.0) * 1.05) /* 0.39375 */

#define TEST(result, str, status) decktestEval(result, DEBUG_MODULE ": " str, status)

static uint8_t testsPass = 0;

static bool bigquadtestRun()
{
  bool status = true;
  float readVoltage;
  GpioRegBuf gpioSaved;
  UBaseType_t prioSaved;

  /* Raise priority to prevent task switch during testing */
  prioSaved = uxTaskPriorityGet(NULL);
  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);

  /* Save GPIO states */
  decktestSaveGPIOStatesABC(&gpioSaved);

  /* Test SDA and CPPM(MOSI) which should be bridged */
  pinMode(DECK_GPIO_SDA, INPUT);
  pinMode(DECK_GPIO_MOSI, INPUT);
  TEST(digitalRead(DECK_GPIO_SDA) == HIGH, "SDA == HIGH", &status);
  TEST(digitalRead(DECK_GPIO_MOSI) == HIGH, "CPPM == HIGH", &status);
  pinMode(DECK_GPIO_MOSI, OUTPUT);
  digitalWrite(DECK_GPIO_MOSI, LOW);
  vTaskDelay(1);
  TEST(digitalRead(DECK_GPIO_SDA) == LOW, "SDA == LOW", &status);
  pinMode(DECK_GPIO_MOSI, INPUT);

  /* Test SCL and BUZ(IO_4) which should be bridged */
  pinMode(DECK_GPIO_SCL, INPUT);
  pinMode(DECK_GPIO_IO4, INPUT);
  TEST(digitalRead(DECK_GPIO_SCL) == HIGH, "SCL == HIGH", &status);
  TEST(digitalRead(DECK_GPIO_IO4) == HIGH, "BUZ == HIGH", &status);
  pinMode(DECK_GPIO_IO4, OUTPUT);
  digitalWrite(DECK_GPIO_IO4, LOW);
  vTaskDelay(1);
  TEST(digitalRead(DECK_GPIO_SCL) == LOW, "SCL == LOW", &status);
  pinMode(DECK_GPIO_IO4, INPUT);

  /* Test M1(TX2), M2(IO_3), M3(IO_2), M4(RX2) */
  /* which are pulled high with 1k */
  pinMode(DECK_GPIO_TX2, INPUT_PULLDOWN);
  pinMode(DECK_GPIO_RX2, INPUT_PULLDOWN);
  pinMode(DECK_GPIO_IO2, INPUT_PULLDOWN);
  pinMode(DECK_GPIO_IO3, INPUT_PULLDOWN);
  TEST(digitalRead(DECK_GPIO_TX2) == HIGH, "M1(TX2) == HIGH", &status);
  TEST(digitalRead(DECK_GPIO_IO3) == HIGH, "M2(IO_3) == HIGH", &status);
  TEST(digitalRead(DECK_GPIO_IO2) == HIGH, "M3(IO_2) == HIGH", &status);
  TEST(digitalRead(DECK_GPIO_RX2) == HIGH, "M4(RX2) == HIGH", &status);

  /* Test TX and RX  which should be bridged */
  pinMode(DECK_GPIO_RX1, INPUT_PULLUP);
  pinMode(DECK_GPIO_TX1, OUTPUT);
  digitalWrite(DECK_GPIO_TX1, LOW);
  vTaskDelay(1);
  TEST(digitalRead(DECK_GPIO_RX1) == LOW, "TX, RX", &status);
  digitalWrite(DECK_GPIO_TX1, HIGH);
  vTaskDelay(1);
  TEST(digitalRead(DECK_GPIO_RX1) == HIGH, "TX, RX", &status);
  pinMode(DECK_GPIO_TX1, INPUT); // restore

  /* Test CUR(SCK) and VBAT(MISO) which should be bridged */
  /* CUR set to output 3V and VBAT to measure */
  pinMode(DECK_GPIO_SCK, OUTPUT);
  digitalWrite(DECK_GPIO_SCK, HIGH);
  vTaskDelay(10);
  readVoltage = analogReadVoltage(DECK_GPIO_MISO);
  TEST((readVoltage > VBAT_TEST_VOLTAGE_LOW &&
        readVoltage < VBAT_TEST_VOLTAGE_HIGH),
        "VBAT(MISO) voltage", &status);

  decktestRestoreGPIOStatesABC(&gpioSaved);

  if (status)
  {
    testsPass = 1;
    DEBUG_PRINT("BigQuad deck test [OK]\n");
  }

  /* Restore priority */
  vTaskPrioritySet(NULL, prioSaved);

  return status;
}

static const DeckDriver bigquadtest_deck = {
  .name = "bcBigQuadTest",

  .usedPeriph = 0,
  .usedGpio = 0,

  .test = bigquadtestRun,
};

DECK_DRIVER(bigquadtest_deck);

PARAM_GROUP_START(BigQuadTest)
PARAM_ADD(PARAM_UINT8, pass, &testsPass)
PARAM_GROUP_STOP(BigQuadTest)

