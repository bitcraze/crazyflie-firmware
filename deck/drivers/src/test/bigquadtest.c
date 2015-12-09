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
 * exptest.c - Testing of expansion port.
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

#define VBAT_TEST_VOLTAGE_LOW      (3.0 / ((1.0 + 69.0 + 10.0) / 10.0) * 0.95) /* 0.35625 */
#define VBAT_TEST_VOLTAGE_HIGH     (3.0 / ((1.0 + 69.0 + 10.0) / 10.0) * 1.05) /* 0.39375 */

#define TEST(x, s) if (!(x)) { s = false; }

static bool bigquadtestRun()
{
  bool status = true;
  float readVoltage;
  GpioRegBuf gpioSaved;

  decktestSaveGPIOStatesABC(&gpioSaved);

#ifdef BIGQUAD_TEST_I2C
  /* Test SDA and SCL which should be connected to GND */
  pinMode(DECK_GPIO_SDA, INPUT);
  pinMode(DECK_GPIO_SCL, INPUT);
  decktestEval(digitalRead(DECK_GPIO_SDA) == LOW, "SDA == LOW", &status);
  decktestEval(digitalRead(DECK_GPIO_SCL) == LOW, "SCL == LOW", &status);
#endif

  /* Test M1(TX2), M2(IO_3), M3(IO_2), M4(RX2) */
  /* which should be connected 5V */
  pinMode(DECK_GPIO_TX2, INPUT_PULLDOWN);
  pinMode(DECK_GPIO_RX2, INPUT_PULLDOWN);
  pinMode(DECK_GPIO_IO2, INPUT_PULLDOWN);
  pinMode(DECK_GPIO_IO3, INPUT_PULLDOWN);
  decktestEval(digitalRead(DECK_GPIO_TX2) == HIGH, "M1(TX2) == HIGH", &status);
  decktestEval(digitalRead(DECK_GPIO_IO3) == HIGH, "M2(IO_3) == HIGH", &status);
  decktestEval(digitalRead(DECK_GPIO_IO2) == HIGH, "M3(IO_2) == HIGH", &status);
  decktestEval(digitalRead(DECK_GPIO_RX2) == HIGH, "M4(RX2) == HIGH", &status);

  /* Test TX and RX  which should be bridged */
  pinMode(DECK_GPIO_RX1, INPUT_PULLUP);
  pinMode(DECK_GPIO_TX1, OUTPUT);
  digitalWrite(DECK_GPIO_TX1, LOW);
  vTaskDelay(1);
  decktestEval(digitalRead(DECK_GPIO_RX1) == LOW, "TX, RX", &status);
  digitalWrite(DECK_GPIO_TX1, HIGH);
  vTaskDelay(1);
  decktestEval(digitalRead(DECK_GPIO_RX1) == HIGH, "TX, RX", &status);
  pinMode(DECK_GPIO_TX1, INPUT); // restore

  /* Test CPPM(MOSI) which should be 5V */
  pinMode(DECK_GPIO_MOSI, INPUT_PULLDOWN);
  decktestEval(digitalRead(DECK_GPIO_MOSI) == HIGH, "CPPM(MOSI) == HIGH", &status);

  /* Test CUR(SCK) and VBAT(MISO) which should be bridged */
  /* CUR set to output 3V and VBAT to measure */
  pinMode(DECK_GPIO_SCK, OUTPUT);
  digitalWrite(DECK_GPIO_SCK, HIGH);
  vTaskDelay(10);
  readVoltage = analogReadVoltage(DECK_GPIO_MISO);
  decktestEval((readVoltage > VBAT_TEST_VOLTAGE_LOW &&
                readVoltage < VBAT_TEST_VOLTAGE_HIGH),
                "VBAT(MISO) voltage", &status);

  decktestRestoreGPIOStatesABC(&gpioSaved);

  if (status)
  {
    DEBUG_PRINT("BigQuad deck test [OK]\n");
  }

  return status;
}

static const DeckDriver bigquadtest_deck = {
  .vid = 0xBC,
  .pid = 0xFE,
  .name = "bcBigQuadTest",

  .usedPeriph = 0,
  .usedGpio = 0,

  .test = bigquadtestRun,
};

DECK_DRIVER(bigquadtest_deck);
