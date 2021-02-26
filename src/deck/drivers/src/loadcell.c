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
 * loadcell.c - Deck driver for HX711 load cell
 * See 
 *  * https://learn.sparkfun.com/tutorials/load-cell-amplifier-hx711-breakout-hookup-guide
 *  * Code based on https://github.com/bogde/HX711
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "deck.h"
#include "param.h"
#include "log.h"
#include "sleepus.h"

// Hardware defines (also update deck driver below!)
#define CLK_PIN    DECK_GPIO_IO1
#define DAT_PIN    DECK_GPIO_IO4

static bool isInit;
static int32_t rawWeight;
static float weight;
static bool enable = true;

static float a = 4.22852802e-05;
static float b = -2.21784688e+01;

static void loadcellTask(void* prm);

static enum hx711_gain
{
  GAIN128 = 1,
  GAIN64 = 3,
  GAIN32 = 2,
} gain;

static void hx711_init(void);
static bool hx711_is_ready(void);
// static void hx711_set_gain(void);
static int32_t hx711_read(void);
// static void hx711_power_down(void);
static void hx711_power_up(void);

static void hx711_init(void)
{
  pinMode(CLK_PIN, OUTPUT);
  pinMode(DAT_PIN, INPUT);
  // hx711_set_gain();
  hx711_power_up();
}

static bool hx711_is_ready(void)
{
  return digitalRead(DAT_PIN) == LOW;
}

// static void hx711_set_gain(void)
// {
//   digitalWrite(CLK_PIN, LOW);
//   hx711_read();
// }

static int32_t hx711_read(void)
{
  // Wait for the chip to become ready.
  // while (!hx711_is_ready());

  // if (!hx711_is_ready()) {
  //   return 0;
  // }

  // Define structures for reading data into.
  int32_t value = 0;

  // Protect the read sequence from system interrupts.  If an interrupt occurs during
  // the time the PD_SCK signal is high it will stretch the length of the clock pulse.
  // If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
  // power down mode during the middle of the read sequence.  While the device will
  // wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
  // forces DOUT high until that cycle is completed.  
  //
  // The result is that all subsequent bits read by shiftIn() will read back as 1,
  // corrupting the value returned by read().

  // Since we use a high-priority task, we do not need to actually disable the interrupt

  // portDISABLE_INTERRUPTS();

  // Pulse the clock pin 24 times to read the data.
  for (int i = 0; i < 24; ++i) {
    digitalWrite(CLK_PIN, HIGH);
    value |= digitalRead(DAT_PIN) << (24 - i);
    digitalWrite(CLK_PIN, LOW);
    sleepus(1);
  }

  // Set the channel and the gain factor for the next reading using the clock pin.
  for (int i = 0; i < gain; ++i) {
    digitalWrite(CLK_PIN, HIGH);
    sleepus(1);
    digitalWrite(CLK_PIN, LOW);
    sleepus(1);
  }

  // portENABLE_INTERRUPTS();

  // Replicate the most significant bit to pad out a 32-bit signed integer
  if (value & (1<<24)) {
    value = 0xFF000000 | value;
  }
  return value;
}

// static void hx711_power_down(void)
// {
//   digitalWrite(CLK_PIN, LOW);
//   digitalWrite(CLK_PIN, HIGH);
// }

static void hx711_power_up(void)
{
  digitalWrite(CLK_PIN, LOW);
}

static void loadcellInit(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  gain = GAIN128;
  hx711_init();

  // Create a task with very high priority to reduce risk that we get
  // invalid data
  xTaskCreate(loadcellTask, "LOADCELL",
              configMINIMAL_STACK_SIZE, NULL,
              /*priority*/6, NULL);

  isInit = true;
}

static void loadcellTask(void* prm)
{
  TickType_t lastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(100));

    if (enable && hx711_is_ready()) {
      rawWeight = hx711_read();
      weight = a * rawWeight + b;
    }
  }
}

static const DeckDriver loadcell_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcLoadcell",

  .usedGpio = DECK_USING_IO_1 | DECK_USING_IO_4,

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
