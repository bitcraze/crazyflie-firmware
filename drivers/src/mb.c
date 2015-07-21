/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 Bitcraze AB
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
 * mb.c - Implementation for the MaxBotix MB1040 Sonar Range Finder (LV-MaxSonar-EZ04)
 */

#include "deck.h"
#include "log.h"

/* Internal tracking of last measured distance. */
static uint32_t mb_distance = 0;

LOG_GROUP_START(mb)
LOG_ADD(LOG_UINT32, distance, &mb_distance)
LOG_GROUP_STOP(mb)

uint32_t mb_read_distance(uint8_t pin)
{
  /*
   * analogRead() returns a 10-bit (0-1023) value scaled to the range between GND (0V) and VREF.
   * The voltage conversion is: V = analogRead() / 1024 * VREF)
   *
   * The MB1040 sensor returns a voltage between GND and VREF, but scaled to VREF / 512 (volts-per-inch).
   * Inches-per-volt is therefore expressed by (512 / VREF).
   *
   * The distance conversion is:             D = (512 / VREF) * V
   * Expanding V, we get:                    D = (512 / VREF) * (analogRead() / 1024 * VREF)
   * Which can be simplified to:             D = analogRead() / 2
   * Last, we convert inches to millimeters: D = 25.4 * analogRead() / 2
   *
   * TODO: The above conversion assumes the ADC VREF is the same as the LV-MaxSonar-EZ04 VCC. To be checked.
   */

  mb_distance = (uint32_t)(25.4 * analogRead(pin) / 2);
  return mb_distance;
}
