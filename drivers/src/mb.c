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
 * mb.c - Implementation for the MaxBotix MB1040 Sonar Range Finder (LV-MaxSonar-EZ4)
 */

#include "config.h"
#include "log.h"
#include "mb.h"

#include <stddef.h>
#include <assert.h>

/* Internal tracking of last measured distance. */
static uint32_t mb_distance = 0;
static uint32_t mb_accuracy = 0; /* 0 accuracy means no measurement or unknown accuracy. */

/* Define a log group */
LOG_GROUP_START(mb)
LOG_ADD(LOG_UINT32, distance, &mb_distance)
LOG_ADD(LOG_UINT32, accuracy, &mb_accuracy)
LOG_GROUP_STOP(mb)

/**
 * Gets the accuracy for a distance measurement from an MB1040 sonar range finder (LV-MaxBotix-EZ4).
 *
 * @param distance The distance measurement to report the accuracy for.
 *
 * @return Accuracy in millimeters.
 */
static uint32_t mbGetAccuracyMB1040(uint32_t distance)
{
  /* Specify the accuracy of the measurement from the MB1040 (LV-MaxBotix-EZ4) sensor. */

  if((distance * 25.4) <= 6) {
    /**
     * The datasheet for the MB1040 specifies that any distance below 6 inches is reported as 6 inches.
     * Since all measurements are given in 1 inch steps, the actual distance can be anything
     * from 7 (exclusive) to 0 (inclusive) inches.
     *
     * The accuracy is therefore set to 7(!) inches.
     */
    mb_accuracy = 7 * 25.4;
  }
  else if((distance * 25.4) >= 20) {
    /**
     * The datasheet for the MB1040 specifies that any distance between 6 and 20 inches may result in
     * measurement inaccuracies up to 2 inches.
     */
    mb_accuracy = 2 * 25.4;
  }
  else if((distance * 25.4) > 254) {
    /**
     * The datasheet for the MB1040 specifies that maximum reported distance is 254 inches. If we for
     * some reason should measure more than this, set the accuracy to 0.
     */
    mb_accuracy = 0;
  }
  else {
    /**
     * Otherwise the accuracy is specified by the datasheet for the MB1040 to be 1 inch.
     */
    mb_accuracy = 1 * 25.4;
  }

  /* Report accuracy if the caller asked for this. */
  return mb_accuracy;
}

/**
 * Reads distance measurement from an MB1040 sonar range finder (LV-MaxBotix-EZ4) via an analog input interface.
 *
 * @param pin      The GPIO pin to use for ADC conversion.
 * @param accuracy If not NULL, this function will write the accuracy of the distance measurement to this parameter.
 *
 * @return The distance measurement in millimeters.
 */
static uint32_t mbReadDistanceMB1040AN(uint8_t pin, uint32_t *accuracy)
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
   * TODO: The above conversion assumes the ADC VREF is the same as the LV-MaxSonar-EZ4 VREF. To be checked.
   */

  mb_distance = (uint32_t)(25.4 * analogRead(pin) / 2);

  if(NULL != accuracy) {
    *accuracy = mbGetAccuracyMB1040(mb_distance);
  }
  return mb_distance;
}

/**
 * Reads distance measurement from the specified sensor type.
 *
 * @param type     The MaxBotix sensor type.
 * @param accuracy If not NULL, this function will write the accuracy of the distance measurement to this parameter.
 *
 * @return The distance measurement in millimeters.
 */
uint32_t mbReadDistance(mbSensor_t type, uint32_t *accuracy)
{
  switch(type) {
    case MB1040AN: {
      return mbReadDistanceMB1040AN(MB_DECK_GPIO, accuracy);
    }
  }

  mb_distance = 0;
  mb_accuracy = 0;
  if(accuracy) {
    *accuracy = mb_accuracy;
  }

  return(mb_distance);
}
