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
 * maxSonar.h - API for the MaxSonar MB1040 (LV-MaxSonar-EZ04)
 */

/**
 * This driver assumes the ADC VREF is the same as the LV-MaxSonar-EZ4 VREF. This means
 * that the MB1040 Sensor should have its VCC pin connected to the VCC pin on the deck
 * port (instead of using the VCOM or VUSB pins).
 *
 * According to the datasheet for the MB1040, the sensor draws typically 2mA, so
 * powering it with the VCC pin (50mA max) on the deck port should be safe.
 *
 * See also https://forum.bitcraze.io/viewtopic.php?f=6&t=1250
 */

#ifndef __MAXSONAR_H__
#define __MAXSONAR_H__

#include <stdint.h>

/**
 * \def MAXSONAR_ENABLED
 * Enable the MaxSonar driver (used by the proximity measurement subsystem).
 */
//#define MAXSONAR_ENABLED

/**
 * \def MAXSONAR_DECK_GPIO
 * The GPIO pin to use if reading via the analog interface of a MaxSonar sensor.
 */
#define MAXSONAR_DECK_GPIO DECK_GPIO_TX2

/**
 * \def MAXSONAR_LOG_ENABLED
 * Uncomment to enable log variables for this driver.
 */
//#define MAXSONAR_LOG_ENABLED

/**
 * List of MaxBotix sensors with different interface types can be added here.
 *
 * Sensors should be listed once for each interface, for instance MB1040AN (Analog), MB1040I2C (I2C), MB1040PWM (PWM) etc.
 */
typedef enum {
  MAXSONAR_MB1040_AN = 0, /* The MB1040 (LV-MaxSonar-EZ4) sensor read by analog conversion (GPIO pin read by ADC). */
} maxSonarSensor_t;

uint32_t maxSonarReadDistance(maxSonarSensor_t type, uint32_t *accuracy);

#endif
