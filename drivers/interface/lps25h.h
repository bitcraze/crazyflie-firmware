/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
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
 * @file ms5611.h
 * Driver for the ms5611 pressure sensor from measurement specialties.
 * Datasheet at http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
 *
 */
#ifndef LPS25H_H
#define LPS25H_H

#include <stdbool.h>
#include "i2cdev.h"

/**
 * Initialize the lps25h driver
 * @param i2cPort  I2C port ( a CPAL_InitTypeDef) the lps25h is connected to.
 *
 * @return True on success, else false.
 */
bool lps25hInit(I2C_Dev *i2cPort);

/**
 * Do a self test of the lps25h. Currently it only tests for sane values.
 *
 * @return True on success, else false.
 */
bool lps25hSelfTest(void);

/**
 * Evaluate self test results.
 *
 * @return True on success, else false.
 */
bool lps25hEvaluateSelfTest(float min, float max, float value, char* string);

/**
 * Test the lps25h I2C connection.
 *
 * @return True on success, else false.
 */
bool lps25hTestConnection(void);

/**
 * Enable the lps25h and configure it.
 *
 * @return True on success, else false.
 */
bool lps25hSetEnabled(bool enable);

/**
 * Get measurement data.
 *
 * @return True on success, else false.
 */
bool lps25hGetData(float* pressure, float* temperature, float* asl);

float lps25hPressureToAltitude(float* pressure);

#endif // LPS25H_H
