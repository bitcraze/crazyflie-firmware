/**
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
 * adc.h - Analog Digital Conversion header file
 */
#ifndef ICM20789_H_
#define ICM20789_H_

#include <stdbool.h>
#include <stdint.h>

bool icm20789Init(void);
bool icm20789Test(void);
bool icm20789TestConnection(void);
bool icm20789SelfTest(void);
void icm20789ReadAllSensors(uint8_t *buffer);

bool icm20789BaroInit(I2C_Dev *i2cPort);
bool icm20789BaroStartMeasurement(void);
bool icm20789BaroGetPressureData(float* pressure,float* temperature, float* asf);

#endif /* ICM20789_H_ */
