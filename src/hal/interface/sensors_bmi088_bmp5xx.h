/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018 Bitcraze AB
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
 */

#ifndef __SENSORS_BMI088_BMP5XX_H__
#define __SENSORS_BMI088_BMP5XX_H__

#include "sensors.h"

void sensorsBmi088Bmp5xxInit_I2C(void);
void sensorsBmi088Bmp5xxInit_SPI(void);
bool sensorsBmi088Bmp5xxTest(void);
bool sensorsBmi088Bmp5xxAreCalibrated(void);
bool sensorsBmi088Bmp5xxManufacturingTest(void);
void sensorsBmi088Bmp5xxAcquire(sensorData_t *sensors);
void sensorsBmi088Bmp5xxWaitDataReady(void);
bool sensorsBmi088Bmp5xxReadGyro(Axis3f *gyro);
bool sensorsBmi088Bmp5xxReadAcc(Axis3f *acc);
bool sensorsBmi088Bmp5xxReadMag(Axis3f *mag);
bool sensorsBmi088Bmp5xxReadBaro(baro_t *baro);
void sensorsBmi088Bmp5xxSetAccMode(accModes accMode);
void sensorsBmi088Bmp5xxDataAvailableCallback(void);

#endif // __SENSORS_BMI088_BMP5XX_H__
