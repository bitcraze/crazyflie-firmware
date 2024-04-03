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

#ifndef __SENSORS_BMI088_BMP3XX_H__
#define __SENSORS_BMI088_BMP3XX_H__

#include "sensors.h"

void sensorsBmi088Bmp3xxInit_I2C(void);
void sensorsBmi088Bmp3xxInit_SPI(void);
bool sensorsBmi088Bmp3xxTest(void);
bool sensorsBmi088Bmp3xxAreCalibrated(void);
bool sensorsBmi088Bmp3xxManufacturingTest(void);
void sensorsBmi088Bmp3xxAcquire(sensorData_t *sensors);
void sensorsBmi088Bmp3xxWaitDataReady(void);
bool sensorsBmi088Bmp3xxReadGyro(Axis3f *gyro);
bool sensorsBmi088Bmp3xxReadAcc(Axis3f *acc);
bool sensorsBmi088Bmp3xxReadMag(Axis3f *mag);
bool sensorsBmi088Bmp3xxReadBaro(baro_t *baro);
void sensorsBmi088Bmp3xxSetAccMode(accModes accMode);
void sensorsBmi088Bmp3xxDataAvailableCallback(void);

#endif // __SENSORS_BMI088_BMP3XX_H__
