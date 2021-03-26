/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2018-2021 Bitcraze AB
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

#pragma once

#include "sensors.h"

void sensorsMpu9250Lps25hInit(void);
bool sensorsMpu9250Lps25hTest(void);
bool sensorsMpu9250Lps25hAreCalibrated(void);
bool sensorsMpu9250Lps25hManufacturingTest(void);
void sensorsMpu9250Lps25hAcquire(sensorData_t *sensors, const uint32_t tick);
void sensorsMpu9250Lps25hWaitDataReady(void);
bool sensorsMpu9250Lps25hReadGyro(Axis3f *gyro);
bool sensorsMpu9250Lps25hReadAcc(Axis3f *acc);
bool sensorsMpu9250Lps25hReadMag(Axis3f *mag);
bool sensorsMpu9250Lps25hReadBaro(baro_t *baro);
void sensorsMpu9250Lps25hSetAccMode(accModes accMode);
