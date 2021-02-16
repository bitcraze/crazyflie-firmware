/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2021 Bitcraze AB
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

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Copy current data in RAM to permanent storage
 *
 * @param baseStation  The base station id to store data for
 * @param geoData      If true, write geometry data for the base station
 * @param calibData    if true, write calibration data for the base station
 * @return true if data was stored
 */
bool lighthouseStoragePersistData(const uint8_t baseStation, const bool geoData, const bool calibData);
void lighthouseStorageVerifySetStorageVersion();
void lighthouseStorageInitializeGeoDataFromStorage();
void lighthouseStorageInitializeCalibDataFromStorage();
