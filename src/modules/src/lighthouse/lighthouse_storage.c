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
 *
 * lighthouse_storage.c - persistent storage of lighthouse data
 */

#include "storage.h"
#include "lighthouse_storage.h"
#include "lighthouse_state.h"
#include "lighthouse_position_est.h"
#include "lighthouse_core.h"
#include "worker.h"

#include "test_support.h"
#include "cfassert.h"

#define DEBUG_MODULE "LH_STORE"
#include "debug.h"

#include "autoconf.h"

// Persistent storage
#define STORAGE_VERSION_KEY "lh/ver"
#define CURRENT_STORAGE_VERSION "1"
#define STORAGE_KEY_GEO "lh/sys/0/geo/"
#define STORAGE_KEY_CALIB "lh/sys/0/cal/"
#define STORAGE_KEY_SYSTEM_TYPE "lh/sys/0/type"
#define KEY_LEN 20

static baseStationGeometry_t geoBuffer;
static lighthouseCalibration_t calibBuffer;


static void generateStorageKey(char* buf, const char* base, const uint8_t baseStation) {
  // TOOD make an implementation that supports baseStations with 2 digits
  ASSERT(baseStation <= 9);

  const int baseLen = strlen(base);
  memcpy(buf, base, baseLen);
  buf[baseLen] = '0' + baseStation;
  buf[baseLen + 1] = '\0';
}

bool lighthouseStoragePersistData(const uint8_t baseStation, const bool geoData, const bool calibData) {
  bool result = true;
  char key[KEY_LEN];

  if (baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS) {
    if (geoData) {
      generateStorageKey(key, STORAGE_KEY_GEO, baseStation);
      result = result && storageStore(key, &lighthouseCoreState.bsGeometry[baseStation], sizeof(lighthouseCoreState.bsGeometry[baseStation]));
    }
    if (calibData) {
      generateStorageKey(key, STORAGE_KEY_CALIB, baseStation);
      result = result && storageStore(key, &lighthouseCoreState.bsCalibration[baseStation], sizeof(lighthouseCoreState.bsCalibration[baseStation]));
    }
  }

  return result;
}

static void lhPersistDataWorker(void* arg) {
  uint8_t baseStation = (uint32_t)arg;

  const bool storeGeo = false;
  const bool storeCalibration = true;
  if (! lighthouseStoragePersistData(baseStation, storeGeo, storeCalibration)) {
    DEBUG_PRINT("WARNING: Failed to persist calibration data for base station %i\n", baseStation + 1);
  }
}

void lighthouseStoragePersistCalibDataBackground(const uint8_t baseStation) {
  if (baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS) {
    workerSchedule(lhPersistDataWorker, (void*)(uint32_t)baseStation);
  }
}

void lighthouseStoragePersistSystemType(lighthouseBaseStationType_t type) {
  storageStore(STORAGE_KEY_SYSTEM_TYPE, &type, sizeof(type));
}

void lighthouseStorageVerifySetStorageVersion() {
  const int bufLen = 5;
  char buffer[bufLen];

  const size_t fetched = storageFetch(STORAGE_VERSION_KEY, buffer, bufLen);
  if (fetched == 0) {
    storageStore(STORAGE_VERSION_KEY, CURRENT_STORAGE_VERSION, strlen(CURRENT_STORAGE_VERSION) + 1);
  } else {
    if (strcmp(buffer, CURRENT_STORAGE_VERSION) != 0) {
      // The storage format version is wrong! What to do?
      // No need to handle until we bump the storage version, assert for now.
      ASSERT_FAILED();
    }
  }
}

void lighthouseStorageInitializeGeoDataFromStorage() {
  char key[KEY_LEN];

  for (int baseStation = 0; baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; baseStation++) {
    if (!lighthouseCoreState.bsGeometry[baseStation].valid) {
      generateStorageKey(key, STORAGE_KEY_GEO, baseStation);
      const size_t geoSize = sizeof(geoBuffer);
      const size_t fetched = storageFetch(key, (void*)&geoBuffer, geoSize);
      if (fetched == geoSize) {
        lighthousePositionSetGeometryData(baseStation, &geoBuffer);
      }
    }
  }
}

void lighthouseStorageInitializeCalibDataFromStorage() {
  char key[KEY_LEN];

  for (int baseStation = 0; baseStation < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; baseStation++) {
    if (!lighthouseCoreState.bsCalibration[baseStation].valid) {
      generateStorageKey(key, STORAGE_KEY_CALIB, baseStation);
      const size_t calibSize = sizeof(calibBuffer);
      const size_t fetched = storageFetch(key, (void*)&calibBuffer, calibSize);
      if (fetched == calibSize) {
        lighthouseCoreSetCalibrationData(baseStation, &calibBuffer);
      }
    }
  }
}

void lighthouseStorageInitializeSystemTypeFromStorage() {
  lighthouseBaseStationType_t type;
  const size_t typeSize = sizeof(lighthouseBaseStationType_t);
  const size_t fetched = storageFetch(STORAGE_KEY_SYSTEM_TYPE, &type, sizeof(type));

  if (fetched == typeSize) {
    lighthouseCoreSetSystemType(type);
  } 
}
