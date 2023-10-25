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


TESTABLE_STATIC void generateStorageKey(char* buf, const char* base, const uint8_t baseStation) {
  ASSERT(baseStation < 100);

  const int baseLen = strlen(base);
  memcpy(buf, base, baseLen);
  if (baseStation <= 9) {
    buf[baseLen] = '0' + baseStation;
    buf[baseLen + 1] = '\0';
  } else {
    buf[baseLen] = '0' + baseStation / 10;
    buf[baseLen + 1] = '0' + baseStation % 10;
    buf[baseLen + 2] = '\0';
  }
}

bool lighthouseStoragePersistData(const uint8_t baseStation, const bool geoData, const bool calibData, const LighthouseStorageDef_t* def) {
  bool result = true;
  char key[KEY_LEN];

  if (baseStation < def->nrOfSupportedBs) {
    if (geoData) {
      generateStorageKey(key, STORAGE_KEY_GEO, baseStation);
      result = result && storageStore(key, &def->geometries[baseStation], sizeof(def->geometries[baseStation]));
    }
    if (calibData) {
      generateStorageKey(key, STORAGE_KEY_CALIB, baseStation);
      result = result && storageStore(key, &def->calibrations[baseStation], sizeof(def->calibrations[baseStation]));
    }
  }

  return result;
}

typedef struct {
  uint8_t baseStation;
  const LighthouseStorageDef_t* def;
} WorkerData_t;

static void lhPersistDataWorker(void* arg) {
  WorkerData_t* workerData = (WorkerData_t*)arg;

  const bool storeGeo = false;
  const bool storeCalibration = true;
  if (! lighthouseStoragePersistData(workerData->baseStation, storeGeo, storeCalibration, workerData->def)) {
    DEBUG_PRINT("WARNING: Failed to persist calibration data for base station %i\n", workerData->baseStation + 1);
  }
}

void lighthouseStoragePersistCalibDataBackground(const uint8_t baseStation, const LighthouseStorageDef_t* def) {
  static WorkerData_t workerData;

  if (baseStation < def->nrOfSupportedBs) {
    workerData.baseStation = baseStation;
    workerData.def = def;

    workerSchedule(lhPersistDataWorker, (void*)(&workerData));
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

void lighthouseStorageInitializeGeoDataFromStorage(LighthouseStorageDef_t* def) {
  char key[KEY_LEN];

  for (int baseStation = 0; baseStation < def->nrOfSupportedBs; baseStation++) {
    if (!def->geometries[baseStation].valid) {
      generateStorageKey(key, STORAGE_KEY_GEO, baseStation);
      const size_t geoSize = sizeof(geoBuffer);
      const size_t fetched = storageFetch(key, (void*)&geoBuffer, geoSize);
      if (fetched == geoSize) {
        lighthousePositionSetGeometryData(baseStation, &geoBuffer);
      }
    }
  }
}

void lighthouseStorageInitializeCalibDataFromStorage(LighthouseStorageDef_t* def) {
  char key[KEY_LEN];

  for (int baseStation = 0; baseStation < def->nrOfSupportedBs; baseStation++) {
    if (!def->calibrations[baseStation].valid) {
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
