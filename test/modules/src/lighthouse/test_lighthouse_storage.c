// @IGNORE_IF_NOT CONFIG_DECK_LIGHTHOUSE

// File under test lighthouse_storage.c
#include "lighthouse_storage.h"

#include "unity.h"
#include "mock_system.h"
#include "mock_storage.h"
#include "mock_lighthouse_position_est.h"
#include "mock_lighthouse_calibration.h"
#include "mock_lighthouse_core.h"
#include "mock_pulse_processor.h"
#include "mock_worker.h"

#include <stdbool.h>

// Functions under test
void lighthouseStorageInitializeGeoDataFromStorage();
void lighthouseStorageInitializeCalibDataFromStorage();

pulseProcessor_t lighthouseCoreState;

void setUp(void) {
  // Empty
}

void tearDown(void) {
  // Empty
}


void testThatGeoDataIsWrittenToStorage() {
  // Fixture
  storageStore_ExpectAndReturn("lh/sys/0/geo/1", &lighthouseCoreState.bsGeometry[1], sizeof(baseStationGeometry_t), true);

  // Test
  bool actual = lighthouseStoragePersistData(1, true, false);

  // Actual
  TEST_ASSERT_TRUE(actual);
}

void testThatFailedGeoDataWriteToStorageReturnsFailure() {
  // Fixture
  storageStore_IgnoreAndReturn(false);

  // Test
  bool actual = lighthouseStoragePersistData(1, true, false);

  // Actual
  TEST_ASSERT_FALSE(actual);
}

void testThatCalibDataIsWrittenToStorage() {
  // Fixture
  storageStore_ExpectAndReturn("lh/sys/0/cal/1", &lighthouseCoreState.bsCalibration[1], sizeof(lighthouseCalibration_t), true);

  // Test
  bool actual = lighthouseStoragePersistData(1, false, true);

  // Actual
  TEST_ASSERT_TRUE(actual);
}

void testThatFailedCalibDataWriteToStorageReturnsFailure() {
  // Fixture
  storageStore_IgnoreAndReturn(false);

  // Test
  bool actual = lighthouseStoragePersistData(1, false, true);

  // Actual
  TEST_ASSERT_FALSE(actual);
}

void testThatNoInitializationOfGeoIsDoneWhenStorageIsEmpty() {
  // Fixture
  storageFetch_IgnoreAndReturn(0);

  // Test
  lighthouseStorageInitializeGeoDataFromStorage();

  // Actual
  // Verified in mocks
}

void testInitializationOfGeoIsDoneFromStorage() {
  // Fixture
  int geoSize = sizeof(baseStationGeometry_t);
  const void* ignored = 0;

  for (int i = 0; i < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; i++) {
    storageFetch_ExpectAndReturn("Ignored", ignored, geoSize, geoSize);
    storageFetch_IgnoreArg_key();
    storageFetch_IgnoreArg_buffer();

    lighthousePositionSetGeometryData_Expect(i, ignored);
    lighthousePositionSetGeometryData_IgnoreArg_geometry();
  }

  // Test
  lighthouseStorageInitializeGeoDataFromStorage();

  // Actual
  // Verified in mocks
}

void testThatNoInitializationOfCalibIsDoneWhenStorageIsEmpty() {
  // Fixture
  storageFetch_IgnoreAndReturn(0);

  // Test
  lighthouseStorageInitializeCalibDataFromStorage();

  // Actual
  // Verified in mocks
}

void testInitializationOfCalibIsDoneFromStorage() {
  // Fixture
  int calibSize = sizeof(lighthouseCalibration_t);
  const void* ignored = 0;

  for (int i = 0; i < CONFIG_DECK_LIGHTHOUSE_MAX_N_BS; i++) {
    storageFetch_ExpectAndReturn("Ignored", ignored, calibSize, calibSize);
    storageFetch_IgnoreArg_key();
    storageFetch_IgnoreArg_buffer();

    lighthouseCoreSetCalibrationData_Expect(i, ignored);
    lighthouseCoreSetCalibrationData_IgnoreArg_calibration();
  }

  // Test
  lighthouseStorageInitializeCalibDataFromStorage();

  // Actual
  // Verified in mocks
}
