/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 - 2020 Bitcraze AB
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
 * storage.c: Key/Buffer persistent storage
 *
 */

#include "storage.h"
#include "param.h"

#include "kve/kve.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "i2cdev.h"
#include "eeprom.h"

#include <string.h>

#define TRACE_MEMORY_ACCESS 0

#if !TRACE_MEMORY_ACCESS
#define DEBUG_MODULE "STORAGE"
#endif
#include "debug.h"

// Memory organization

// Low level memory access

// ToDo: Shall we handle partition elsewhere?
#define KVE_PARTITION_START (1024)
#define KVE_PARTITION_LENGTH (7*1024)

#define DEFAULT_DEFRAG_ON_STARTUP true

#ifdef CONFIG_DEFRAG_STORAGE_ON_STARTUP
#define DEFRAG_ON_STARTUP CONFIG_DEFRAG_STORAGE_ON_STARTUP
#else
#define DEFRAG_ON_STARTUP DEFAULT_DEFRAG_ON_STARTUP
#endif

static SemaphoreHandle_t storageMutex;

static size_t readEeprom(size_t address, void* data, size_t length)
{
  if (length == 0) {
    return 0;
  }

  bool success = eepromReadBuffer(data, KVE_PARTITION_START + address, length);

#if TRACE_MEMORY_ACCESS
  DEBUG_PRINT("R %s @%04x l%d: ", success?" OK ":"FAIL", address, length);

  for (int i=0; i < length; i++) {
    DEBUG_PRINT("%02x ", ((char*)data)[i]);
  }

  DEBUG_PRINT("\n");
#endif

  if (success) {
    return length;
  } else {
    return 0;
  }
}

static size_t writeEeprom(size_t address, const void* data, size_t length)
{
  if (length == 0) {
    return 0;
  }

  bool success = eepromWriteBuffer(data, KVE_PARTITION_START + address, length);

#if TRACE_MEMORY_ACCESS
  DEBUG_PRINT("W %s @%04x: ", success?" OK ":"FAIL", address);

  for (int i=0; i < length; i++) {
    DEBUG_PRINT("%02x ", ((char*)data)[i]);
  }

  DEBUG_PRINT("\n");
#endif

  if (success) {
    return length;
  } else {
    return 0;
  }
}

static void flushEeprom(void)
{
  // NOP for now, lets fix the EEPROM write first!
}

static kveMemory_t kve = {
  .memorySize = KVE_PARTITION_LENGTH,
  .read = readEeprom,
  .write = writeEeprom,
  .flush = flushEeprom,
};

// Public API

static bool isInit = false;

void storageInit()
{
  storageMutex = xSemaphoreCreateMutex();

  isInit = true;
  if (DEFRAG_ON_STARTUP) {
    kveDefrag(&kve);
  }
}

bool storageTest()
{
  xSemaphoreTake(storageMutex, portMAX_DELAY);

  bool pass = kveCheck(&kve);

  xSemaphoreGive(storageMutex);

  DEBUG_PRINT("Storage check %s.\n", pass?"[OK]":"[FAIL]");

  if (!pass) {
    pass = storageReformat();
  }

  return pass;
}

bool storageStore(const char* key, const void* buffer, size_t length)
{
  if (!isInit) {
    return false;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  bool result = kveStore(&kve, key, buffer, length);

  xSemaphoreGive(storageMutex);

  return result;
}


bool storageForeach(const char *prefix, storageFunc_t func)
{
   if (!isInit) {
    return 0;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  bool success = kveForeach(&kve, prefix, func);

  xSemaphoreGive(storageMutex);

  return success;
}

size_t storageFetch(const char *key, void* buffer, size_t length)
{
  if (!isInit) {
    return 0;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  size_t result = kveFetch(&kve, key, buffer, length);

  xSemaphoreGive(storageMutex);

  return result;
}

bool storageDelete(const char* key)
{
  if (!isInit) {
    return false;
  }

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  bool result = kveDelete(&kve, key);

  xSemaphoreGive(storageMutex);

  return result;
}

bool storageReformat() {
  DEBUG_PRINT("Reformatting storage ...\n");

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  kveFormat(&kve);
  bool pass = kveCheck(&kve);

  xSemaphoreGive(storageMutex);

  DEBUG_PRINT("Storage check %s.\n", pass?"[OK]":"[FAIL]");

  if (pass == false) {
    DEBUG_PRINT("Error: Cannot format storage!\n");
  }

  return pass;
}

void storagePrintStats()
{
  kveStats_t stats;

  xSemaphoreTake(storageMutex, portMAX_DELAY);

  kveGetStats(&kve, &stats);

  xSemaphoreGive(storageMutex);


  DEBUG_PRINT("Used storage: %d item stored, %d Bytes/%d Bytes (%d%%)\n", stats.totalItems, stats.itemSize, stats.totalSize, (stats.itemSize*100)/stats.totalSize);
  DEBUG_PRINT("Fragmentation: %d%%\n", stats.fragmentation);
  DEBUG_PRINT("Efficiency: Data: %d Bytes (%d%%), Keys: %d Bytes (%d%%), Metadata: %d Bytes (%d%%)\n",
    stats.dataSize, (stats.dataSize*100)/stats.totalSize,
    stats.keySize, (stats.keySize*100)/stats.totalSize,
    stats.metadataSize, (stats.metadataSize*100)/stats.totalSize);
}

static bool storageStats;

static void printStats(void)
{
  if (storageStats) {
    storagePrintStats();

    storageStats = false;
  }
}

static bool reformatValue;

static void doReformat(void)
{
  if (reformatValue) {
    storageReformat();
  }
}

PARAM_GROUP_START(system)

/**
 * @brief Set to nonzero to dump CPU and stack usage to console
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, storageStats, &storageStats, printStats)

/**
 * @brief Set to nonzero to re-format the storage. Warning: all data will be lost!
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, storageReformat, &reformatValue, doReformat)

PARAM_GROUP_STOP(system)
