// File under test kve.c
#include "kve/kve.h"
#include "kve/kve_storage.h"

#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <errno.h>

#include "unity.h"

#define KVE_PARTITION_LENGTH (7*1024)

uint8_t kveData[KVE_PARTITION_LENGTH];

static size_t read(size_t address, void* data, size_t length)
{
  if ((length == 0) || (address + length > KVE_PARTITION_LENGTH)) {
    return 0;
  }

  memcpy(data, &kveData[address], length);

  return length;
}

static size_t write(size_t address, const void* data, size_t length)
{
  if ((length == 0) || (address + length > KVE_PARTITION_LENGTH)) {
    return 0;
  }

  memcpy(&kveData[address], data, length);

  return length;
}

static void flush(void)
{
  // Not valid for RAM memory implementation.
}

static kveMemory_t kve = {
  .memorySize = KVE_PARTITION_LENGTH,
  .read = read,
  .write = write,
  .flush = flush,
};

static bool fromStorageOneKey(const char *key, void *buffer, size_t length)
{

  (void)length;
  (void)buffer;
  TEST_ASSERT_EQUAL_STRING("prm/testEmpty", key);

// For debug printing
//  uint8_t *byteBuf = (uint8_t *)buffer;
//  printf("%s:%i:", key, (int)length);
//
//  for (int i = 0; i < (int)length; i++)
//  {
//    printf("%.2X", byteBuf[i]);
//  }
//  printf("\n");

  return true;
}

static bool fromFullStorageAllKeys(const char *key, void *buffer, size_t length)
{
  static uint32_t i = 0;
  (void)length;
  (void)buffer;
  char keyString[30];

// For debug printing
//  uint8_t *byteBuf = (uint8_t *)buffer;
//  printf("%s:%i\n", key, (int)length);

  sprintf(keyString, "prm/test.value%i", i++);
  TEST_ASSERT_EQUAL_STRING(keyString, key);

  return true;
}

static void fillKveMemory(void)
{
  int i;
  char keyString[30];
  // Fill memory
  for (i = 0; i < (KVE_PARTITION_LENGTH / 10); i++)
  {
    sprintf(keyString, "prm/test.value%i", i);
    if (!kveStore(&kve, keyString, &i, sizeof(i)))
    {
      break;
    }
  }
  //printf("Nr stored:%i\n", i);
}

//-----------------------------Test cases -------------------------------- //

void setUp(void) {
  // The full memory is initialized to zero
  memset(kveData, 0, KVE_PARTITION_LENGTH);
  kveFormat(&kve);
}

void tearDown(void) {
  // Empty
}

void testSetupKve(void) {
  // Fixture
  // Test
  bool actual = kveCheck(&kve);

  // Assert
  TEST_ASSERT_EQUAL(true, actual);
}

void testFetchEmpty(void) {
  // Fixture
  uint8_t buffer[8];
  // Test
  bool actual = kveFetch(&kve, "testEmpty", buffer, 8);

  // Assert
  TEST_ASSERT_EQUAL(false, actual);
}

void testStoreAndReadFirstKeyValue(void) {
  // Fixture
  bool expectedStore = true;
  int expectedRead = 4;
  uint32_t u32Store = 0xBEAF;
  uint32_t u32Read = 0;
  // Test
  bool actualStore = kveStore(&kve, "testEmpty", &u32Store, sizeof(uint32_t));
  int actualRead = kveFetch(&kve, "testEmpty", &u32Read, sizeof(uint32_t));

//  printf("size:%i\n", actualRead);

  // Assert
  TEST_ASSERT_EQUAL(expectedStore, actualStore);
  TEST_ASSERT_EQUAL(expectedRead, actualRead);
  TEST_ASSERT_EQUAL_UINT32(u32Store, u32Read);
}

void testOneStoredForEach(void) {
  // Fixture
  uint32_t u32Store = 0xBEAF;

  // Test
  bool actualStore = kveStore(&kve, "prm/testEmpty", &u32Store, sizeof(uint32_t));

  // Assert
  bool actual = kveForeach(&kve, "", fromStorageOneKey);
  TEST_ASSERT_EQUAL(true, actualStore);
  TEST_ASSERT_EQUAL(true, actual);
}

void testFullStorageForEach(void) {
  // Fixture
  uint32_t u32Store = 0xBEAF;
  // Fill memory
  fillKveMemory();

  // Test
  // Assert
  bool actual = kveForeach(&kve, "", fromFullStorageAllKeys);
  TEST_ASSERT_EQUAL(true, actual);
}

void testStoreWhenMemoryIsFull(void) {
  // Fixture
  bool expected = false;
  uint32_t u32Store = 0xBEAF;
  // Fill memory
  fillKveMemory();
  // Test
  bool actual = kveStore(&kve, "prm/full", &u32Store, sizeof(uint32_t));
  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testRemoveAndStoreWhenMemoryIsFull(void) {
  // Fixture
  uint32_t u32Store = 0xBEAF;
  // Fill memory
  fillKveMemory();
  // Test
  bool actualDelete = kveDelete(&kve, "prm/test.value10");
  bool actualStore =  kveStore(&kve,  "prm/test.hole10", &u32Store, sizeof(uint32_t));
  // Assert
  TEST_ASSERT_EQUAL(true, actualDelete);
  TEST_ASSERT_EQUAL(true, actualStore);
}

void testRemoveAndStoreBiggerWhenMemoryIsFull(void) {
  // Fixture
  uint32_t u32Store = 0xBEAF;
  // Fill memory
  fillKveMemory();
  // Test
  bool actualFull =  kveStore(&kve,  "prm/full", &u32Store, sizeof(uint32_t));
  bool actualDelete = kveDelete(&kve, "prm/test.value1");
  bool actualStore =  kveStore(&kve,  "prm/test.bigger1000", &u32Store, sizeof(uint32_t));
  // Assert
  TEST_ASSERT_EQUAL(false, actualFull);
  TEST_ASSERT_EQUAL(true, actualDelete);
  TEST_ASSERT_EQUAL(false, actualStore);
}

void testStorageStatistics(void) {
  // Fixture
  // Fill memory
  char *name = "hello";
  uint32_t i = 42;
  kveStore(&kve, name, &i, sizeof(i));
  // Test
  kveStats_t stats;
  kveGetStats(&kve, &stats);
  // Assert
  TEST_ASSERT_EQUAL(strlen(name), stats.keySize);
  TEST_ASSERT_EQUAL(sizeof(i), stats.dataSize);
  TEST_ASSERT_EQUAL(3, stats.metadataSize);
  TEST_ASSERT_EQUAL(strlen(name) + sizeof(i) + 3, stats.itemSize);
  TEST_ASSERT_EQUAL(1, stats.totalItems);
  TEST_ASSERT_EQUAL(strlen(name) + sizeof(i) + 3, stats.totalSize - stats.freeSpace);
  TEST_ASSERT_EQUAL(0, stats.fragmentation);
  TEST_ASSERT_EQUAL(0, stats.holeSize);
  TEST_ASSERT_EQUAL(strlen(name) + sizeof(i) + 3, stats.totalSize - stats.spaceLeftUntilForcedDefrag);
}

void testFragmentationStatisticsIncreaseWhenFragmented(void) {
  // Fixture
  // Fill memory
  char data[128];
  kveStore(&kve, "hello", data, sizeof(data));
  kveDelete(&kve, "hello");
  kveStore(&kve, "hello", data, sizeof(data));
  // Test
  kveStats_t stats;
  kveGetStats(&kve, &stats);
  // Assert
  TEST_ASSERT_NOT_EQUAL(0, stats.fragmentation);
}