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
  // NOP for now, lets fix the EEPROM write first!
}

static kveMemory_t kve = {
  .memorySize = KVE_PARTITION_LENGTH,
  .read = read,
  .write = write,
  .flush = flush,
};

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
  bool expected = true;
  // Test
  bool actual = kveCheck(&kve);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testFetchEmpty(void) {
  // Fixture
  bool expected = false;
  uint8_t buffer[8];
  // Test
  bool actual = kveFetch(&kve, "testEmpty", buffer, 8);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
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

  printf("size:%i\n", actualRead);

  // Assert
  TEST_ASSERT_EQUAL(expectedStore, actualStore);
  TEST_ASSERT_EQUAL(expectedRead, actualRead);
  TEST_ASSERT_EQUAL_UINT32(u32Store, u32Read);
}

static bool fromStorage(const char *key, void *buffer, size_t length)
{
  uint8_t *byteBuf = (uint8_t *)buffer;
  printf("%s:%i:", key, (int)length);

  for (int i = 0; i < (int)length; i++)
  {
    printf("%.2X", byteBuf[i]);
  }
  printf("\n");

  return true;
}

void testPrintStored(void) {
  // Fixture
  bool expected = true;
  uint32_t u32Store = 0xBEAF;

  // Test
  bool actualStore = kveStore(&kve, "testEmpty", &u32Store, sizeof(uint32_t));
  bool actual = kveForeach(&kve, "", fromStorage);

  // Assert
  TEST_ASSERT_EQUAL(expected, actual);
}

void testStoreUntilMemoryIsFull(void) {
  // Fixture
  int i;
  char keyString[30];
  bool expected = true;
  // Fill memory
  for (i = 0; i < (KVE_PARTITION_LENGTH / 10); i++)
  {
    sprintf(keyString, "prm/test.value%i", i);
    if (kveStore(&kve, keyString, &i, sizeof(i)))
    {
      break;
    }
  }

  printf("Nr stored:%i\n", i);
  // Test

  // Assert
}
