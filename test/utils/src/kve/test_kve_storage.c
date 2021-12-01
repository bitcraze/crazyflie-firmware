#include "kve/kve_storage.h"

#include "unity.h"
#include <string.h>

#define TEST_MEMORY_SIZE 100

static size_t min(size_t a, size_t b)
{
    if (a < b) {
        return a;
    } else {
        return b;
    }
}

char memory[TEST_MEMORY_SIZE];
char cacheMemory[TEST_MEMORY_SIZE];

size_t kvememoryRead(size_t address, void* data, size_t length) {
    if(address > TEST_MEMORY_SIZE) {
        return 0;
    }
    size_t toRead = min(length, TEST_MEMORY_SIZE - address);
    memcpy(data, &memory[address], toRead);

    return toRead;
}

size_t kvememoryWrite(size_t address, const void* data, size_t length) {
    if(address > TEST_MEMORY_SIZE) {
        return 0;
    }
    size_t toWrite = min(length, TEST_MEMORY_SIZE - address);
    memcpy(&cacheMemory[address], data, toWrite);

    return toWrite;
}

void kvememoryFlush() {
    memcpy(memory, cacheMemory, TEST_MEMORY_SIZE);
}

kveMemory_t kveMemory = {
    .memorySize = TEST_MEMORY_SIZE,
    .read = kvememoryRead,
    .write = kvememoryWrite,
    .flush = kvememoryFlush,
};

void setUp(void) {
    // The full memory is initialized to the characted 'a'
    memset(memory, 'a', TEST_MEMORY_SIZE);
    memset(cacheMemory, 'a', TEST_MEMORY_SIZE);
}

void testThatWriteItemDoesWriteTheItemAtTheRightAddress() {
  // Fixture
  size_t address = 42;
  char *key = "hello";
  void *buffer = "world";
  size_t bufferLength = 5;

  size_t expectedItemSize = 3 + strlen(key) + strlen(buffer);
  char *expectedItemInMemory = "a\x0d\x00\x05helloworlda";


  // Test
  size_t itemSize = kveStorageWriteItem(&kveMemory, address, key, buffer, bufferLength);

  // Assert
  TEST_ASSERT_EQUAL(expectedItemSize, itemSize);
  TEST_ASSERT_EQUAL_MEMORY(expectedItemInMemory, &memory[address-1], expectedItemSize+2);
}

void testThatWriteHoleDoesWriteAHoleAtTheRightAddress() {
  // Fixture
  memset(memory, 'a', TEST_MEMORY_SIZE);

  size_t address = 42;
  uint16_t length = 5;

  size_t expectedItemSize = length;
  char *expectedItemInMemory = "a\x05\0\0aa";


  // Test
  size_t itemSize = kveStorageWriteHole(&kveMemory, address, length);

  // Assert
  TEST_ASSERT_EQUAL(expectedItemSize, itemSize);
  TEST_ASSERT_EQUAL_MEMORY(expectedItemInMemory, &memory[address - 1], expectedItemSize + 1);
}

void testThatWriteEndDoesWriteTheEndAtTheRightAddress() {
  // Fixture
  memset(memory, 'a', TEST_MEMORY_SIZE);

  size_t address = 42;

  size_t expectedItemSize = 2;
  char *expectedItemInMemory = "a\xff\xff" "a";


  // Test
  size_t itemSize = kveStorageWriteEnd(&kveMemory, address);

  // Assert
  TEST_ASSERT_EQUAL(expectedItemSize, itemSize);
  TEST_ASSERT_EQUAL_MEMORY(expectedItemInMemory, &memory[address - 1], expectedItemSize + 2);
}

void testThatMoveMemoryDoesMoveAnItem() {
  // Fixture
  size_t address = 42;
  size_t newAddress = 7;
  char *key = "hello";
  void *buffer = "world";
  size_t bufferLength = 5;

  char *expectedItemInMemory = "\x0d\x00\x05helloworld";

  // Create an item
  size_t itemSize = kveStorageWriteItem(&kveMemory, address, key, buffer, bufferLength);

  // Test
  kveStorageMoveMemory(&kveMemory, address, newAddress, itemSize);

  // Assert
  TEST_ASSERT_EQUAL_MEMORY(expectedItemInMemory, &memory[newAddress], itemSize);
}

void testThatFindItemByKeyFindsAnItem() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);

  // Test
  size_t found_address = kveStorageFindItemByKey(&kveMemory, 0, "world");

  // Assert
  TEST_ASSERT_EQUAL(world_address, found_address);
}

void testThatFindItemByKeyDoNotFindAnItemWhenSearchingAfterIt() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 13;
  size_t end_address = world_address + 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);
  kveStorageWriteEnd(&kveMemory, end_address);

  // Test
  size_t found_address = kveStorageFindItemByKey(&kveMemory, world_address, "hello");

  // Assert
  TEST_ASSERT_FALSE(KVE_STORAGE_IS_VALID(found_address));
}

void testThatFindItemByPrefixFindsAnItem() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 17;

  kveStorageWriteItem(&kveMemory, hello_address, "pre/hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "pre/world", "hello", 5);

  size_t foundAdr = 0;
  char foundKey[] = "qwertyqwertyqwertyqwerty";

  // Test
  size_t size = kveStorageFindItemByPrefix(&kveMemory, 0, "pre/", foundKey, &foundAdr);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(hello_address, foundAdr);
  TEST_ASSERT_EQUAL_STRING("pre/hello", foundKey);
  TEST_ASSERT_EQUAL_UINT32(17, size);
}

void testThatFindItemByPrefixFindsNextItem() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 17;

  kveStorageWriteItem(&kveMemory, hello_address, "pre/hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "pre/world", "hello", 5);

  size_t foundAdr = 0;
  char foundKey[] = "qwertyqwertyqwertyqwerty";
  size_t size = kveStorageFindItemByPrefix(&kveMemory, 0, "pre/", foundKey, &foundAdr);

  // Test
  size = kveStorageFindItemByPrefix(&kveMemory, foundAdr + size, "pre/", foundKey, &foundAdr);

  // Assert
  TEST_ASSERT_EQUAL_UINT32(world_address, foundAdr);
  TEST_ASSERT_EQUAL_STRING("pre/world", foundKey);
  TEST_ASSERT_EQUAL_UINT32(17, size);
}



void testThatFindEndFindsTheEnd() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 13;
  size_t end_address = world_address + 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);
  kveStorageWriteEnd(&kveMemory, end_address);

  // Test
  size_t found_address = kveStorageFindEnd(&kveMemory, 0);

  // Assert
  TEST_ASSERT_EQUAL(end_address, found_address);
}

void testThatFindEndReturnInvalidAddressIfNoEnd() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);

  // Test
  size_t found_address = kveStorageFindEnd(&kveMemory, 0);

  // Assert
  TEST_ASSERT_FALSE(KVE_STORAGE_IS_VALID(found_address));
}

void testThatFindNextItemFindsTheNextItem() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 13;
  size_t end_address = world_address + 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);
  kveStorageWriteEnd(&kveMemory, end_address);

  // Test
  size_t found_address = kveStorageFindNextItem(&kveMemory, hello_address);

  // Assert
  TEST_ASSERT_EQUAL(world_address, found_address);
}

void testThatFindNextItemReturnInvalidAddressIfNoMoreItem() {
  // Fixture
  size_t hello_address = 0;
  size_t world_address = 13;
  size_t end_address = world_address + 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);
  kveStorageWriteEnd(&kveMemory, end_address);

  // Test
  size_t found_address = kveStorageFindNextItem(&kveMemory, world_address);

  // Assert
  TEST_ASSERT_FALSE(KVE_STORAGE_IS_VALID(found_address));
}

void testThatFindNextItemJumpsOverHoles() {
  // Fixture
  size_t hello_address = 0;
  size_t hole_address = 13;
  size_t hole_length = 10;
  size_t world_address = 23;
  size_t end_address = world_address + 13;

  kveStorageWriteItem(&kveMemory, hello_address, "hello", "world", 5);
  kveStorageWriteHole(&kveMemory, hole_address, hole_length);
  kveStorageWriteItem(&kveMemory, world_address, "world", "hello", 5);
  kveStorageWriteEnd(&kveMemory, end_address);

  // Test
  size_t found_address = kveStorageFindNextItem(&kveMemory, hello_address);

  // Assert
  TEST_ASSERT_EQUAL(world_address, found_address);
}
