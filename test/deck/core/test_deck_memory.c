// File under test deck_memory.c
#include "deck_memory.h"

#include "unity.h"
#include "mock_deck_core.h"
#include "mock_mem.h"

#include <string.h>

// Functions under test
bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);
bool handleMemWrite(const uint32_t memAddr, const uint8_t writeLen, const uint8_t* buffer);


// Fixtures
uint8_t stockDeckProperties;
uint8_t stockDeckPropertiesFcn() { return stockDeckProperties; }

uint32_t read_vAddr;
uint8_t read_len;
bool read_isCalled;
bool mockRead(const uint32_t vAddr, const uint8_t len, uint8_t* buffer) { read_isCalled = true; if (!read_vAddr) read_vAddr = vAddr; read_len += len; return true; }

uint32_t write_vAddr;
uint8_t write_len;
bool write_isCalled;
bool mockWrite(const uint32_t vAddr, const uint8_t len, const uint8_t* buffer) { write_isCalled = true; write_vAddr = vAddr; write_len = len; return true; }

char* stockName = "A name";
DeckMemDef_t stockMemDef;
DeckDriver stockDriver;
DeckInfo stockInfo;


// Constants
#define SIZE_ONE_DECK 0x40
#define MAX_NR_DECKS 4
#define BUF_SIZE (SIZE_ONE_DECK * MAX_NR_DECKS)
uint8_t buffer[BUF_SIZE];

#define DECK_0 (1 + 0 * SIZE_ONE_DECK)
#define DECK_1 (1 + 1 * SIZE_ONE_DECK)
#define DECK_2 (1 + 2 * SIZE_ONE_DECK)
#define DECK_3 (1 + 3 * SIZE_ONE_DECK)

#define DECK_MEM_SIZE 0x10000000

const uint8_t MASK_IS_VALID          = 1;
const uint8_t MASK_IS_STARTED        = 2;
const uint8_t MASK_SUPPORTS_READ     = 4;
const uint8_t MASK_SUPPORTS_WRITE    = 8;
const uint8_t MASK_SUPPORTS_UPGRADE  = 16;
const uint8_t MASK_UPGRADE_REQUIRED  = 32;
const uint8_t MASK_BOOTLOADER_ACTIVE = 64;

// emulate the size the CRTP protocol will enforce
#define READ_CHUNK_SIZE 20

bool handleMemReadWrapper(uint32_t memAddr, size_t len, uint8_t *buffer)
{
    size_t bytesRead = 0;

    while (bytesRead < len) {
        size_t readLen = len - bytesRead;
        if (readLen > READ_CHUNK_SIZE) {
            readLen = READ_CHUNK_SIZE;
        }
        if (!handleMemRead(memAddr + bytesRead, readLen, buffer + bytesRead)) {
            return false;
        }
        bytesRead += readLen;
    }

    return true;
}

void setUp() {
    memset(buffer, 0, BUF_SIZE);
    stockDeckProperties = 0;

    // Stock deck definition
    memset(&stockMemDef, 0, sizeof(stockMemDef));
    stockMemDef.properties = stockDeckPropertiesFcn;

    memset(&stockDriver, 0, sizeof(stockDriver));
    stockDriver.memoryDef = &stockMemDef;
    stockDriver.name = stockName;

    memset(&stockInfo, 0, sizeof(stockInfo));
    stockInfo.driver = &stockDriver;

    read_isCalled = false;
    read_vAddr = 0;
    read_len = 0;

    write_isCalled = false;
    write_vAddr = 0;
    write_len = 0;
}

void testFirstByteIsVersion() {
    // Fixture
    deckCount_IgnoreAndReturn(0);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(1, buffer[0]);
}

void testNoInstalledDecks() {
    // Fixture
    deckCount_IgnoreAndReturn(0);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_1]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_2]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_3]);
}

void testTwoDecskWithoutMemorySupport() {
    // Fixture
    DeckDriver driverNoMem = { .memoryDef = 0 };
    DeckInfo deckNoMem = { .driver = &driverNoMem };

    deckCount_IgnoreAndReturn(2);
    deckInfo_ExpectAndReturn(0, &deckNoMem);
    deckInfo_IgnoreAndReturn(&deckNoMem);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_1]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_2]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_3]);
}

void testOneDeckReadSupported() {
    // Fixture
    stockMemDef.read = mockRead;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_READ, buffer[DECK_0]);
}

void testOneDeckWriteSupported() {
    // Fixture
    stockMemDef.write = mockWrite;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_WRITE, buffer[DECK_0]);
}

void testOneDeckUpgradeupported() {
    // Fixture
    stockMemDef.write = mockWrite;
    stockMemDef.supportsUpgrade = true;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_WRITE | MASK_SUPPORTS_UPGRADE, buffer[DECK_0]);
}

void testOneDeckStarted() {
    // Fixture
    stockDeckProperties = DECK_MEMORY_MASK_STARTED;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_IS_STARTED, buffer[DECK_0]);
}

void testOneDeckRequiresUpgrade() {
    // Fixture
    stockDeckProperties = DECK_MEMORY_MASK_UPGRADE_REQUIRED;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_UPGRADE_REQUIRED, buffer[DECK_0]);
}

void testOneDeckInBootloaderMode() {
    // Fixture
    stockDeckProperties = DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_BOOTLOADER_ACTIVE, buffer[DECK_0]);
}

void testOneDeckRequiredHash() {
    // Fixture
    stockMemDef.requiredHash = 0x12345678;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[DECK_0 + 1]);
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[DECK_0 + 2]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[DECK_0 + 3]);
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[DECK_0 + 4]);
}

void testOneDeckRequiredLength() {
    // Fixture
    stockMemDef.requiredSize = 0x12345678;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[DECK_0 + 5]);
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[DECK_0 + 6]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[DECK_0 + 7]);
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[DECK_0 + 8]);
}

void testBaseAddressSecondDeck() {
    // Fixture
    deckCount_IgnoreAndReturn(2);
    deckInfo_IgnoreAndReturn(&stockInfo);

    uint32_t expected = DECK_MEM_SIZE * 2;

    // Test
    handleMemReadWrapper(0, BUF_SIZE, buffer);

    // Assert
    uint32_t actual = 0;
    memcpy(&actual, &buffer[DECK_1 + 0x0009], 4);

    TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testReadOfFiveFirstBytes() {
    // Fixture
    stockMemDef.requiredHash = 0x12345678;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(0, 5, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(1, buffer[0]);      // Version
    TEST_ASSERT_EQUAL_UINT32(1, buffer[1]);     // valid bit
    TEST_ASSERT_EQUAL_UINT32(0x78, buffer[2]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[3]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[4]);  // Required hash
}

void testReadNotFromZero() {
    // Fixture
    stockMemDef.requiredHash = 0x12345678;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(3, 2, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[0]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[1]);  // Required hash
}

void testReadOfFiveFirstBytesFromThirdDeck() {
    // Fixture
    stockMemDef.requiredHash = 0x12345678;

    deckCount_IgnoreAndReturn(3);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(DECK_2, 5, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(1, buffer[0]);     // valid bit
    TEST_ASSERT_EQUAL_UINT32(0x78, buffer[1]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[2]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[3]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x12, buffer[4]);  // Required hash
}

void testReadPartOfSecondAndThirdDeck() {
    // Fixture
    DeckDriver driverNoMem = { .memoryDef = 0 };
    DeckInfo deckNoMem = { .driver = &driverNoMem };

    stockMemDef.requiredHash = 0x12345678;

    deckCount_IgnoreAndReturn(3);
    deckInfo_IgnoreAndReturn(&deckNoMem);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    handleMemReadWrapper(DECK_2 - 1, 5, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(0, buffer[0]);     // last byte of deck 2
    TEST_ASSERT_EQUAL_UINT32(1, buffer[1]);     // valid bit
    TEST_ASSERT_EQUAL_UINT32(0x78, buffer[2]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[3]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[4]);  // Required hash
}

void testReadFromDeck() {
    // Fixture
    stockMemDef.read = mockRead;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    bool actual = handleMemReadWrapper(DECK_MEM_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_TRUE(read_isCalled);
    TEST_ASSERT_EQUAL_UINT32(100, read_vAddr);
    TEST_ASSERT_EQUAL_UINT8(30, read_len);
    TEST_ASSERT_TRUE(actual)
}

void testReadFromDeckWithoutReadFunction() {
    // Fixture
    stockMemDef.read = 0;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    bool actual = handleMemReadWrapper(DECK_MEM_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(read_isCalled);
    TEST_ASSERT_FALSE(actual);
}

void testReadFromNonExistingDeck() {
    // Fixture
    deckCount_IgnoreAndReturn(0);

    // Test
    bool actual = handleMemReadWrapper(DECK_MEM_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(read_isCalled);
    TEST_ASSERT_FALSE(actual);
}

void testWriteToDeck() {
    // Fixture
    stockMemDef.write = mockWrite;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    bool actual = handleMemWrite(DECK_MEM_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_TRUE(write_isCalled);
    TEST_ASSERT_EQUAL_UINT32(100, write_vAddr);
    TEST_ASSERT_EQUAL_UINT8(30, write_len);
    TEST_ASSERT_TRUE(actual)
}

void testWriteToDeckWithoutWriteFunction() {
    // Fixture
    stockMemDef.write = 0;

    deckCount_IgnoreAndReturn(1);
    deckInfo_IgnoreAndReturn(&stockInfo);

    // Test
    bool actual = handleMemWrite(DECK_MEM_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(write_isCalled);
    TEST_ASSERT_FALSE(actual);
}

void testWriteToNonExistingDeck() {
    // Fixture
    deckCount_IgnoreAndReturn(0);

    // Test
    bool actual = handleMemWrite(DECK_MEM_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(write_isCalled);
    TEST_ASSERT_FALSE(actual);
}
