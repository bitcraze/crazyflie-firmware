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
uint8_t stockDeckPropertiesPrimary;
uint8_t stockDeckPropertiesPrimaryFcn() { return stockDeckPropertiesPrimary; }

uint8_t stockDeckPropertiesSecondary;
uint8_t stockDeckPropertiesSecondaryFcn() { return stockDeckPropertiesSecondary; }

uint32_t read_vAddr;
uint8_t read_len;
bool read_isCalled;
bool mockRead(const uint32_t vAddr, const uint8_t len, uint8_t* buffer) { read_isCalled = true; read_vAddr = vAddr; read_len = len; return true; }

uint32_t write_vAddr;
uint8_t write_len;
bool write_isCalled;
bool mockWrite(const uint32_t vAddr, const uint8_t len, const uint8_t* buffer) { write_isCalled = true; write_vAddr = vAddr; write_len = len; return true; }

char* stockName = "A name";
DeckMemDef_t stockPrimaryMemDef;
DeckMemDef_t stockSecondaryMemDef;
DeckDriver stockDriver;
DeckInfo stockInfo;


// Constants
#define SIZE_ONE_DECK_INFO 0x20

#define DECK_0_INFO_PRIMARY   (1 + 0 * SIZE_ONE_DECK_INFO)
#define DECK_0_INFO_SECONDARY (1 + 1 * SIZE_ONE_DECK_INFO)
#define DECK_1_INFO_PRIMARY   (1 + 2 * SIZE_ONE_DECK_INFO)
#define DECK_1_INFO_SECONDARY (1 + 3 * SIZE_ONE_DECK_INFO)
#define DECK_2_INFO_PRIMARY   (1 + 4 * SIZE_ONE_DECK_INFO)
#define DECK_2_INFO_SECONDARY (1 + 5 * SIZE_ONE_DECK_INFO)
#define DECK_3_INFO_PRIMARY   (1 + 6 * SIZE_ONE_DECK_INFO)
#define DECK_3_INFO_SECONDARY (1 + 7 * SIZE_ONE_DECK_INFO)

#define BUF_SIZE 0xff
uint8_t buffer[BUF_SIZE];

#define DECK_MEM_MAP_SIZE 0x10000000

const uint8_t MASK_IS_VALID          = 1;
const uint8_t MASK_IS_STARTED        = 2;
const uint8_t MASK_SUPPORTS_READ     = 4;
const uint8_t MASK_SUPPORTS_WRITE    = 8;
const uint8_t MASK_SUPPORTS_UPGRADE  = 16;
const uint8_t MASK_UPGRADE_REQUIRED  = 32;
const uint8_t MASK_BOOTLOADER_ACTIVE = 64;


void setUp() {
    memset(buffer, 0, BUF_SIZE);
    stockDeckPropertiesPrimary = 0;
    stockDeckPropertiesSecondary = 0;

    // Stock deck definition
    memset(&stockPrimaryMemDef, 0, sizeof(stockPrimaryMemDef));
    stockPrimaryMemDef.properties = stockDeckPropertiesPrimaryFcn;

    memset(&stockSecondaryMemDef, 0, sizeof(stockSecondaryMemDef));
    stockSecondaryMemDef.properties = stockDeckPropertiesSecondaryFcn;

    memset(&stockDriver, 0, sizeof(stockDriver));
    stockDriver.memoryDef = &stockPrimaryMemDef;

    // Set no secondary memory as default to simplify testing
    stockDriver.memoryDefSecondary = 0;

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

void testFirstByteOfInfoIsVersion() {
    // Fixture
    deckCount_ExpectAndReturn(0);

    // Test
    handleMemRead(0, 1, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(2, buffer[0]);
}

void testNoInstalledDecks() {
    // Fixture
    deckCount_ExpectAndReturn(0);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0_INFO_PRIMARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0_INFO_SECONDARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_1_INFO_PRIMARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_1_INFO_SECONDARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_2_INFO_PRIMARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_2_INFO_SECONDARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_3_INFO_PRIMARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_3_INFO_SECONDARY] & MASK_IS_VALID);
}

void testTwoDecksWithoutMemorySupport() {
    // Fixture
    DeckDriver driverNoMem = { .memoryDef = 0 };
    DeckInfo deckNoMem = { .driver = &driverNoMem };

    deckCount_ExpectAndReturn(2);
    deckInfo_ExpectAndReturn(0, &deckNoMem);
    deckInfo_ExpectAndReturn(1, &deckNoMem);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0_INFO_PRIMARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0_INFO_SECONDARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_1_INFO_PRIMARY] & MASK_IS_VALID);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_1_INFO_SECONDARY] & MASK_IS_VALID);
}

void testOneDeckReadPrimarySupported() {
    // Fixture
    stockPrimaryMemDef.read = mockRead;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_READ, buffer[DECK_0_INFO_PRIMARY]);
}

void testOneDeckWritePrimarySupported() {
    // Fixture
    stockPrimaryMemDef.write = mockWrite;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_WRITE, buffer[DECK_0_INFO_PRIMARY]);
}

void testOneDeckUpgradePrimarySupported() {
    // Fixture
    stockPrimaryMemDef.write = mockWrite;
    stockPrimaryMemDef.supportsUpgrade = true;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_WRITE | MASK_SUPPORTS_UPGRADE, buffer[DECK_0_INFO_PRIMARY]);
}

void testOneDeckPrimaryStarted() {
    // Fixture
    stockDeckPropertiesPrimary = DECK_MEMORY_MASK_STARTED;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_IS_STARTED, buffer[DECK_0_INFO_PRIMARY]);
}

void testOneDeckRequiresPrimaryUpgrade() {
    // Fixture
    stockDeckPropertiesPrimary = DECK_MEMORY_MASK_UPGRADE_REQUIRED;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_UPGRADE_REQUIRED, buffer[DECK_0_INFO_PRIMARY]);
}

void testOneDeckPrimaryInBootloaderMode() {
    // Fixture
    stockDeckPropertiesPrimary = DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_BOOTLOADER_ACTIVE, buffer[DECK_0_INFO_PRIMARY]);
}

void testOneDeckSecondaryInfoNotSetByPrimary() {
    // Fixture
    stockPrimaryMemDef.write = mockWrite;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_WRITE, buffer[DECK_0_INFO_PRIMARY]);
    TEST_ASSERT_EQUAL_UINT8(0, buffer[DECK_0_INFO_SECONDARY]);
}

void testOneDeckAtleastOneSecondaryInfoBitIsSet() {
    // Fixture
    stockPrimaryMemDef.write = mockWrite;

    stockDriver.memoryDefSecondary = &stockSecondaryMemDef;
    stockSecondaryMemDef.read = mockRead;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_WRITE, buffer[DECK_0_INFO_PRIMARY]);
    TEST_ASSERT_EQUAL_UINT8(MASK_IS_VALID | MASK_SUPPORTS_READ, buffer[DECK_0_INFO_SECONDARY]);
}

void testOneDeckRequiredHashPrimary() {
    // Fixture
    stockPrimaryMemDef.requiredHash = 0x12345678;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[DECK_0_INFO_PRIMARY + 1]);
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[DECK_0_INFO_PRIMARY + 2]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[DECK_0_INFO_PRIMARY + 3]);
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[DECK_0_INFO_PRIMARY + 4]);
}

void testOneDeckRequiredLengthPrimary() {
    // Fixture
    stockPrimaryMemDef.requiredSize = 0x12345678;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0x78, buffer[DECK_0_INFO_PRIMARY + 5]);
    TEST_ASSERT_EQUAL_UINT8(0x56, buffer[DECK_0_INFO_PRIMARY + 6]);
    TEST_ASSERT_EQUAL_UINT8(0x34, buffer[DECK_0_INFO_PRIMARY + 7]);
    TEST_ASSERT_EQUAL_UINT8(0x12, buffer[DECK_0_INFO_PRIMARY + 8]);
}

void testBaseAddressSecondDeckPrimary() {
    // Fixture
    deckCount_ExpectAndReturn(2);
    deckInfo_ExpectAndReturn(0, &stockInfo);
    deckInfo_ExpectAndReturn(1, &stockInfo);

    uint32_t expected = DECK_MEM_MAP_SIZE * 2;

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    uint32_t actual = 0;
    memcpy(&actual, &buffer[DECK_1_INFO_PRIMARY + 0x0009], 4);

    TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testBaseAddressSecondDeckSecondary() {
    // Fixture
    deckCount_ExpectAndReturn(2);
    deckInfo_ExpectAndReturn(0, &stockInfo);
    deckInfo_ExpectAndReturn(1, &stockInfo);

    stockDriver.memoryDefSecondary = &stockSecondaryMemDef;

    uint32_t expected = DECK_MEM_MAP_SIZE * 2 + DECK_MEM_MAP_SIZE;

    // Test
    handleMemRead(0, BUF_SIZE, buffer);

    // Assert
    uint32_t actual = 0;
    memcpy(&actual, &buffer[DECK_1_INFO_SECONDARY + 0x0009], 4);

    TEST_ASSERT_EQUAL_UINT32(expected, actual);
}

void testReadOfFiveFirstBytes() {
    // Fixture
    stockPrimaryMemDef.requiredHash = 0x12345678;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(0, 5, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(2, buffer[0]);      // Version
    TEST_ASSERT_EQUAL_UINT32(1, buffer[1]);     // valid bit
    TEST_ASSERT_EQUAL_UINT32(0x78, buffer[2]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[3]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[4]);  // Required hash
}

void testReadNotFromZero() {
    // Fixture
    stockPrimaryMemDef.requiredHash = 0x12345678;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    handleMemRead(3, 2, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[0]);  // Required hash
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[1]);  // Required hash
}

void testReadFromPrimaryDeckMemory() {
    // Fixture
    stockPrimaryMemDef.read = mockRead;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    bool actual = handleMemRead(DECK_MEM_MAP_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_TRUE(read_isCalled);
    TEST_ASSERT_EQUAL_UINT32(100, read_vAddr);
    TEST_ASSERT_EQUAL_UINT8(30, read_len);
    TEST_ASSERT_TRUE(actual)
}

void testReadFromSecondaryDeckMemory() {
    // Fixture
    stockDriver.memoryDefSecondary = &stockSecondaryMemDef;
    stockSecondaryMemDef.read = mockRead;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    bool actual = handleMemRead(DECK_MEM_MAP_SIZE * 2 + 100, 30, buffer);

    // Assert
    TEST_ASSERT_TRUE(read_isCalled);
    TEST_ASSERT_EQUAL_UINT32(100, read_vAddr);
    TEST_ASSERT_EQUAL_UINT8(30, read_len);
    TEST_ASSERT_TRUE(actual)
}

void testReadFromDeckWithoutReadFunction() {
    // Fixture
    stockPrimaryMemDef.read = 0;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    bool actual = handleMemRead(DECK_MEM_MAP_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(read_isCalled);
    TEST_ASSERT_FALSE(actual);
}

void testReadFromNonExistingDeck() {
    // Fixture
    deckCount_ExpectAndReturn(0);

    // Test
    bool actual = handleMemRead(DECK_MEM_MAP_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(read_isCalled);
    TEST_ASSERT_FALSE(actual);
}

void testWriteToPrimaryDeckMemory() {
    // Fixture
    stockPrimaryMemDef.write = mockWrite;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    bool actual = handleMemWrite(DECK_MEM_MAP_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_TRUE(write_isCalled);
    TEST_ASSERT_EQUAL_UINT32(100, write_vAddr);
    TEST_ASSERT_EQUAL_UINT8(30, write_len);
    TEST_ASSERT_TRUE(actual)
}

void testWriteToSecondaryDeckMemory() {
    // Fixture
    stockSecondaryMemDef.write = mockWrite;
    stockDriver.memoryDefSecondary = &stockSecondaryMemDef;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    bool actual = handleMemWrite(DECK_MEM_MAP_SIZE * 2 + 100, 30, buffer);

    // Assert
    TEST_ASSERT_TRUE(write_isCalled);
    TEST_ASSERT_EQUAL_UINT32(100, write_vAddr);
    TEST_ASSERT_EQUAL_UINT8(30, write_len);
    TEST_ASSERT_TRUE(actual)
}

void testWriteToDeckWithoutWriteFunction() {
    // Fixture
    stockPrimaryMemDef.write = 0;

    deckCount_ExpectAndReturn(1);
    deckInfo_ExpectAndReturn(0, &stockInfo);

    // Test
    bool actual = handleMemWrite(DECK_MEM_MAP_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(write_isCalled);
    TEST_ASSERT_FALSE(actual);
}

void testWriteToNonExistingDeck() {
    // Fixture
    deckCount_ExpectAndReturn(0);

    // Test
    bool actual = handleMemWrite(DECK_MEM_MAP_SIZE + 100, 30, buffer);

    // Assert
    TEST_ASSERT_FALSE(write_isCalled);
    TEST_ASSERT_FALSE(actual);
}
