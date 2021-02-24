// File under test deck_fw_update.c
#include "deck_fw_update.h"

#include "unity.h"
#include "mock_deck_core.h"
#include "mock_mem.h"

#include <string.h>

// Functions under test
bool handleMemRead(const uint32_t memAddr, const uint8_t readLen, uint8_t* buffer);


// Fixtures
// Deck driver 0
DeckFwUpdateDef_t updateDef0 = {
    .requiredHash = 0x12345678,
};
DeckDriver driver0 = {
    .fwUpdate = &updateDef0,
};
DeckInfo info0 = {.driver = &driver0};

// Deck driver 1
DeckDriver driver1 = {
    .fwUpdate = 0,
};
DeckInfo info1 = {.driver = &driver1};

// Deck driver 2
DeckFwUpdateDef_t updateDef2 = {
    .requiredHash = 0x86754231,
};
DeckDriver driver2 = {
    .fwUpdate = &updateDef2,
};
DeckInfo info2 = {.driver = &driver2};

const int nrOfDecks = 3;

uint8_t buffer[255];
uint8_t sizeOfOneDeck = 14;


void setUp() {
    deckCount_IgnoreAndReturn(nrOfDecks);

    memset(buffer, 0, 255);
}

void testThatReadOfFirstByteReturnsNrOfDecks() {
    // Fixture
    // Test
    handleMemRead(0, 1, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(nrOfDecks, buffer[0]);
}

void testThatReadOfFiveFirstBytes() {
    // Fixture
    deckInfo_ExpectAndReturn(0, &info0);

    // Test
    handleMemRead(0, 5, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(nrOfDecks, buffer[0]);
    TEST_ASSERT_EQUAL_UINT32(0x78, buffer[1]);
    TEST_ASSERT_EQUAL_UINT32(0x56, buffer[2]);
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[3]);
    TEST_ASSERT_EQUAL_UINT32(0x12, buffer[4]);
}

void testReadNotFromZero() {
    // Fixture
    deckInfo_ExpectAndReturn(0, &info0);

    // Test
    handleMemRead(3, 2, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(0x34, buffer[0]);
    TEST_ASSERT_EQUAL_UINT32(0x12, buffer[1]);
}

void testThatDeckWithFlashIsUpdatable() {
    // Fixture
    deckInfo_ExpectAndReturn(0, &info0);

    // Test
    handleMemRead(1, sizeOfOneDeck, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(1, buffer[13]);
}

void testThatDeckWithoutFlashIsNotUpdatable() {
    // Fixture
    deckInfo_ExpectAndReturn(1, &info1);

    // Test
    handleMemRead(1 + sizeOfOneDeck, sizeOfOneDeck, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT8(0, buffer[13]);
}

void testReadOfThirdDeck() {
    // Fixture
    deckInfo_ExpectAndReturn(2, &info2);

    // Test
    handleMemRead(1 + sizeOfOneDeck * 2, sizeOfOneDeck, buffer);

    // Assert
    TEST_ASSERT_EQUAL_UINT32(0x31, buffer[0]);
    TEST_ASSERT_EQUAL_UINT32(0x42, buffer[1]);
    TEST_ASSERT_EQUAL_UINT32(0x75, buffer[2]);
    TEST_ASSERT_EQUAL_UINT32(0x86, buffer[3]);

    TEST_ASSERT_EQUAL_UINT8(1, buffer[13]);
}

void testReadOfSecondAndThirdDeck() {
    // Fixture
    deckInfo_ExpectAndReturn(1, &info1);
    deckInfo_ExpectAndReturn(2, &info2);

    // Test
    handleMemRead(1 + sizeOfOneDeck * 2 - 1, 3, buffer);

    // Assert

    // Updateable deck 2
    TEST_ASSERT_EQUAL_UINT32(0, buffer[0]);

    // first 2 bytes of hash for deck 3
    TEST_ASSERT_EQUAL_UINT32(0x31, buffer[1]);
    TEST_ASSERT_EQUAL_UINT32(0x42, buffer[2]);
}

///////////////////////////////////
