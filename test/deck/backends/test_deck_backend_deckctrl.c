#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "deck_discovery.h" // @NO_MODULE
#include "deck_core.h" // @NO_MODULE
#include "i2cdev.h" // @NO_MODULE
#include "mem.h" // @NO_MODULE
#include "FreeRTOS.h" // @NO_MODULE
#include "task.h" // @NO_MODULE
// @MODULE "deck/backends/deck_backend_deckctrl.c"

extern const DeckDiscoveryBackend_t *backend_deckctrlBackend;

I2cDrv deckBus;

static uint8_t metadata[2][0x20];

static void prepareMetadata(const uint8_t index, const char revision) {
  memset(metadata[index], 0, sizeof(metadata[index]));
  metadata[index][0] = 0xBC;
  metadata[index][1] = 0xDC;
  metadata[index][4] = 0xBC;
  metadata[index][5] = 0x15;
  metadata[index][6] = (uint8_t)revision;
  memcpy(&metadata[index][7], "bcCam", 6);
  metadata[index][0x16] = 25;
  metadata[index][0x17] = 1;
  metadata[index][0x18] = 1;

  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(metadata[index]) - 1; i++) {
    sum += metadata[index][i];
  }
  metadata[index][sizeof(metadata[index]) - 1] = (uint8_t)(0U - sum);
}

bool i2cdevReadReg16(I2C_Dev *dev, uint8_t address, uint16_t memAddress,
                     uint16_t length, uint8_t *data) {
  TEST_ASSERT_EQUAL_PTR(&deckBus, dev);

  if (address == DECKCTRL_LISTEN_I2C_ADDRESS) {
    TEST_ASSERT_EQUAL_HEX16(0, memAddress);
    TEST_ASSERT_EQUAL_UINT16(2, length);
    memset(data, 0, length);
    return true;
  }

  if (address == DECKCTRL_DEFAULT_I2C_ADDRESS) {
    TEST_ASSERT_EQUAL_HEX16(0x1900, memAddress);
    TEST_ASSERT_EQUAL_UINT16(12, length);
    memset(data, 0xA5, length);
    return true;
  }

  TEST_ASSERT_TRUE(address == DECKCTRL_START_I2C_ADDRESS ||
                   address == DECKCTRL_START_I2C_ADDRESS + 1);
  TEST_ASSERT_EQUAL_HEX16(0, memAddress);
  TEST_ASSERT_EQUAL_UINT16(sizeof(metadata[0]), length);
  memcpy(data, metadata[address - DECKCTRL_START_I2C_ADDRESS], length);
  return true;
}

bool i2cdevWriteReg16(I2C_Dev *dev, uint8_t address, uint16_t memAddress,
                      uint16_t length, const uint8_t *data) {
  TEST_ASSERT_EQUAL_PTR(&deckBus, dev);
  TEST_ASSERT_EQUAL_HEX8(DECKCTRL_DEFAULT_I2C_ADDRESS, address);
  TEST_ASSERT_EQUAL_HEX16(0x1800, memAddress);
  TEST_ASSERT_EQUAL_UINT16(1, length);
  TEST_ASSERT_TRUE(data[0] == DECKCTRL_START_I2C_ADDRESS ||
                   data[0] == DECKCTRL_START_I2C_ADDRESS + 1);
  return true;
}

void memoryRegisterHandler(const MemoryHandlerDef_t *handler) {
  TEST_ASSERT_NOT_NULL(handler);
}

void vTaskDelay(const TickType_t ticks) {
  (void)ticks;
}

void setUp(void) {
  prepareMetadata(0, 'G');
  prepareMetadata(1, 'F');
}

void tearDown(void) {}

void testDiscoveryPropagatesStableNulTerminatedBoardRevisionPerDeck(void) {
  DeckInfo *first = backend_deckctrlBackend->getNextDeck();
  DeckInfo *second = backend_deckctrlBackend->getNextDeck();

  TEST_ASSERT_NOT_NULL(first);
  TEST_ASSERT_NOT_NULL(second);
  TEST_ASSERT_NOT_NULL(first->boardRevision);
  TEST_ASSERT_NOT_NULL(second->boardRevision);
  TEST_ASSERT_TRUE(first->boardRevision != second->boardRevision);
  TEST_ASSERT_EQUAL_STRING("G", first->boardRevision);
  TEST_ASSERT_EQUAL_STRING("F", second->boardRevision);
}
