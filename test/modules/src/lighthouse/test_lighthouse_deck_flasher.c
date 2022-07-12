// @IGNORE_IF_NOT CONFIG_DECK_LIGHTHOUSE

// File under test lighthouse_deck_flasher.c
#include "lighthouse_deck_flasher.h"

#include "unity.h"
#include "mock_system.h"
#include "mock_lh_bootloader.h"
#include "mock_crc32.h"

#include "freertosMocks.h"

#include <stdbool.h>

const DeckMemDef_t* noMemDef = 0;


void setUp(void) {
  // Empty
}

void tearDown(void) {
  // Empty
}


void testThaEraseFwIsCalledWhenWritingTheFirstBlock() {
  // Fixture
  lhblFlashEraseFirmware_ExpectAndReturn(true);
  uint8_t buffer[] = {1, 2, 3, 4};
  lhblFlashWritePage_ExpectWithArrayAndReturn(LH_FW_ADDR + 0, 4, buffer, 4, true);

  // Test
  bool actual = lighthouseDeckFlasherWrite(0, 4, buffer, noMemDef);

  // Actual
  TEST_ASSERT_TRUE(actual);
}


void testThaEraseFwSplitsWriteBetweenTwoPages() {
  // Fixture
  uint8_t buffer[] = {1, 2, 3, 4};
  lhblFlashWritePage_ExpectWithArrayAndReturn(LH_FW_ADDR + 254, 2, buffer, 2, true);
  lhblFlashWritePage_ExpectWithArrayAndReturn(LH_FW_ADDR + 256, 2, &buffer[2], 2, true);


  // Test
  bool actual = lighthouseDeckFlasherWrite(254, 4, buffer, noMemDef);

  // Actual
  TEST_ASSERT_TRUE(actual);
}
