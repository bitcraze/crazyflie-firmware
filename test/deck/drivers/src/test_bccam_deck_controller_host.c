#include <stdint.h>

#include "unity.h"
#include "bccam_deck_controller.h"

#define TEST_QCC_EN_PIN 0
#define TEST_QCC_BOOT_PIN 5

void setUp(void) {
  bccam_deck_controller_test_trace_reset();
}

void tearDown(void) {
}

static void assert_trace_entry(uint8_t index,
                               bccam_deck_controller_test_event_t event,
                               uint8_t pin,
                               uint32_t value) {
  const bccam_deck_controller_test_trace_entry_t *entry =
    bccam_deck_controller_test_trace_entry(index);

  TEST_ASSERT_NOT_NULL(entry);
  TEST_ASSERT_EQUAL_INT(event, entry->event);
  TEST_ASSERT_EQUAL_UINT8(pin, entry->pin);
  TEST_ASSERT_EQUAL_UINT32(value, entry->value);
}

void testFirmwareBootWindowDrivesResetLowAndBootLow(void) {
  bccam_deck_controller_t controller;

  bccam_deck_controller_init(&controller, NULL);

  TEST_ASSERT_TRUE(bccam_deck_controller_begin_boot(&controller,
                                                    BCCAM_DECK_BOOT_FIRMWARE,
                                                    11,
                                                    22));

  TEST_ASSERT_EQUAL_UINT8(6, bccam_deck_controller_test_trace_count());
  assert_trace_entry(0,
                     BCCAM_DECK_CTRL_TEST_SET_DIRECTION,
                     TEST_QCC_EN_PIN,
                     BCCAM_DECK_CTRL_TEST_OUTPUT);
  assert_trace_entry(1,
                     BCCAM_DECK_CTRL_TEST_WRITE,
                     TEST_QCC_EN_PIN,
                     BCCAM_DECK_CTRL_TEST_LOW);
  assert_trace_entry(2,
                     BCCAM_DECK_CTRL_TEST_DELAY,
                     0,
                     11);
  assert_trace_entry(3,
                     BCCAM_DECK_CTRL_TEST_SET_DIRECTION,
                     TEST_QCC_BOOT_PIN,
                     BCCAM_DECK_CTRL_TEST_OUTPUT);
  assert_trace_entry(4,
                     BCCAM_DECK_CTRL_TEST_WRITE,
                     TEST_QCC_BOOT_PIN,
                     BCCAM_DECK_CTRL_TEST_LOW);
  assert_trace_entry(5,
                     BCCAM_DECK_CTRL_TEST_DELAY,
                     0,
                     22);
}

void testBootloaderBootWindowDrivesResetLowAndBootHigh(void) {
  bccam_deck_controller_t controller;

  bccam_deck_controller_init(&controller, NULL);

  TEST_ASSERT_TRUE(bccam_deck_controller_begin_boot(&controller,
                                                    BCCAM_DECK_BOOT_BOOTLOADER,
                                                    11,
                                                    22));

  TEST_ASSERT_EQUAL_UINT8(6, bccam_deck_controller_test_trace_count());
  assert_trace_entry(0,
                     BCCAM_DECK_CTRL_TEST_SET_DIRECTION,
                     TEST_QCC_EN_PIN,
                     BCCAM_DECK_CTRL_TEST_OUTPUT);
  assert_trace_entry(1,
                     BCCAM_DECK_CTRL_TEST_WRITE,
                     TEST_QCC_EN_PIN,
                     BCCAM_DECK_CTRL_TEST_LOW);
  assert_trace_entry(2,
                     BCCAM_DECK_CTRL_TEST_DELAY,
                     0,
                     11);
  assert_trace_entry(3,
                     BCCAM_DECK_CTRL_TEST_SET_DIRECTION,
                     TEST_QCC_BOOT_PIN,
                     BCCAM_DECK_CTRL_TEST_OUTPUT);
  assert_trace_entry(4,
                     BCCAM_DECK_CTRL_TEST_WRITE,
                     TEST_QCC_BOOT_PIN,
                     BCCAM_DECK_CTRL_TEST_HIGH);
  assert_trace_entry(5,
                     BCCAM_DECK_CTRL_TEST_DELAY,
                     0,
                     22);
}

void testReleaseBootWindowDrivesResetHigh(void) {
  bccam_deck_controller_t controller;

  bccam_deck_controller_init(&controller, NULL);

  TEST_ASSERT_TRUE(bccam_deck_controller_release_boot(&controller, 33));

  TEST_ASSERT_EQUAL_UINT8(2, bccam_deck_controller_test_trace_count());
  assert_trace_entry(0,
                     BCCAM_DECK_CTRL_TEST_WRITE,
                     TEST_QCC_EN_PIN,
                     BCCAM_DECK_CTRL_TEST_HIGH);
  assert_trace_entry(1,
                     BCCAM_DECK_CTRL_TEST_DELAY,
                     0,
                     33);
}
