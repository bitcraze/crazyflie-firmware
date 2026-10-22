#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "deck_core.h"

typedef enum {
  BCCAM_DECK_BOOT_FIRMWARE = 0,
  BCCAM_DECK_BOOT_BOOTLOADER,
} bccam_deck_boot_mode_t;

typedef struct {
  DeckInfo *deck_info;
} bccam_deck_controller_t;

void bccam_deck_controller_init(bccam_deck_controller_t *controller,
                                DeckInfo *deck_info);

bool bccam_deck_controller_begin_boot(bccam_deck_controller_t *controller,
                                      bccam_deck_boot_mode_t mode,
                                      uint32_t hold_ticks,
                                      uint32_t setup_ticks);

bool bccam_deck_controller_release_boot(bccam_deck_controller_t *controller,
                                        uint32_t wait_ticks);

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
typedef enum {
  BCCAM_DECK_CTRL_TEST_SET_DIRECTION = 0,
  BCCAM_DECK_CTRL_TEST_WRITE,
  BCCAM_DECK_CTRL_TEST_DELAY,
} bccam_deck_controller_test_event_t;

enum {
  BCCAM_DECK_CTRL_TEST_LOW = 0,
  BCCAM_DECK_CTRL_TEST_HIGH = 1,
  BCCAM_DECK_CTRL_TEST_OUTPUT = 1,
};

typedef struct {
  bccam_deck_controller_test_event_t event;
  uint8_t pin;
  uint32_t value;
} bccam_deck_controller_test_trace_entry_t;

void bccam_deck_controller_test_trace_reset(void);
uint8_t bccam_deck_controller_test_trace_count(void);
const bccam_deck_controller_test_trace_entry_t *
bccam_deck_controller_test_trace_entry(uint8_t index);
#endif
