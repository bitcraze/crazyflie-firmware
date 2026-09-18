#include "bccam_deck_controller.h"

#include <stddef.h>

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "FreeRTOS.h"
#include "deckctrl_gpio.h"
#include "task.h"
#endif

// Deck controller logical GPIO mapping (STM32 deck-ctrl MCU)
#define GPIO_QCC_EN   0 // PA2 - QCC enable/reset
#define GPIO_QCC_BOOT 5 // PA7 - QCC SYS_BOOT

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
#define OUTPUT 1
#define LOW 0
#define HIGH 1

#define TEST_TRACE_MAX 16

static bccam_deck_controller_test_trace_entry_t test_trace[TEST_TRACE_MAX];
static uint8_t test_trace_count;

static void test_trace_append(bccam_deck_controller_test_event_t event,
                              uint8_t pin,
                              uint32_t value) {
  if (test_trace_count >= TEST_TRACE_MAX) {
    return;
  }

  test_trace[test_trace_count].event = event;
  test_trace[test_trace_count].pin = pin;
  test_trace[test_trace_count].value = value;
  test_trace_count++;
}

static bool controller_set_direction(DeckInfo *deck_info,
                                     uint8_t pin,
                                     uint32_t direction) {
  (void)deck_info;
  test_trace_append(BCCAM_DECK_CTRL_TEST_SET_DIRECTION, pin, direction);
  return true;
}

static bool controller_write(DeckInfo *deck_info, uint8_t pin, uint32_t value) {
  (void)deck_info;
  test_trace_append(BCCAM_DECK_CTRL_TEST_WRITE, pin, value);
  return true;
}

static void controller_delay(uint32_t ticks) {
  test_trace_append(BCCAM_DECK_CTRL_TEST_DELAY, 0, ticks);
}

void bccam_deck_controller_test_trace_reset(void) {
  test_trace_count = 0;
}

uint8_t bccam_deck_controller_test_trace_count(void) {
  return test_trace_count;
}

const bccam_deck_controller_test_trace_entry_t *
bccam_deck_controller_test_trace_entry(uint8_t index) {
  if (index >= test_trace_count) {
    return NULL;
  }

  return &test_trace[index];
}
#else
static bool controller_set_direction(DeckInfo *deck_info,
                                     uint8_t pin,
                                     uint32_t direction) {
  return deckctrl_gpio_set_direction(deck_info, (DeckCtrlGPIOPin)pin, direction);
}

static bool controller_write(DeckInfo *deck_info, uint8_t pin, uint32_t value) {
  return deckctrl_gpio_write(deck_info, (DeckCtrlGPIOPin)pin, value);
}

static void controller_delay(uint32_t ticks) {
  vTaskDelay((TickType_t)ticks);
}
#endif

void bccam_deck_controller_init(bccam_deck_controller_t *controller,
                                DeckInfo *deck_info) {
  if (controller == NULL) {
    return;
  }

  controller->deck_info = deck_info;
}

bool bccam_deck_controller_begin_boot(bccam_deck_controller_t *controller,
                                      bccam_deck_boot_mode_t mode,
                                      uint32_t hold_ticks,
                                      uint32_t setup_ticks) {
  if (controller == NULL) {
    return false;
  }
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  if (controller->deck_info == NULL) {
    return false;
  }
#endif

  if (!controller_set_direction(controller->deck_info, GPIO_QCC_EN, OUTPUT)) {
    return false;
  }
  if (!controller_write(controller->deck_info, GPIO_QCC_EN, LOW)) {
    return false;
  }
  controller_delay(hold_ticks);

  if (!controller_set_direction(controller->deck_info, GPIO_QCC_BOOT, OUTPUT)) {
    return false;
  }
  const uint32_t boot_value =
    (mode == BCCAM_DECK_BOOT_BOOTLOADER) ? HIGH : LOW;
  if (!controller_write(controller->deck_info, GPIO_QCC_BOOT, boot_value)) {
    return false;
  }
  controller_delay(setup_ticks);

  return true;
}

bool bccam_deck_controller_release_boot(bccam_deck_controller_t *controller,
                                        uint32_t wait_ticks) {
  if (controller == NULL) {
    return false;
  }
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  if (controller->deck_info == NULL) {
    return false;
  }
#endif

  if (!controller_write(controller->deck_info, GPIO_QCC_EN, HIGH)) {
    return false;
  }
  controller_delay(wait_ticks);

  return true;
}
