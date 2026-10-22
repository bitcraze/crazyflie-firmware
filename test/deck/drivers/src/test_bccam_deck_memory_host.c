#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "deck_memory.h"
#include "deck_core.h" // @NO_MODULE
#include "bccam_uart_service.h"
#include "bccam_bootloader_uart_client.h" // @NO_MODULE
#include "deckctrl_gpio.h" // @NO_MODULE
#include "mem.h" // @NO_MODULE
#include "task.h" // @NO_MODULE
// @MODULE "bccam_bootloader_uart_client.c"
// @MODULE "bccam_deck.c"
// @MODULE "bccam_deck_controller.c"
// @MODULE "bccam_firmware_uart_client.c"
// @MODULE "bccam_uart_crc.c"
// @MODULE "bccam_uart_frame.c"
// @MODULE "bccam_uart_link.c"
// @MODULE "bccam_uart_runtime.c"

#define DECK_0_PRIMARY_MEMORY_BASE 0x10000000u
#define ISP_CMD_FLASH_WRITE 0x31u

extern const DeckDriver *driver_bccam_deck;

static DeckInfo *installed_deck;

int deckCount(void) { return 1; }
DeckInfo *deckInfo(int index) {
  TEST_ASSERT_EQUAL_INT(0, index);
  return installed_deck;
}
void memoryRegisterHandler(const MemoryHandlerDef_t *handler) { (void)handler; }
bool deckctrl_gpio_set_direction(DeckInfo *info, DeckCtrlGPIOPin pin,
                                 uint32_t direction) {
  (void)info; (void)pin; (void)direction; return true;
}
bool deckctrl_gpio_write(DeckInfo *info, DeckCtrlGPIOPin pin, uint32_t value) {
  (void)info; (void)pin; (void)value; return true;
}
void vTaskDelay(const TickType_t ticks) { (void)ticks; }

bool handleMemWrite(const uint8_t internal_id,
                    const uint32_t mem_addr,
                    const uint8_t write_len,
                    const uint8_t *buffer);

void setUp(void) {
  bccam_uart_service_test_reset();
  bccam_bootloader_uart_client_test_trace_reset();
}

void tearDown(void) {
}

static void queue_bootloader_ok(void) {
  const uint8_t ok[] = { 'O', 'K' };
  bccam_bootloader_uart_client_test_queue_rx(ok, sizeof(ok));
}

static const bccam_bootloader_uart_client_test_trace_entry_t *
find_flash_write_command(void) {
  const uint8_t count = bccam_bootloader_uart_client_test_trace_count();
  for (uint8_t i = 0; i < count; i++) {
    const bccam_bootloader_uart_client_test_trace_entry_t *entry =
      bccam_bootloader_uart_client_test_trace_entry(i);
    if (entry != NULL &&
        entry->event == BCCAM_BOOTLOADER_UART_CLIENT_TEST_UART_SEND &&
        entry->length >= 8 && entry->bytes[0] == ISP_CMD_FLASH_WRITE) {
      return entry;
    }
  }

  TEST_FAIL_MESSAGE("ISP flash-write command not found");
  return NULL;
}

void testDeckMemoryImageOffsetZeroReachesQccRomAsAddressZero(void) {
  const uint8_t image[] = { 1, 2, 3, 4 };
  DeckInfo deck_info = {
    .driver = driver_bccam_deck,
  };

  *driver_bccam_deck->memoryDef->newFwSizeP = sizeof(image);
  bccam_uart_service_test_set_state(BCCAM_UART_SERVICE_STATE_ISP_READY);
  queue_bootloader_ok();
  queue_bootloader_ok();
  installed_deck = &deck_info;

  TEST_ASSERT_TRUE(handleMemWrite(0,
                                  DECK_0_PRIMARY_MEMORY_BASE,
                                  sizeof(image),
                                  image));

  const bccam_bootloader_uart_client_test_trace_entry_t *write =
    find_flash_write_command();
  TEST_ASSERT_EQUAL_HEX8(0, write->bytes[4]);
  TEST_ASSERT_EQUAL_HEX8(0, write->bytes[5]);
  TEST_ASSERT_EQUAL_HEX8(0, write->bytes[6]);
  TEST_ASSERT_EQUAL_HEX8(0, write->bytes[7]);
}
