#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "deck_core.h" // @NO_MODULE
#include "deckctrl_gpio.h" // @NO_MODULE
#include "bccam_uart_service.h" // @NO_MODULE
#include "task.h" // @NO_MODULE
// @MODULE "bccam_deck.c"

extern const DeckDriver *driver_bccam_deck;
void bccam_deck_test_reset(void);

uint8_t bccam_uart_service_link_state_log;
static uint8_t gpio_direction_count;
static uint8_t gpio_write_count;
static uint8_t service_init_count;
static uint8_t delay_count;
static DeckCtrlGPIOPin direction_pins[2];
static DeckCtrlGPIOPin write_pins[2];
static bccam_uart_service_state_t service_state;
static DeckInfo deck_info;

bool deckctrl_gpio_set_direction(DeckInfo *info, DeckCtrlGPIOPin pin,
                                 uint32_t direction) {
  (void)info;
  TEST_ASSERT_EQUAL_UINT32(OUTPUT, direction);
  direction_pins[gpio_direction_count++] = pin;
  return true;
}

bool deckctrl_gpio_write(DeckInfo *info, DeckCtrlGPIOPin pin, uint32_t value) {
  (void)info;
  TEST_ASSERT_EQUAL_UINT32(HIGH, value);
  write_pins[gpio_write_count++] = pin;
  return true;
}

void vTaskDelay(const TickType_t ticks) {
  TEST_ASSERT_EQUAL_UINT32(M2T(50), ticks);
  delay_count++;
}

void bccam_uart_service_init(DeckInfo *info) {
  TEST_ASSERT_EQUAL_PTR(&deck_info, info);
  service_init_count++;
}
void bccam_uart_service_get_status(bccam_uart_service_status_t *status) {
  memset(status, 0, sizeof(*status));
  status->service_state = service_state;
}
bool bccam_uart_service_state_has_active_bootloader(
  bccam_uart_service_state_t state) {
  return state == BCCAM_UART_SERVICE_STATE_ISP_READY ||
         state == BCCAM_UART_SERVICE_STATE_ISP_BUSY;
}
void bccam_uart_service_request_bootloader(void) {}
void bccam_uart_service_request_firmware(void) {}
bool bccam_uart_service_write_flash(uint32_t address, uint8_t length,
                                    const uint8_t *data, uint32_t image_size) {
  (void)address; (void)length; (void)data; (void)image_size; return true;
}
bool bccam_uart_service_read_flash(uint32_t address, uint8_t length,
                                   uint8_t *data) {
  (void)address; (void)length; (void)data; return true;
}

void setUp(void) {
  memset(&deck_info, 0, sizeof(deck_info));
  gpio_direction_count = 0;
  gpio_write_count = 0;
  service_init_count = 0;
  delay_count = 0;
  service_state = BCCAM_UART_SERVICE_STATE_UNINITIALIZED;
  bccam_deck_test_reset();
}

void tearDown(void) {}

static void assert_no_init_side_effects(void) {
  TEST_ASSERT_EQUAL_UINT8(0, gpio_direction_count);
  TEST_ASSERT_EQUAL_UINT8(0, gpio_write_count);
  TEST_ASSERT_EQUAL_UINT8(0, service_init_count);
  TEST_ASSERT_EQUAL_UINT8(0, delay_count);
  TEST_ASSERT_FALSE(driver_bccam_deck->test());
}

void testMissingBoardRevisionIsRejectedWithoutHardwareOrProtocolSideEffects(void) {
  deck_info.boardRevision = NULL;
  driver_bccam_deck->init(&deck_info);
  assert_no_init_side_effects();
}

void testNonGBoardRevisionIsRejectedWithoutHardwareOrProtocolSideEffects(void) {
  deck_info.boardRevision = "F";
  driver_bccam_deck->init(&deck_info);
  assert_no_init_side_effects();
  TEST_ASSERT_BITS_LOW(DECK_MEMORY_MASK_STARTED,
                       driver_bccam_deck->memoryDef->properties());
  TEST_ASSERT_BITS_HIGH(DECK_MEMORY_MASK_SUPPORTS_HOT_RESTART,
                        driver_bccam_deck->memoryDef->properties());
}

void testRevisionGInitializesRevGPowerAndUartSelectMappings(void) {
  deck_info.boardRevision = "G";
  driver_bccam_deck->init(&deck_info);

  TEST_ASSERT_EQUAL_UINT8(2, gpio_direction_count);
  TEST_ASSERT_EQUAL_UINT8(2, gpio_write_count);
  TEST_ASSERT_EQUAL_INT(DECKCTRL_GPIO_PIN_10, direction_pins[0]);
  TEST_ASSERT_EQUAL_INT(DECKCTRL_GPIO_PIN_11, direction_pins[1]);
  TEST_ASSERT_EQUAL_INT(DECKCTRL_GPIO_PIN_10, write_pins[0]);
  TEST_ASSERT_EQUAL_INT(DECKCTRL_GPIO_PIN_11, write_pins[1]);
  TEST_ASSERT_EQUAL_UINT8(1, delay_count);
  TEST_ASSERT_EQUAL_UINT8(1, service_init_count);
  TEST_ASSERT_TRUE(driver_bccam_deck->test());
  TEST_ASSERT_BITS_HIGH(DECK_MEMORY_MASK_STARTED,
                        driver_bccam_deck->memoryDef->properties());
}

void testDeckPropertiesOnlyReportBootloaderActiveForReadyAndBusy(void) {
  service_state = BCCAM_UART_SERVICE_STATE_ISP_ENTERING;
  TEST_ASSERT_BITS_LOW(DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE,
                       driver_bccam_deck->memoryDef->properties());
  service_state = BCCAM_UART_SERVICE_STATE_ISP_READY;
  TEST_ASSERT_BITS_HIGH(DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE,
                        driver_bccam_deck->memoryDef->properties());
  service_state = BCCAM_UART_SERVICE_STATE_ISP_BUSY;
  TEST_ASSERT_BITS_HIGH(DECK_MEMORY_MASK_BOOT_LOADER_ACTIVE,
                        driver_bccam_deck->memoryDef->properties());
}
