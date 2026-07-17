#include <stdint.h>
#include <string.h>

#include "unity.h"
#include "deck_memory.h" // @NO_MODULE
#include "deck_core.h" // @NO_MODULE
#include "deckctrl_gpio.h" // @NO_MODULE
#include "mem.h" // @NO_MODULE
#include "system.h" // @NO_MODULE
#include "FreeRTOS.h" // @NO_MODULE
#include "task.h" // @NO_MODULE
#include "uart1.h" // @NO_MODULE
// @MODULE "deck/core/deck_memory.c"
// @MODULE "bccam_deck.c"

#define DECK_0_PRIMARY_MEMORY_BASE 0x10000000u
#define TEST_FIRMWARE_BAUDRATE 1000000u
#define ISP_CMD_FLASH_ERASE 0x30u
#define ISP_CMD_FLASH_WRITE 0x31u

extern const DeckDriver *driver_bccam_deck;
void bccam_deck_test_reset(void);
void bccam_deck_test_set_bootloader(bool active);
bool handleMemWrite(const uint8_t internal_id, const uint32_t mem_addr,
                    const uint8_t write_len, const uint8_t *buffer);

typedef enum {
  EVENT_GPIO_DIRECTION,
  EVENT_GPIO_WRITE,
  EVENT_DELAY,
  EVENT_UART_INIT,
  EVENT_TASK_CREATE,
} event_type_t;

typedef struct {
  event_type_t type;
  uint32_t first;
  uint32_t second;
} event_t;

typedef struct {
  uint32_t length;
  uint8_t bytes[16];
} uart_send_t;

static DeckInfo deck_info;
static event_t events[16];
static uint8_t event_count;
static uart_send_t uart_sends[8];
static uint8_t uart_send_count;
static const uint8_t ack_bytes[] = { 'O', 'K', 'O', 'K' };
static uint8_t ack_index;

static void record_event(event_type_t type, uint32_t first, uint32_t second) {
  TEST_ASSERT_LESS_THAN_UINT8(sizeof(events) / sizeof(events[0]), event_count);
  events[event_count++] = (event_t) {
    .type = type,
    .first = first,
    .second = second,
  };
}

bool deckctrl_gpio_set_direction(DeckInfo *info, DeckCtrlGPIOPin pin,
                                 uint32_t direction) {
  TEST_ASSERT_EQUAL_PTR(&deck_info, info);
  record_event(EVENT_GPIO_DIRECTION, pin, direction);
  return true;
}

bool deckctrl_gpio_write(DeckInfo *info, DeckCtrlGPIOPin pin, uint32_t value) {
  TEST_ASSERT_EQUAL_PTR(&deck_info, info);
  record_event(EVENT_GPIO_WRITE, pin, value);
  return true;
}

void vTaskDelay(const TickType_t ticks) {
  record_event(EVENT_DELAY, ticks, 0);
}

BaseType_t xTaskCreate(TaskFunction_t task_code, const char * const name,
                       const configSTACK_DEPTH_TYPE stack_depth,
                       void * const parameters, UBaseType_t priority,
                       TaskHandle_t * const created_task) {
  (void)task_code;
  (void)stack_depth;
  (void)parameters;
  (void)priority;
  (void)created_task;
  TEST_ASSERT_EQUAL_STRING("bcCam", name);
  record_event(EVENT_TASK_CREATE, 0, 0);
  return pdPASS;
}

TickType_t xTaskGetTickCount(void) {
  return 0;
}

void systemWaitStart(void) {}

int deckCount(void) {
  return 1;
}

DeckInfo *deckInfo(int index) {
  TEST_ASSERT_EQUAL_INT(0, index);
  return &deck_info;
}

void memoryRegisterHandler(const MemoryHandlerDef_t *handler) {
  (void)handler;
}

void uart1Init(const uint32_t baudrate) {
  record_event(EVENT_UART_INIT, baudrate, 0);
}

void uart1SetBaudrate(const uint32_t baudrate) {
  (void)baudrate;
}

bool uart1GetDataWithTimeout(uint8_t *value, const uint32_t timeout_ticks) {
  (void)timeout_ticks;
  if (ack_index >= sizeof(ack_bytes)) {
    return false;
  }
  *value = ack_bytes[ack_index++];
  return true;
}

void uart1SendData(uint32_t length, uint8_t *data) {
  TEST_ASSERT_LESS_THAN_UINT8(sizeof(uart_sends) / sizeof(uart_sends[0]),
                              uart_send_count);
  TEST_ASSERT_LESS_OR_EQUAL_UINT32(sizeof(uart_sends[0].bytes), length);
  uart_sends[uart_send_count].length = length;
  memcpy(uart_sends[uart_send_count].bytes, data, length);
  uart_send_count++;
}

void uart1SendDmaIfAvailable(uint32_t length, uint8_t *data) {
  uart1SendData(length, data);
}

void setUp(void) {
  memset(&deck_info, 0, sizeof(deck_info));
  memset(events, 0, sizeof(events));
  memset(uart_sends, 0, sizeof(uart_sends));
  event_count = 0;
  uart_send_count = 0;
  ack_index = 0;
  bccam_deck_test_reset();
}

void tearDown(void) {}

static void assert_no_init_side_effects(void) {
  TEST_ASSERT_EQUAL_UINT8(0, event_count);
  TEST_ASSERT_FALSE(driver_bccam_deck->test());
}

void testNullDeckInfoIsRejectedWithoutSideEffects(void) {
  driver_bccam_deck->init(NULL);
  assert_no_init_side_effects();
}

void testMissingBoardRevisionIsRejectedWithoutSideEffects(void) {
  deck_info.boardRevision = NULL;
  driver_bccam_deck->init(&deck_info);
  assert_no_init_side_effects();
}

void testNonGBoardRevisionsAreRejectedWithoutSideEffects(void) {
  deck_info.boardRevision = "F";
  driver_bccam_deck->init(&deck_info);
  assert_no_init_side_effects();

  deck_info.boardRevision = "G2";
  driver_bccam_deck->init(&deck_info);
  assert_no_init_side_effects();
}

static void assert_event(uint8_t index, event_type_t type,
                         uint32_t first, uint32_t second) {
  TEST_ASSERT_EQUAL_INT(type, events[index].type);
  TEST_ASSERT_EQUAL_UINT32(first, events[index].first);
  TEST_ASSERT_EQUAL_UINT32(second, events[index].second);
}

void testRevisionGUsesRevGControlsAndOneMbaudBeforeStartingTask(void) {
  deck_info.boardRevision = "G";
  driver_bccam_deck->init(&deck_info);

  TEST_ASSERT_EQUAL_UINT8(12, event_count);
  assert_event(0, EVENT_GPIO_DIRECTION, DECKCTRL_GPIO_PIN_10, OUTPUT);
  assert_event(1, EVENT_GPIO_WRITE, DECKCTRL_GPIO_PIN_10, HIGH);
  assert_event(2, EVENT_DELAY, M2T(50), 0);
  assert_event(3, EVENT_GPIO_DIRECTION, DECKCTRL_GPIO_PIN_5, OUTPUT);
  assert_event(4, EVENT_GPIO_WRITE, DECKCTRL_GPIO_PIN_5, LOW);
  assert_event(5, EVENT_GPIO_DIRECTION, DECKCTRL_GPIO_PIN_0, OUTPUT);
  assert_event(6, EVENT_GPIO_WRITE, DECKCTRL_GPIO_PIN_0, HIGH);
  assert_event(7, EVENT_DELAY, M2T(100), 0);
  assert_event(8, EVENT_GPIO_DIRECTION, DECKCTRL_GPIO_PIN_11, OUTPUT);
  assert_event(9, EVENT_GPIO_WRITE, DECKCTRL_GPIO_PIN_11, HIGH);
  assert_event(10, EVENT_UART_INIT, TEST_FIRMWARE_BAUDRATE, 0);
  assert_event(11, EVENT_TASK_CREATE, 0, 0);
  TEST_ASSERT_TRUE(driver_bccam_deck->test());
}

void testDeckMemoryOffsetZeroUsesRomAddressZero(void) {
  const uint8_t image[] = { 1, 2, 3, 4 };

  deck_info.driver = driver_bccam_deck;
  *driver_bccam_deck->memoryDef->newFwSizeP = sizeof(image);
  bccam_deck_test_set_bootloader(true);

  TEST_ASSERT_TRUE(handleMemWrite(0, DECK_0_PRIMARY_MEMORY_BASE,
                                  sizeof(image), image));

  TEST_ASSERT_EQUAL_UINT8(4, uart_send_count);
  TEST_ASSERT_EQUAL_HEX8(ISP_CMD_FLASH_ERASE, uart_sends[0].bytes[0]);
  TEST_ASSERT_EQUAL_UINT32(8, uart_sends[1].length);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[1].bytes[0]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[1].bytes[1]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[1].bytes[2]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[1].bytes[3]);
  TEST_ASSERT_EQUAL_HEX8(ISP_CMD_FLASH_WRITE, uart_sends[2].bytes[0]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[2].bytes[4]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[2].bytes[5]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[2].bytes[6]);
  TEST_ASSERT_EQUAL_HEX8(0, uart_sends[2].bytes[7]);
}
