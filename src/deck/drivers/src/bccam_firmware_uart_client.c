#include "bccam_firmware_uart_client.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "uart1.h"
#endif

#define BCCAM_UART_BAUDRATE       1000000
#define BCCAM_FW_RESET_HOLD_MS    50
#define BCCAM_FW_BOOT_SETUP_MS    50
// The Camera UART starts late in the QCC application boot sequence. Since
// ESTABLISH is intentionally not retried, wait until that UART is ready.
#define BCCAM_FW_BOOT_WAIT_MS     2000
#define BCCAM_FW_RX_DRAIN_MS      100
#define BCCAM_FW_RX_BYTES_PER_POLL 128
#define BCCAM_UART_TX_COMPLETE_TIMEOUT_MS 100

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
#define TEST_TRACE_MAX 192
#define TEST_RX_MAX 4096

static bccam_firmware_uart_client_test_trace_entry_t test_trace[TEST_TRACE_MAX];
static uint8_t test_trace_count;
static bool test_uart_send_result;
static uint8_t test_rx[TEST_RX_MAX];
static uint32_t test_rx_head;
static uint32_t test_rx_tail;

static bool firmware_uart_recv(uint8_t *byte, uint32_t timeout_ticks);

static bccam_firmware_uart_client_test_trace_entry_t *
test_trace_append_entry(bccam_firmware_uart_client_test_event_t event) {
  if (test_trace_count >= TEST_TRACE_MAX) {
    return NULL;
  }

  memset(&test_trace[test_trace_count], 0, sizeof(test_trace[test_trace_count]));
  test_trace[test_trace_count].event = event;
  return &test_trace[test_trace_count++];
}

void bccam_firmware_uart_client_test_trace_reset(void) {
  memset(test_trace, 0, sizeof(test_trace));
  memset(test_rx, 0, sizeof(test_rx));
  test_trace_count = 0;
  test_uart_send_result = true;
  test_rx_head = 0;
  test_rx_tail = 0;
  bccam_deck_controller_test_trace_reset();
}

void bccam_firmware_uart_client_test_set_uart_send_result(bool result) {
  test_uart_send_result = result;
}

uint8_t bccam_firmware_uart_client_test_trace_count(void) {
  return test_trace_count;
}

const bccam_firmware_uart_client_test_trace_entry_t *
bccam_firmware_uart_client_test_trace_entry(uint8_t index) {
  if (index >= test_trace_count) {
    return NULL;
  }

  return &test_trace[index];
}

void bccam_firmware_uart_client_test_queue_rx(const uint8_t *bytes,
                                              uint32_t length) {
  if (bytes == NULL) {
    return;
  }

  for (uint32_t i = 0; i < length && test_rx_tail < TEST_RX_MAX; i++) {
    test_rx[test_rx_tail++] = bytes[i];
  }
}

int bccam_firmware_uart_client_test_consume_rx(
  bccam_firmware_uart_client_t *client) {
  if (client == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  uint8_t byte;
  while (firmware_uart_recv(&byte, 0)) {
    const int result =
      bccam_uart_runtime_receive_byte(&client->runtime, byte);
    if (result != BCCAM_UART_OK) {
      return result;
    }
  }

  return BCCAM_UART_OK;
}

static void firmware_uart_set_baudrate(uint32_t baudrate) {
  bccam_firmware_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SET_BAUDRATE);
  if (entry != NULL) {
    entry->value = baudrate;
  }
}

static bool firmware_uart_send(uint32_t size, const uint8_t *data) {
  bccam_firmware_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_SEND);
  if (entry != NULL) {
    entry->length = size;
    if (data != NULL) {
      const uint32_t copy_len =
        (size <= sizeof(entry->bytes)) ? size : sizeof(entry->bytes);
      memcpy(entry->bytes, data, copy_len);
    }
  }
  return test_uart_send_result;
}

static bool firmware_uart_recv(uint8_t *byte, uint32_t timeout_ticks) {
  (void)timeout_ticks;
  if (byte == NULL || test_rx_head >= test_rx_tail) {
    return false;
  }

  *byte = test_rx[test_rx_head++];
  bccam_firmware_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_RECEIVE);
  if (entry != NULL) {
    entry->byte = *byte;
  }
  return true;
}

static void firmware_uart_init(void) {
}
#else
static bool tick_has_reached(TickType_t now, TickType_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static void firmware_uart_set_baudrate(uint32_t baudrate) {
  uart1SetBaudrate(baudrate);
}

static bool firmware_uart_send(uint32_t size, const uint8_t *data) {
  return uart1SendDataWaitComplete(
    size, data, M2T(BCCAM_UART_TX_COMPLETE_TIMEOUT_MS));
}

static bool firmware_uart_recv(uint8_t *byte, uint32_t timeout_ticks) {
  return uart1GetDataWithTimeout(byte, timeout_ticks);
}

static void firmware_uart_init(void) {
  uart1Init(BCCAM_UART_BAUDRATE);
}
#endif

static void firmware_drain_rx(uint32_t timeout_ms) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  (void)timeout_ms;
  test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_UART_DRAIN);
#else
  uint8_t dummy;
  const TickType_t end = xTaskGetTickCount() + M2T(timeout_ms);
  while (!tick_has_reached(xTaskGetTickCount(), end)) {
    if (!firmware_uart_recv(&dummy, M2T(5))) {
      break;
    }
  }
#endif
}

static bool firmware_deck_begin_boot(bccam_deck_controller_t *deck_controller,
                                     bccam_deck_boot_mode_t mode,
                                     uint32_t hold_ticks,
                                     uint32_t setup_ticks) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  bccam_firmware_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_BEGIN_BOOT);
  if (entry != NULL) {
    entry->boot_mode = mode;
  }
#endif
  return bccam_deck_controller_begin_boot(deck_controller,
                                          mode,
                                          hold_ticks,
                                          setup_ticks);
}

static bool firmware_deck_release_boot(bccam_deck_controller_t *deck_controller,
                                       uint32_t wait_ticks) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  bccam_firmware_uart_client_test_trace_entry_t *entry =
    test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_DECK_RELEASE_BOOT);
  if (entry != NULL) {
    entry->value = wait_ticks;
  }
#endif
  return bccam_deck_controller_release_boot(deck_controller, wait_ticks);
}

static bool send_firmware_frame(void *context,
                                const uint8_t *data,
                                uint32_t length) {
  (void)context;
  return firmware_uart_send(length, data);
}

void bccam_firmware_uart_client_init(
  bccam_firmware_uart_client_t *client,
  bccam_deck_controller_t *deck_controller) {
  if (client == NULL) {
    return;
  }

  client->deck_controller = deck_controller;
  bccam_uart_runtime_init(&client->runtime);
  firmware_uart_init();
}

bool bccam_firmware_uart_client_enter(
  bccam_firmware_uart_client_t *client,
  uint32_t now_ticks) {
  if (client == NULL || client->deck_controller == NULL) {
    return false;
  }

  if (!firmware_deck_begin_boot(client->deck_controller,
                                BCCAM_DECK_BOOT_FIRMWARE,
                                M2T(BCCAM_FW_RESET_HOLD_MS),
                                M2T(BCCAM_FW_BOOT_SETUP_MS))) {
    return false;
  }

  firmware_uart_set_baudrate(BCCAM_UART_BAUDRATE);

  if (!firmware_deck_release_boot(client->deck_controller,
                                  M2T(BCCAM_FW_BOOT_WAIT_MS))) {
    return false;
  }

  firmware_drain_rx(BCCAM_FW_RX_DRAIN_MS);

  const int result = bccam_uart_runtime_on_firmware_boot(&client->runtime);
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  test_trace_append_entry(BCCAM_FIRMWARE_UART_CLIENT_TEST_RUNTIME_START_ESTABLISHMENT);
#endif
  return result == BCCAM_UART_OK;
}

void bccam_firmware_uart_client_suspend(
  bccam_firmware_uart_client_t *client) {
  if (client == NULL) {
    return;
  }

  bccam_uart_runtime_on_bootloader_enter(&client->runtime);
}

int bccam_firmware_uart_client_on_rx_event(
  bccam_firmware_uart_client_t *client,
  const bccam_uart_rx_event_t *event) {
  if (client == NULL || event == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  switch (event->type) {
    case BCCAM_UART_RX_EVENT_FAULT:
      return bccam_uart_runtime_on_rx_fault(&client->runtime, event->fault);

    case BCCAM_UART_RX_EVENT_RAW_FRAME:
      if (event->raw_frame.length > BCCAM_UART_FRAME_MAX_ENCODED_SIZE) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
      }
      return bccam_uart_runtime_on_raw_frame(&client->runtime,
                                             event->raw_frame.bytes,
                                             event->raw_frame.length);

    default:
      return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
}

int bccam_firmware_uart_client_pump_tx(
  bccam_firmware_uart_client_t *client) {
  if (client == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  return bccam_uart_runtime_pump_tx(&client->runtime,
                                    send_firmware_frame,
                                    NULL);
}

int bccam_firmware_uart_client_poll(
  bccam_firmware_uart_client_t *client,
  uint32_t now_ticks) {
  if (client == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  (void)now_ticks;
  int result = bccam_firmware_uart_client_pump_tx(client);
  if (result != BCCAM_UART_OK) {
    return result;
  }
  if (bccam_uart_runtime_get_state(&client->runtime) == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_OK;
  }
  result = bccam_uart_runtime_step_control_probe(&client->runtime);
  if (result != BCCAM_UART_OK) {
    return result;
  }
  return bccam_firmware_uart_client_pump_tx(client);
}

bccam_uart_firmware_startup_result_t
bccam_firmware_uart_client_startup_result(
  const bccam_firmware_uart_client_t *client) {
  if (client == NULL) {
    return BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL;
  }

  return bccam_uart_runtime_firmware_startup_result(&client->runtime);
}

bccam_uart_control_probe_phase_t
bccam_firmware_uart_client_control_probe_phase(
  const bccam_firmware_uart_client_t *client) {
  if (client == NULL) {
    return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  }

  return bccam_uart_runtime_control_probe_phase(&client->runtime);
}

void bccam_firmware_uart_client_observe(
  const bccam_firmware_uart_client_t *client,
  bccam_firmware_uart_client_observation_t *observation) {
  if (observation == NULL) {
    return;
  }

  memset(observation, 0, sizeof(*observation));

  if (client == NULL) {
    observation->link_state = BCCAM_UART_LINK_FAULT;
    observation->startup_result = BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL;
    observation->control_probe_phase =
      BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
    observation->last_error = BCCAM_UART_ERR_BAD_ARGUMENT;
    return;
  }

  const bccam_uart_link_counters_t *counters =
    bccam_uart_runtime_get_counters(&client->runtime);
  if (counters != NULL) {
    observation->counters = *counters;
  }

  observation->link_state = bccam_uart_runtime_get_state(&client->runtime);
  observation->startup_result =
    bccam_uart_runtime_firmware_startup_result(&client->runtime);
  observation->control_probe_phase =
    bccam_uart_runtime_control_probe_phase(&client->runtime);
  observation->negotiated_payload =
    bccam_uart_link_get_negotiated_payload(&client->runtime.link);
  observation->rx_bytes = client->runtime.rx_bytes;
  observation->tx_bytes = client->runtime.tx_bytes;
  observation->tx_flushes = client->runtime.tx_flushes;
  observation->control_malformed =
    bccam_uart_runtime_get_control_malformed_count(&client->runtime);
  observation->last_error = (int32_t)client->runtime.last_error;
  observation->control_tx_credit =
    bccam_uart_runtime_control_tx_credit(&client->runtime);
  observation->control_rx_slots =
    bccam_uart_runtime_control_rx_slots(&client->runtime);
  observation->control_schema_module_count =
    bccam_uart_runtime_control_schema_module_count(&client->runtime);
  observation->advertised_service_count = client->runtime.link.service_count;
  observation->control_probe_done =
    bccam_uart_runtime_control_probe_done(&client->runtime);
}

bool bccam_firmware_uart_client_control_descriptor(
  const bccam_firmware_uart_client_t *client,
  bccam_uart_service_descriptor_t *descriptor) {
  return client != NULL &&
    bccam_uart_link_control_binding(&client->runtime.link, descriptor);
}

const bccam_uart_control_schema_module_t *
bccam_firmware_uart_client_control_schema_module(
  const bccam_firmware_uart_client_t *client,
  uint8_t index) {
  if (client == NULL) {
    return NULL;
  }

  return bccam_uart_runtime_control_schema_module(&client->runtime, index);
}
