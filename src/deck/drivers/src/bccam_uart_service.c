#include "bccam_uart_service.h"

#include <stddef.h>
#include <string.h>

#include "bccam_bootloader_uart_client.h"
#include "bccam_deck_controller.h"
#include "bccam_firmware_uart_client.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "static_mem.h"
#include "system.h"
#include "uart1.h"

#define DEBUG_MODULE "BCCAM"
#include "debug.h"
#else
#define DEBUG_PRINT(...)
#endif

#define BCCAM_FW_STARTUP_TIMEOUT_MS 3000
#define BCCAM_UART_REQUEST_QUEUE_LENGTH 4
#define BCCAM_UART_RX_QUEUE_LENGTH 8
#define BCCAM_UART_RX_DRAIN_BUDGET BCCAM_UART_RX_QUEUE_LENGTH
#define BCCAM_UART_TASK_NAME "bcCamUart"
#define BCCAM_UART_TASK_STACKSIZE (configMINIMAL_STACK_SIZE * 4)
#define BCCAM_UART_TASK_PRI (tskIDLE_PRIORITY + 2)

typedef enum {
  BCCAM_UART_REQUEST_ENTER_BOOTLOADER,
  BCCAM_UART_REQUEST_RESET_TO_FIRMWARE,
  BCCAM_UART_REQUEST_FLASH_WRITE,
  BCCAM_UART_REQUEST_FLASH_READ,
} bccam_uart_request_type_t;

typedef struct {
  bccam_uart_request_type_t type;
  uint32_t address;
  uint8_t length;
  const uint8_t *write_data;
  uint8_t *read_data;
  uint32_t new_fw_size;
  bool *result_out;
  SemaphoreHandle_t done;
} bccam_uart_request_t;

static bccam_uart_service_state_t service_state =
  BCCAM_UART_SERVICE_STATE_UNINITIALIZED;

static bccam_deck_controller_t deck_controller;
static bccam_bootloader_uart_client_t bootloader_client;
static bccam_firmware_uart_client_t firmware_client;
static uint32_t firmware_startup_deadline_tick;
static uint32_t firmware_startup_reset_count;

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
static bool test_firmware_startup_result_forced;
static bccam_uart_firmware_startup_result_t test_firmware_startup_result;
static bool test_firmware_control_probe_phase_forced;
static bccam_uart_control_probe_phase_t test_firmware_control_probe_phase;
static uint16_t test_rx_post_drain_poll_count;
static uint16_t test_incompatible_report_count;
static bool test_bootloader_enter_result_forced;
static bool test_bootloader_enter_result;
#endif

#ifdef CONFIG_DECK_BCCAM_DEBUG
static bool control_probe_logged;
static uint16_t control_malformed_logged;
#endif

uint8_t bccam_uart_service_link_state_log;
static bccam_uart_service_status_t service_status_cache;

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static xQueueHandle request_queue;
static xQueueHandle rx_queue;
static TaskHandle_t bcCamUartTaskHandle;
static volatile BaseType_t rx_queue_overflow_pending;
typedef struct {
  BaseType_t *higher_priority_task_woken;
} bccam_uart_rx_collector_isr_context_t;
static bccam_uart_rx_collector_t rx_collector;
static bccam_uart_rx_collector_isr_context_t rx_collector_context;
STATIC_MEM_QUEUE_ALLOC(request_queue,
                       BCCAM_UART_REQUEST_QUEUE_LENGTH,
                       sizeof(bccam_uart_request_t));
STATIC_MEM_QUEUE_ALLOC(rx_queue,
                       BCCAM_UART_RX_QUEUE_LENGTH,
                       sizeof(bccam_uart_rx_event_t));
STATIC_MEM_TASK_ALLOC(bcCamUartTask, BCCAM_UART_TASK_STACKSIZE);
#endif

static void observe_firmware_client(
  bccam_firmware_uart_client_observation_t *observation) {
  bccam_firmware_uart_client_observe(&firmware_client, observation);
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  if (observation != NULL && test_firmware_startup_result_forced) {
    observation->startup_result = test_firmware_startup_result;
    if (test_firmware_startup_result == BCCAM_UART_FIRMWARE_STARTUP_READY) {
      observation->link_state = BCCAM_UART_LINK_ACTIVE;
      observation->control_probe_done = true;
      observation->control_probe_phase = BCCAM_UART_CONTROL_PROBE_DONE;
    } else if (test_firmware_startup_result ==
               BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE) {
      observation->link_state = BCCAM_UART_LINK_ACTIVE;
      observation->control_probe_phase =
        BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE;
    } else if (test_firmware_startup_result ==
               BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL) {
      observation->link_state = BCCAM_UART_LINK_FAULT;
    }
  }
  if (observation != NULL && test_firmware_control_probe_phase_forced) {
    observation->control_probe_phase = test_firmware_control_probe_phase;
    observation->control_probe_done =
      test_firmware_control_probe_phase == BCCAM_UART_CONTROL_PROBE_DONE;
  }
#endif
}

static void populate_status_from_live(bccam_uart_service_status_t *status) {
  if (status == NULL) {
    return;
  }

  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);

  status->service_state = service_state;
  status->link_state = (uint8_t)observation.link_state;
  status->tx_credit = observation.control_tx_credit;
  status->rx_slots = observation.control_rx_slots;
  status->negotiated_payload = observation.negotiated_payload;
  status->rx_frames = observation.counters.rx_frames;
  status->tx_frames = observation.counters.tx_frames;
  status->rx_bytes = observation.rx_bytes;
  status->tx_bytes = observation.tx_bytes;
  status->rx_crc_errors = observation.counters.rx_crc_errors;
  status->link_faults = observation.counters.link_faults;
  status->tx_flushes = observation.tx_flushes;
  status->last_error = observation.last_error;
  status->control_probe_done = observation.control_probe_done ? 1u : 0u;
  status->control_schema_count = observation.control_schema_module_count;
}

static void store_service_status_cache(
  const bccam_uart_service_status_t *status) {
  if (status == NULL) {
    return;
  }

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  taskENTER_CRITICAL();
#endif
  service_status_cache = *status;
  bccam_uart_service_link_state_log = service_status_cache.link_state;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  taskEXIT_CRITICAL();
#endif
}

static void update_service_status_cache(void) {
  bccam_uart_service_status_t status;
  populate_status_from_live(&status);
  store_service_status_cache(&status);
}

static bool is_firmware_state(bccam_uart_service_state_t state) {
  return state == BCCAM_UART_SERVICE_STATE_FW_RESETTING ||
         state == BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING ||
         state == BCCAM_UART_SERVICE_STATE_FW_ACTIVE ||
         state == BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE;
}

static bool is_rx_firmware_state(bccam_uart_service_state_t state) {
  return state == BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING ||
         state == BCCAM_UART_SERVICE_STATE_FW_ACTIVE ||
         state == BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE;
}

static bool is_isp_state(bccam_uart_service_state_t state) {
  return state == BCCAM_UART_SERVICE_STATE_ISP_ENTERING ||
         state == BCCAM_UART_SERVICE_STATE_ISP_READY ||
         state == BCCAM_UART_SERVICE_STATE_ISP_BUSY ||
         state == BCCAM_UART_SERVICE_STATE_ISP_COMPLETE;
}

static void reset_control_probe_logging(void);
static void reset_rx_queue_state(void);
static void enter_fw_resetting(void);

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static bool rx_collector_emit_from_isr(void *context,
                                       const bccam_uart_rx_event_t *event) {
  bccam_uart_rx_collector_isr_context_t *isr_context = context;
  BaseType_t *higher_priority_task_woken =
    (isr_context != NULL) ?
    isr_context->higher_priority_task_woken : NULL;

  return bccam_uart_service_submit_rx_event_from_isr(
    event,
    higher_priority_task_woken);
}

static void reset_rx_collector_for_firmware_mode(void) {
  rx_collector_context.higher_priority_task_woken = NULL;
  bccam_uart_rx_collector_init(&rx_collector,
                               rx_collector_emit_from_isr,
                               &rx_collector_context);
}

static void bccam_uart_rx_byte_from_isr(
  uint8_t byte,
  BaseType_t *higher_priority_task_woken) {
  rx_collector_context.higher_priority_task_woken =
    higher_priority_task_woken;
  bccam_uart_rx_collector_feed_byte(&rx_collector, byte);
  rx_collector_context.higher_priority_task_woken = NULL;
}

static void bccam_uart_error_from_isr(
  BaseType_t *higher_priority_task_woken) {
  rx_collector_context.higher_priority_task_woken =
    higher_priority_task_woken;
  bccam_uart_rx_collector_report_uart_error(&rx_collector);
  rx_collector_context.higher_priority_task_woken = NULL;
}

static void enable_firmware_rx_callbacks(void) {
  reset_rx_collector_for_firmware_mode();
  uart1SetCallbacks(bccam_uart_rx_byte_from_isr,
                    bccam_uart_error_from_isr);
}

static void disable_firmware_rx_callbacks(void) {
  uart1ClearCallbacks();
  rx_collector_context.higher_priority_task_woken = NULL;
}
#endif

static bool tick_has_reached(TickType_t now, TickType_t deadline) {
  return (int32_t)(now - deadline) >= 0;
}

static void suspend_firmware_link(void) {
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  disable_firmware_rx_callbacks();
  reset_rx_queue_state();
#endif
  bccam_firmware_uart_client_suspend(&firmware_client);
  firmware_startup_deadline_tick = 0;
  reset_control_probe_logging();
  update_service_status_cache();
}

static void reset_flash_session(void) {
  bccam_bootloader_uart_client_reset_flash_session(&bootloader_client);
}

static void reset_control_probe_logging(void) {
#ifdef CONFIG_DECK_BCCAM_DEBUG
  control_probe_logged = false;
  control_malformed_logged = 0;
#endif
}

static const char *uart_result_name(int result) {
  switch (result) {
    case BCCAM_UART_OK:
      return "OK";
    case BCCAM_UART_ERR_BUFFER_TOO_SMALL:
      return "BUFFER_TOO_SMALL";
    case BCCAM_UART_ERR_PAYLOAD_TOO_LONG:
      return "PAYLOAD_TOO_LONG";
    case BCCAM_UART_ERR_BAD_ARGUMENT:
      return "BAD_ARGUMENT";
    case BCCAM_UART_ERR_BAD_CRC:
      return "BAD_CRC";
    case BCCAM_UART_ERR_BAD_VERSION:
      return "BAD_VERSION";
    case BCCAM_UART_ERR_BAD_LENGTH:
      return "BAD_LENGTH";
    case BCCAM_UART_ERR_NO_CREDIT:
      return "NO_CREDIT";
    case BCCAM_UART_ERR_NOT_ACTIVE:
      return "NOT_ACTIVE";
    case BCCAM_UART_ERR_LINK_FAULT:
      return "LINK_FAULT";
    case BCCAM_UART_ERR_TRANSACTION_BUSY:
      return "TRANSACTION_BUSY";
    case BCCAM_UART_ERR_UNKNOWN_SERVICE:
      return "UNKNOWN_SERVICE";
    case BCCAM_UART_ERR_MALFORMED_MANAGEMENT:
      return "MALFORMED_MANAGEMENT";
  }

  return "UNKNOWN";
}

static const char *link_state_name(bccam_uart_link_state_t state) {
  switch (state) {
    case BCCAM_UART_LINK_INACTIVE:
      return "INACTIVE";
    case BCCAM_UART_LINK_ACTIVE:
      return "ACTIVE";
    case BCCAM_UART_LINK_FAULT:
      return "FAULT";
  }

  return "UNKNOWN";
}

static const char *control_probe_phase_name(
  bccam_uart_control_probe_phase_t phase) {
  switch (phase) {
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK:
      return "WAITING_FOR_LINK";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE:
      return "WAITING_FOR_CONTROL_SERVICE";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LOCAL_RX_CREDIT:
      return "WAITING_FOR_LOCAL_RX_CREDIT";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_TARGET_TX_CREDIT:
      return "WAITING_FOR_TARGET_TX_CREDIT";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_RESPONSE:
      return "WAITING_FOR_RESPONSE";
    case BCCAM_UART_CONTROL_PROBE_DONE:
      return "DONE";
  }

  return "UNKNOWN";
}

static const char *startup_result_name(
  bccam_uart_firmware_startup_result_t result) {
  switch (result) {
    case BCCAM_UART_FIRMWARE_STARTUP_WAITING:
      return "WAITING";
    case BCCAM_UART_FIRMWARE_STARTUP_READY:
      return "READY";
    case BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE:
      return "INCOMPATIBLE";
    case BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL:
      return "ABNORMAL";
  }

  return "UNKNOWN";
}

static void append_char(char *buffer,
                        size_t buffer_size,
                        size_t *position,
                        char value) {
  if (buffer == NULL || position == NULL || buffer_size == 0u) {
    return;
  }

  if (*position < buffer_size - 1u) {
    buffer[*position] = value;
  }
  (*position)++;
  buffer[(*position < buffer_size) ? *position : buffer_size - 1u] = '\0';
}

static void append_string(char *buffer,
                          size_t buffer_size,
                          size_t *position,
                          const char *value) {
  if (value == NULL) {
    value = "";
  }

  while (*value != '\0') {
    append_char(buffer, buffer_size, position, *value);
    value++;
  }
}

static void append_uint(char *buffer,
                        size_t buffer_size,
                        size_t *position,
                        uint32_t value) {
  char digits[10];
  size_t count = 0;

  do {
    digits[count++] = (char)('0' + (value % 10u));
    value /= 10u;
  } while (value > 0u && count < sizeof(digits));

  while (count > 0u) {
    append_char(buffer, buffer_size, position, digits[--count]);
  }
}

static void append_int(char *buffer,
                       size_t buffer_size,
                       size_t *position,
                       int32_t value) {
  if (value < 0) {
    const uint32_t magnitude = 0u - (uint32_t)value;
    append_char(buffer, buffer_size, position, '-');
    append_uint(buffer, buffer_size, position, magnitude);
  } else {
    append_uint(buffer, buffer_size, position, (uint32_t)value);
  }
}

static void append_key_value_string(char *buffer,
                                    size_t buffer_size,
                                    size_t *position,
                                    const char *key,
                                    const char *value) {
  append_string(buffer, buffer_size, position, ", ");
  append_string(buffer, buffer_size, position, key);
  append_char(buffer, buffer_size, position, '=');
  append_string(buffer, buffer_size, position, value);
}

static void append_key_value_uint(char *buffer,
                                  size_t buffer_size,
                                  size_t *position,
                                  const char *key,
                                  uint32_t value) {
  append_string(buffer, buffer_size, position, ", ");
  append_string(buffer, buffer_size, position, key);
  append_char(buffer, buffer_size, position, '=');
  append_uint(buffer, buffer_size, position, value);
}

static void format_poll_failure(
  char *buffer,
  size_t buffer_size,
  int result,
  const bccam_firmware_uart_client_observation_t *observation,
  bool include_debug_detail) {
  if (buffer == NULL || buffer_size == 0u) {
    return;
  }

  size_t position = 0;
  buffer[0] = '\0';
  append_string(buffer, buffer_size, &position, "UART link poll failed: ");
  append_string(buffer, buffer_size, &position, uart_result_name(result));

  if (!include_debug_detail) {
    return;
  }

  append_string(buffer, buffer_size, &position, " (");
  append_int(buffer, buffer_size, &position, result);
  append_char(buffer, buffer_size, &position, ')');

  if (observation == NULL) {
    return;
  }

  append_key_value_string(buffer,
                          buffer_size,
                          &position,
                          "link",
                          link_state_name(observation->link_state));
  append_key_value_string(buffer,
                          buffer_size,
                          &position,
                          "phase",
                          control_probe_phase_name(
                            observation->control_probe_phase));
  append_key_value_string(buffer,
                          buffer_size,
                          &position,
                          "startup",
                          startup_result_name(observation->startup_result));
  append_key_value_uint(buffer,
                        buffer_size,
                        &position,
                        "crc",
                        observation->counters.rx_crc_errors);
  append_key_value_uint(buffer,
                        buffer_size,
                        &position,
                        "mgmt",
                        observation->counters.rx_malformed_management_errors);
  append_key_value_uint(buffer,
                        buffer_size,
                        &position,
                        "control",
                        observation->control_malformed);
  append_key_value_uint(buffer,
                        buffer_size,
                        &position,
                        "resync",
                        observation->counters.rx_resyncs);
  append_key_value_uint(buffer,
                        buffer_size,
                        &position,
                        "faults",
                        observation->counters.link_faults);
}

static void log_poll_failure(
  int result,
  const bccam_firmware_uart_client_observation_t *observation) {
  char line[256];

#ifdef CONFIG_DECK_BCCAM_DEBUG
  format_poll_failure(line, sizeof(line), result, observation, true);
#else
  format_poll_failure(line, sizeof(line), result, observation, false);
#endif

  DEBUG_PRINT("%s\n", line);
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_uart_service_test_format_poll_failure(
  char *buffer,
  size_t buffer_size,
  int result,
  const bccam_firmware_uart_client_observation_t *observation,
  bool include_debug_detail) {
  format_poll_failure(buffer,
                      buffer_size,
                      result,
                      observation,
                      include_debug_detail);
}
#endif

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static void clear_rx_queue_overflow_pending(void) {
  taskENTER_CRITICAL();
  rx_queue_overflow_pending = pdFALSE;
  taskEXIT_CRITICAL();
}

static BaseType_t take_rx_queue_overflow_pending(void) {
  taskENTER_CRITICAL();
  const BaseType_t pending = rx_queue_overflow_pending;
  rx_queue_overflow_pending = pdFALSE;
  taskEXIT_CRITICAL();
  return pending;
}

static void set_rx_queue_overflow_pending_from_isr(void) {
  const UBaseType_t saved_interrupt_status = taskENTER_CRITICAL_FROM_ISR();
  rx_queue_overflow_pending = pdTRUE;
  taskEXIT_CRITICAL_FROM_ISR(saved_interrupt_status);
}
#endif

static void reset_rx_queue_state(void) {
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  clear_rx_queue_overflow_pending();
  if (rx_queue != NULL) {
    xQueueReset(rx_queue);
  }
#endif
}

static void enter_fw_resetting(void) {
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  disable_firmware_rx_callbacks();
#endif
  service_state = BCCAM_UART_SERVICE_STATE_FW_RESETTING;
  firmware_startup_deadline_tick = 0;
  reset_control_probe_logging();
  reset_rx_queue_state();
  update_service_status_cache();
}

static const char *startup_phase_message(void) {
  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);

  if (observation.link_state == BCCAM_UART_LINK_FAULT) {
    return "startup stopped because the UART link entered fault state.";
  }

  switch (observation.control_probe_phase) {
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK:
      return "startup stopped while waiting for Camera Link establishment or catalog enumeration.";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE:
      return "startup stopped because a compatible Control service was not advertised.";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LOCAL_RX_CREDIT:
      return "startup stopped while preparing local Control communication.";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_TARGET_TX_CREDIT:
      return "startup stopped while waiting for the camera deck to grant Control communication.";
    case BCCAM_UART_CONTROL_PROBE_WAITING_FOR_RESPONSE:
      return "startup stopped after sending the Control request, before a valid response was received.";
    case BCCAM_UART_CONTROL_PROBE_DONE:
      return "startup completed.";
  }

  return "startup stopped in an unknown state.";
}

static bool update_firmware_startup_progress(const char **phase_message) {
  if (phase_message != NULL) {
    *phase_message = NULL;
  }

  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);

  if (observation.startup_result == BCCAM_UART_FIRMWARE_STARTUP_READY) {
    firmware_startup_deadline_tick = 0;
    service_state = BCCAM_UART_SERVICE_STATE_FW_ACTIVE;
    return false;
  }

  if (observation.startup_result == BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL) {
    if (phase_message != NULL) {
      *phase_message = startup_phase_message();
    }
    enter_fw_resetting();
    return false;
  }

  if (observation.startup_result !=
      BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE) {
    return false;
  }

  if (phase_message != NULL) {
    *phase_message = startup_phase_message();
  }

  service_state = BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE;
  firmware_startup_deadline_tick = 0;
  reset_control_probe_logging();
  update_service_status_cache();
  return true;
}

#ifdef CONFIG_DECK_BCCAM_DEBUG
static void log_control_probe_progress(void) {
  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);

  if (!control_probe_logged && observation.control_probe_done) {
    const uint8_t module_count = observation.control_schema_module_count;

    DEBUG_PRINT("UART control schema modules: %u\n", module_count);
    for (uint8_t i = 0; i < module_count; i++) {
      const bccam_uart_control_schema_module_t *module =
        bccam_firmware_uart_client_control_schema_module(&firmware_client, i);
      if (module != NULL) {
        DEBUG_PRINT("UART control schema %s: %s %u.%u\n",
                    module->namespace,
                    module->contract_id,
                    module->major,
                    module->minor);
      }
    }
    control_probe_logged = true;
  }

  const uint16_t malformed = observation.control_malformed;
  if (malformed != control_malformed_logged) {
    DEBUG_PRINT("UART control malformed responses: %u\n", malformed);
    control_malformed_logged = malformed;
  }
}
#endif

static void start_firmware_establishment_at(TickType_t now) {
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  disable_firmware_rx_callbacks();
#endif
  reset_rx_queue_state();

  if (!bccam_firmware_uart_client_enter(&firmware_client, now)) {
    DEBUG_PRINT("Camera Link establishment start failed\n");
    enter_fw_resetting();
    return;
  }

  // Discard bytes captured while the target was held in reset, booting, or
  // while the firmware client drained the UART.
  reset_rx_queue_state();
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  enable_firmware_rx_callbacks();
#endif
  reset_control_probe_logging();
  service_state = BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING;
  firmware_startup_deadline_tick =
    (uint32_t)now + M2T(BCCAM_FW_STARTUP_TIMEOUT_MS);
  update_service_status_cache();
}

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static void start_firmware_establishment(void) {
  start_firmware_establishment_at(xTaskGetTickCount());
}
#endif

static bool handle_enter_bootloader_request(void) {
  if (service_state == BCCAM_UART_SERVICE_STATE_ISP_COMPLETE) {
    reset_flash_session();
    service_state = BCCAM_UART_SERVICE_STATE_ISP_ENTERING;
    return true;
  }

  if (!is_firmware_state(service_state)) {
    return false;
  }

  suspend_firmware_link();
  service_state = BCCAM_UART_SERVICE_STATE_ISP_ENTERING;
  return true;
}

static bool handle_reset_to_firmware_request(void) {
  if (service_state == BCCAM_UART_SERVICE_STATE_FW_RESETTING) {
    return true;
  }

  if (is_firmware_state(service_state)) {
    enter_fw_resetting();
    return true;
  }

  if (!is_isp_state(service_state)) {
    return false;
  }

  reset_flash_session();
  enter_fw_resetting();
  return true;
}

static bool flash_write_allowed(void) {
  return service_state == BCCAM_UART_SERVICE_STATE_ISP_READY;
}

static bool service_write_flash_now(uint32_t mem_addr,
                                    uint8_t write_len,
                                    const uint8_t *buffer,
                                    uint32_t new_fw_size) {
  if (!flash_write_allowed() || buffer == NULL) {
    return false;
  }

  service_state = BCCAM_UART_SERVICE_STATE_ISP_BUSY;
  update_service_status_cache();
  const bool result =
    bccam_bootloader_uart_client_write_flash(&bootloader_client,
                                             mem_addr,
                                             write_len,
                                             buffer,
                                             new_fw_size);
  if (result && bccam_bootloader_uart_client_flash_completed(&bootloader_client)) {
    service_state = BCCAM_UART_SERVICE_STATE_ISP_COMPLETE;
  } else {
    service_state = BCCAM_UART_SERVICE_STATE_ISP_READY;
  }
  update_service_status_cache();
  return result;
}

static bool service_read_flash_now(uint32_t mem_addr,
                                   uint8_t read_len,
                                   uint8_t *buffer) {
  if (service_state != BCCAM_UART_SERVICE_STATE_ISP_READY || buffer == NULL) {
    return false;
  }

  service_state = BCCAM_UART_SERVICE_STATE_ISP_BUSY;
  update_service_status_cache();
  const bool result =
    bccam_bootloader_uart_client_read_flash(&bootloader_client,
                                            mem_addr,
                                            read_len,
                                            buffer);
  service_state = BCCAM_UART_SERVICE_STATE_ISP_READY;
  update_service_status_cache();
  return result;
}

static void complete_request(bccam_uart_request_t *request, bool result) {
  if (request->result_out != NULL) {
    *request->result_out = result;
  }
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  if (request->done != NULL) {
    xSemaphoreGive(request->done);
  }
#endif
}

static void handle_request(bccam_uart_request_t *request) {
  bool result = false;

  switch (request->type) {
    case BCCAM_UART_REQUEST_ENTER_BOOTLOADER:
      result = handle_enter_bootloader_request();
      break;
    case BCCAM_UART_REQUEST_RESET_TO_FIRMWARE:
      result = handle_reset_to_firmware_request();
      break;
    case BCCAM_UART_REQUEST_FLASH_WRITE:
      result = service_write_flash_now(request->address,
                                       request->length,
                                       request->write_data,
                                       request->new_fw_size);
      break;
    case BCCAM_UART_REQUEST_FLASH_READ:
      result = service_read_flash_now(request->address,
                                      request->length,
                                      request->read_data);
      break;
  }

  complete_request(request, result);
}

static bool submit_async_request(const bccam_uart_request_t *request) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  (void)request;
  return false;
#else
  if (request_queue == NULL) {
    return false;
  }
  if (xQueueSend(request_queue, request, 0) != pdTRUE) {
    return false;
  }
  if (bcCamUartTaskHandle != NULL) {
    (void)xTaskNotifyGive(bcCamUartTaskHandle);
  }
  return true;
#endif
}

static bool submit_sync_request(bccam_uart_request_t *request) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  (void)request;
  return false;
#else
  StaticSemaphore_t done_buffer;
  bool result = false;
  request->done = xSemaphoreCreateBinaryStatic(&done_buffer);
  request->result_out = &result;

  if (!submit_async_request(request)) {
    return false;
  }

  xSemaphoreTake(request->done, portMAX_DELAY);
  return result;
#endif
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
static void populate_startup_report(
  bccam_uart_service_test_startup_recovery_report_t *report,
  bool incompatible,
  uint32_t timeout_ms,
  const char *phase_message) {
  if (report == NULL) {
    return;
  }

  report->timeout_ms = timeout_ms;
  report->phase_message = phase_message;
  report->incompatible = incompatible;
  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);
  report->reset_count = firmware_startup_reset_count;
  report->advertised_service_count = observation.advertised_service_count;
  memset(&report->control_descriptor, 0, sizeof(report->control_descriptor));
  report->control_descriptor_present =
    bccam_firmware_uart_client_control_descriptor(
      &firmware_client, &report->control_descriptor);
}
#endif

static void report_advertised_services(void) {
  bccam_firmware_uart_client_observation_t observation;
  bccam_uart_service_descriptor_t control;
  observe_firmware_client(&observation);

  DEBUG_PRINT("bcCam UART: advertised service count: %u\n",
              observation.advertised_service_count);
  if (bccam_firmware_uart_client_control_descriptor(&firmware_client, &control)) {
    DEBUG_PRINT("bcCam UART: compatible Control handle=%u version=%u.%u\n",
                control.handle, control.major, control.minor);
  }
}

static void report_firmware_startup_recovery(
  const char *phase_message,
  bccam_uart_control_probe_phase_t probe_phase) {
  DEBUG_PRINT("bcCam UART: Camera deck startup did not complete within %u ms; resetting camera deck processor and retrying.\n",
              BCCAM_FW_STARTUP_TIMEOUT_MS);
  if (phase_message != NULL) {
    DEBUG_PRINT("bcCam UART: %s\n", phase_message);
  }
  if (probe_phase == BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE) {
    report_advertised_services();
  }
}

static void report_firmware_startup_incompatible(const char *phase_message) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  test_incompatible_report_count++;
#endif
  DEBUG_PRINT("bcCam UART: Camera deck firmware is incompatible with this Crazyflie firmware; automatic recovery stopped.\n");
  if (phase_message != NULL) {
    DEBUG_PRINT("bcCam UART: %s\n", phase_message);
  }
  report_advertised_services();
}

static bool update_startup_recovery_watchdog(
  uint32_t now_ticks,
  const char **phase_message,
  bccam_uart_control_probe_phase_t *probe_phase) {
  if (phase_message != NULL) {
    *phase_message = NULL;
  }

  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);
  if (probe_phase != NULL) {
    *probe_phase = observation.control_probe_phase;
  }

  if (observation.startup_result == BCCAM_UART_FIRMWARE_STARTUP_READY ||
      observation.startup_result == BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE ||
      observation.startup_result == BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL) {
    firmware_startup_deadline_tick = 0;
    return false;
  }

  if (firmware_startup_deadline_tick == 0) {
    firmware_startup_deadline_tick =
      now_ticks + M2T(BCCAM_FW_STARTUP_TIMEOUT_MS);
    return false;
  }

  if (!tick_has_reached((TickType_t)now_ticks,
                        (TickType_t)firmware_startup_deadline_tick)) {
    return false;
  }

  firmware_startup_reset_count++;
  if (phase_message != NULL) {
    *phase_message = startup_phase_message();
  }
  enter_fw_resetting();
  return true;
}

static void poll_firmware_link_at(uint32_t now_ticks) {
  const bccam_uart_service_state_t previous_service_state = service_state;
  const int result =
    bccam_firmware_uart_client_poll(&firmware_client, now_ticks);

  bccam_firmware_uart_client_observation_t observation;
  observe_firmware_client(&observation);
  if (result != BCCAM_UART_OK) {
    log_poll_failure(result, &observation);
  }

  if (observation.link_state == BCCAM_UART_LINK_FAULT ||
      observation.startup_result == BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL) {
    DEBUG_PRINT("UART link fault, resetting QCC748\n");
    enter_fw_resetting();
    return;
  }

  if (observation.link_state == BCCAM_UART_LINK_INACTIVE &&
      (previous_service_state == BCCAM_UART_SERVICE_STATE_FW_ACTIVE ||
       previous_service_state == BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE)) {
    service_state = BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING;
    firmware_startup_deadline_tick =
      now_ticks + M2T(BCCAM_FW_STARTUP_TIMEOUT_MS);
  }

  if (observation.link_state == BCCAM_UART_LINK_ACTIVE &&
      previous_service_state != BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE) {
    service_state = BCCAM_UART_SERVICE_STATE_FW_ACTIVE;
#ifdef CONFIG_DECK_BCCAM_DEBUG
    log_control_probe_progress();
#endif

    const char *incompatible_phase_message = NULL;
    if (update_firmware_startup_progress(&incompatible_phase_message)) {
      report_firmware_startup_incompatible(incompatible_phase_message);
    }
  }

  const char *phase_message = NULL;
  bccam_uart_control_probe_phase_t probe_phase =
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  if (update_startup_recovery_watchdog(now_ticks,
                                       &phase_message,
                                       &probe_phase)) {
    report_firmware_startup_recovery(phase_message, probe_phase);
  }

  update_service_status_cache();
}

static bool handle_rx_event(const bccam_uart_rx_event_t *event) {
  if (event == NULL || !is_rx_firmware_state(service_state)) {
    return false;
  }

  const int result =
    bccam_firmware_uart_client_on_rx_event(&firmware_client, event);
  if (result != BCCAM_UART_OK ||
      bccam_uart_runtime_get_state(&firmware_client.runtime) ==
        BCCAM_UART_LINK_FAULT) {
    enter_fw_resetting();
    return true;
  }

  return true;
}

static void poll_after_rx_event_drain_at(uint32_t now_ticks) {
  if (!is_rx_firmware_state(service_state)) {
    return;
  }

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  test_rx_post_drain_poll_count++;
#endif
  poll_firmware_link_at(now_ticks);
}

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static bool handle_pending_rx_queue_overflow(uint32_t now_ticks,
                                             bool *handled_any) {
  if (take_rx_queue_overflow_pending() != pdTRUE) {
    return false;
  }

  const bccam_uart_rx_event_t overflow_event = {
    .type = BCCAM_UART_RX_EVENT_FAULT,
    .fault = BCCAM_UART_RX_FAULT_QUEUE_OVERFLOW,
  };

  const bool handled = handle_rx_event(&overflow_event);
  if (handled_any != NULL) {
    *handled_any = handled || *handled_any;
  }
  poll_after_rx_event_drain_at(now_ticks);
  return true;
}

static bool drain_rx_events(uint32_t now_ticks) {
  uint8_t drained = 0;
  bool handled_any = false;

  while (drained < BCCAM_UART_RX_DRAIN_BUDGET) {
    if (handle_pending_rx_queue_overflow(now_ticks, &handled_any)) {
      return true;
    }

    bccam_uart_rx_event_t event;
    if (xQueueReceive(rx_queue, &event, 0) != pdTRUE) {
      break;
    }

    if (handle_pending_rx_queue_overflow(now_ticks, &handled_any)) {
      return true;
    }

    handled_any = handle_rx_event(&event) || handled_any;
    drained++;
  }

  if (handle_pending_rx_queue_overflow(now_ticks, &handled_any)) {
    return true;
  }

  if (handled_any) {
    poll_after_rx_event_drain_at(now_ticks);
  }

  return drained > 0;
}

static void enter_firmware_mode(void) {
  start_firmware_establishment();
  DEBUG_PRINT("QCC748 in firmware mode\n");
}
#endif

static bool enter_bootloader_mode(void) {
  bool entered = false;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  disable_firmware_rx_callbacks();
#endif
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  if (test_bootloader_enter_result_forced) {
    entered = test_bootloader_enter_result;
  } else
#endif
  {
    entered = bccam_bootloader_uart_client_enter(&bootloader_client);
  }
  if (entered) {
    service_state = BCCAM_UART_SERVICE_STATE_ISP_READY;
  }
  return entered;
}

static void service_poll_once_at(uint32_t now_ticks) {
  if (service_state == BCCAM_UART_SERVICE_STATE_FW_RESETTING) {
    start_firmware_establishment_at((TickType_t)now_ticks);
    if (service_state == BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING) {
      poll_firmware_link_at(now_ticks);
    }
  } else if (service_state == BCCAM_UART_SERVICE_STATE_ISP_ENTERING) {
    if (!enter_bootloader_mode()) {
      enter_fw_resetting();
    }
  } else if (service_state == BCCAM_UART_SERVICE_STATE_FW_ESTABLISHING ||
             service_state == BCCAM_UART_SERVICE_STATE_FW_ACTIVE ||
             service_state == BCCAM_UART_SERVICE_STATE_FW_INCOMPATIBLE) {
    poll_firmware_link_at(now_ticks);
  }
}

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
static void bcCamUartTask(void *arg) {
  (void)arg;
  systemWaitStart();
  enter_firmware_mode();

  while (true) {
    bccam_uart_request_t request;
    bool did_work = false;
    const uint32_t now_ticks = (uint32_t)xTaskGetTickCount();

    if (xQueueReceive(request_queue, &request, 0) == pdTRUE) {
      handle_request(&request);
      update_service_status_cache();
      did_work = true;
    }

    const bool did_rx_work = drain_rx_events(now_ticks);
    if (did_rx_work) {
      update_service_status_cache();
    }
    did_work = did_rx_work || did_work;
    if (!did_rx_work) {
      service_poll_once_at(now_ticks);
    }
    update_service_status_cache();

    if (!did_work) {
      (void)ulTaskNotifyTake(pdTRUE, M2T(10));
    }
  }
}
#endif

void bccam_uart_service_init(DeckInfo *deck_info_arg) {
  bccam_deck_controller_init(&deck_controller, deck_info_arg);
  bccam_bootloader_uart_client_init(&bootloader_client, &deck_controller);
  bccam_firmware_uart_client_init(&firmware_client, &deck_controller);
  firmware_startup_deadline_tick = 0;
  firmware_startup_reset_count = 0;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  request_queue = STATIC_MEM_QUEUE_CREATE(request_queue);
  rx_queue = STATIC_MEM_QUEUE_CREATE(rx_queue);
  reset_rx_collector_for_firmware_mode();
  reset_rx_queue_state();
#endif
  enter_fw_resetting();
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  bcCamUartTaskHandle = STATIC_MEM_TASK_CREATE(bcCamUartTask,
                                               bcCamUartTask,
                                               BCCAM_UART_TASK_NAME,
                                               NULL,
                                               BCCAM_UART_TASK_PRI);
#endif
}

bool bccam_uart_service_submit_rx_event_from_isr(
  const bccam_uart_rx_event_t *event,
  BaseType_t *higher_priority_task_woken) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  (void)event;
  (void)higher_priority_task_woken;
  return false;
#else
  if (event == NULL || rx_queue == NULL) {
    return false;
  }

  BaseType_t local_task_woken = pdFALSE;
  BaseType_t *task_woken =
    (higher_priority_task_woken != NULL) ?
    higher_priority_task_woken : &local_task_woken;

  if (xQueueSendFromISR(rx_queue, event, task_woken) != pdTRUE) {
    set_rx_queue_overflow_pending_from_isr();
    if (bcCamUartTaskHandle != NULL) {
      vTaskNotifyGiveFromISR(bcCamUartTaskHandle, task_woken);
    }
    return false;
  }

  if (bcCamUartTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(bcCamUartTaskHandle, task_woken);
  }
  return true;
#endif
}

void bccam_uart_service_request_bootloader(void) {
  const bccam_uart_request_t request = {
    .type = BCCAM_UART_REQUEST_ENTER_BOOTLOADER,
  };
  (void)submit_async_request(&request);
}

void bccam_uart_service_request_firmware(void) {
  const bccam_uart_request_t request = {
    .type = BCCAM_UART_REQUEST_RESET_TO_FIRMWARE,
  };
  (void)submit_async_request(&request);
}

bool bccam_uart_service_write_flash(uint32_t mem_addr,
                                    uint8_t write_len,
                                    const uint8_t *buffer,
                                    uint32_t new_fw_size) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  return service_write_flash_now(mem_addr, write_len, buffer, new_fw_size);
#else
  bccam_uart_request_t request = {
    .type = BCCAM_UART_REQUEST_FLASH_WRITE,
    .address = mem_addr,
    .length = write_len,
    .write_data = buffer,
    .new_fw_size = new_fw_size,
  };
  return submit_sync_request(&request);
#endif
}

bool bccam_uart_service_read_flash(uint32_t mem_addr,
                                   uint8_t read_len,
                                   uint8_t *buffer) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  return service_read_flash_now(mem_addr, read_len, buffer);
#else
  bccam_uart_request_t request = {
    .type = BCCAM_UART_REQUEST_FLASH_READ,
    .address = mem_addr,
    .length = read_len,
    .read_data = buffer,
  };
  return submit_sync_request(&request);
#endif
}

bool bccam_uart_service_state_has_active_bootloader(
  bccam_uart_service_state_t state) {
  return state == BCCAM_UART_SERVICE_STATE_ISP_READY ||
         state == BCCAM_UART_SERVICE_STATE_ISP_BUSY;
}

bccam_uart_service_state_t bccam_uart_service_get_state(void) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  update_service_status_cache();
#endif
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  taskENTER_CRITICAL();
#endif
  const bccam_uart_service_state_t state = service_status_cache.service_state;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  taskEXIT_CRITICAL();
#endif
  return state;
}

void bccam_uart_service_get_status(bccam_uart_service_status_t *status) {
#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
  update_service_status_cache();
#endif
  if (status == NULL) {
    return;
  }

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  taskENTER_CRITICAL();
#endif
  *status = service_status_cache;
#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
  taskEXIT_CRITICAL();
#endif
}

#if defined(UNIT_TEST) || defined(UNIT_TEST_MODE)
void bccam_uart_service_test_reset(void) {
  service_state = BCCAM_UART_SERVICE_STATE_UNINITIALIZED;
  firmware_startup_deadline_tick = 0;
  firmware_startup_reset_count = 0;
  test_rx_post_drain_poll_count = 0;
  test_incompatible_report_count = 0;
  test_firmware_startup_result_forced = false;
  test_firmware_startup_result = BCCAM_UART_FIRMWARE_STARTUP_WAITING;
  test_firmware_control_probe_phase_forced = false;
  test_firmware_control_probe_phase =
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  test_bootloader_enter_result_forced = false;
  test_bootloader_enter_result = false;
  bccam_firmware_uart_client_init(&firmware_client, &deck_controller);
  bccam_firmware_uart_client_test_trace_reset();
  reset_flash_session();
  bccam_bootloader_uart_client_init(&bootloader_client, &deck_controller);
  update_service_status_cache();
}

void bccam_uart_service_test_set_state(bccam_uart_service_state_t state) {
  service_state = state;
  update_service_status_cache();
}

bool bccam_uart_service_test_handle_request(bccam_uart_service_test_request_t request) {
  bccam_uart_request_t service_request = { 0 };
  bool result = false;
  service_request.result_out = &result;

  switch (request) {
    case BCCAM_UART_SERVICE_TEST_REQ_ENTER_BOOTLOADER:
      service_request.type = BCCAM_UART_REQUEST_ENTER_BOOTLOADER;
      break;
    case BCCAM_UART_SERVICE_TEST_REQ_RESET_TO_FIRMWARE:
      service_request.type = BCCAM_UART_REQUEST_RESET_TO_FIRMWARE;
      break;
    default:
      return false;
  }

  handle_request(&service_request);
  update_service_status_cache();
  return result;
}

void bccam_uart_service_test_set_firmware_startup_result(
  bccam_uart_firmware_startup_result_t result) {
  test_firmware_startup_result_forced = true;
  test_firmware_startup_result = result;
}

void bccam_uart_service_test_set_firmware_control_probe_phase(
  bccam_uart_control_probe_phase_t phase) {
  test_firmware_control_probe_phase_forced = true;
  test_firmware_control_probe_phase = phase;
}

void bccam_uart_service_test_set_bootloader_enter_result(bool result) {
  test_bootloader_enter_result_forced = true;
  test_bootloader_enter_result = result;
}

void bccam_uart_service_test_poll_once(void) {
  service_poll_once_at(0);
}

void bccam_uart_service_test_handle_rx_event(
  const bccam_uart_rx_event_t *event) {
  if (handle_rx_event(event)) {
    poll_after_rx_event_drain_at(0);
  }
}

void bccam_uart_service_test_handle_rx_events(
  const bccam_uart_rx_event_t *events,
  uint8_t event_count) {
  bool handled_any = false;

  if (events == NULL) {
    return;
  }

  for (uint8_t i = 0; i < event_count; i++) {
    handled_any = handle_rx_event(&events[i]) || handled_any;
  }

  if (handled_any) {
    poll_after_rx_event_drain_at(0);
  }
}

uint16_t bccam_uart_service_test_rx_post_drain_poll_count(void) {
  return test_rx_post_drain_poll_count;
}

uint16_t bccam_uart_service_test_incompatible_report_count(void) {
  return test_incompatible_report_count;
}

bool bccam_uart_service_test_flash_write_allowed(void) {
  return flash_write_allowed();
}

void bccam_uart_service_test_start_firmware_establishment(void) {
  bccam_firmware_uart_client_init(&firmware_client, &deck_controller);
  start_firmware_establishment_at(0);
}

const char *bccam_uart_service_test_startup_phase_message(void) {
  return startup_phase_message();
}

bool bccam_uart_service_test_update_startup_watchdog(
  uint32_t now_ticks,
  bccam_uart_service_test_startup_recovery_report_t *report) {
  const char *phase_message = NULL;
  bccam_uart_control_probe_phase_t probe_phase =
    BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  const bool recovered =
    update_startup_recovery_watchdog(now_ticks, &phase_message, &probe_phase);
  populate_startup_report(report,
                          false,
                          recovered ? BCCAM_FW_STARTUP_TIMEOUT_MS : 0,
                          phase_message);

  return recovered;
}

void bccam_uart_service_test_update_startup_progress(
  bccam_uart_service_test_startup_recovery_report_t *report) {
  const char *phase_message = NULL;
  const bool incompatible = update_firmware_startup_progress(&phase_message);

  populate_startup_report(report, incompatible, 0, phase_message);
}
#endif
