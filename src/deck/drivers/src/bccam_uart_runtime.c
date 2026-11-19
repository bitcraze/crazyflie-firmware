#include "bccam_uart_runtime.h"

#include <stddef.h>
#include <string.h>

#if !defined(UNIT_TEST) && !defined(UNIT_TEST_MODE)
#include "FreeRTOS.h"
#include "task.h"
/* Protects apiservice_mailbox, shared by the API Service task and bcCamUartTask. */
#define APISERVICE_MAILBOX_LOCK() taskENTER_CRITICAL()
#define APISERVICE_MAILBOX_UNLOCK() taskEXIT_CRITICAL()
#else
#define APISERVICE_MAILBOX_LOCK()
#define APISERVICE_MAILBOX_UNLOCK()
#endif

#define APISERVICE_MAX_GRANTS_PENDING 127u

#define APISERVICE_SCHEMA_REQUEST_ID 0u

static const char apiservice_schema_modules_path[] = "meta.schema_modules";
static const uint8_t apiservice_schema_modules_request[] = {
  SYSLINK2_APISERVICE_KIND_GET, APISERVICE_SCHEMA_REQUEST_ID,
  sizeof(apiservice_schema_modules_path) - 1u,
  'm','e','t','a','.','s','c','h','e','m','a','_','m','o','d','u','l','e','s'
};

static void set_last_error(bccam_uart_runtime_t *runtime, int result) {
  if (runtime != NULL && result != BCCAM_UART_OK) {
    runtime->last_error = result;
  }
}

static bool temporary_result(int result) {
  return result == BCCAM_UART_ERR_TRANSACTION_BUSY ||
         result == BCCAM_UART_ERR_NO_CREDIT;
}

/** Empty the API Service mailbox and publish not ready. */
static void clear_apiservice_mailbox(bccam_uart_runtime_t *runtime) {
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  APISERVICE_MAILBOX_LOCK();
  if (mailbox->ready) {
    mailbox->ready_lost_unobserved = true;
  }
  mailbox->ready = false;
  mailbox->payload_mtu = 0u;
  mailbox->grants_pending = 0u;
  mailbox->tx_full = false;
  mailbox->rx_full = false;
  APISERVICE_MAILBOX_UNLOCK();
}

static void clear_apiservice_state(bccam_uart_runtime_t *runtime) {
  clear_apiservice_mailbox(runtime);
  runtime->apiservice_bound = false;
  runtime->apiservice_id = 0u;
  runtime->apiservice_rx_credit_opened = false;
  runtime->apiservice_request_sent = false;
  runtime->apiservice_probe_done = false;
  runtime->apiservice_rx_release_pending = false;
  runtime->apiservice_rx_malformed = 0u;
  runtime->apiservice_schema_module_count = 0u;
  memset(runtime->apiservice_schema_modules, 0,
         sizeof(runtime->apiservice_schema_modules));
}

static void reset_runtime_session(bccam_uart_runtime_t *runtime) {
  runtime->next_descriptor_ordinal = 0u;
  clear_apiservice_state(runtime);
  runtime->apiservice_mailbox.closed = false;
  bccam_uart_frame_parser_configure(&runtime->parser,
                                    BCCAM_UART_LINK_VERSION_1,
                                    BCCAM_UART_BOOTSTRAP_PAYLOAD_MTU);
}

static void sync_parser_counters(bccam_uart_runtime_t *runtime) {
  runtime->link.counters.rx_crc_errors = runtime->parser.rx_crc_errors;
  runtime->link.counters.rx_length_errors = runtime->parser.rx_length_errors;
  runtime->link.counters.rx_version_errors = runtime->parser.rx_version_errors;
  runtime->link.counters.rx_resyncs = runtime->parser.rx_resyncs;
}

static bool contains_nul(const uint8_t *data, uint8_t len) {
  for (uint8_t i = 0u; i < len; i++) {
    if (data[i] == 0u) {
      return true;
    }
  }
  return false;
}

static bool parse_schema_module_table(bccam_uart_runtime_t *runtime,
                                      const uint8_t *table,
                                      uint8_t table_len) {
  if (runtime == NULL || table == NULL || table_len < 1u) {
    return false;
  }
  uint8_t offset = 0u;
  const uint8_t count = table[offset++];
  if (count > BCCAM_UART_APISERVICE_MAX_SCHEMA_MODULES) {
    return false;
  }
  memset(runtime->apiservice_schema_modules, 0,
         sizeof(runtime->apiservice_schema_modules));
  for (uint8_t i = 0u; i < count; i++) {
    bccam_uart_apiservice_schema_module_t *module =
      &runtime->apiservice_schema_modules[i];
    if (offset >= table_len) {
      return false;
    }
    const uint8_t namespace_len = table[offset++];
    if (namespace_len == 0u ||
        namespace_len >= BCCAM_UART_APISERVICE_SCHEMA_NAMESPACE_MAX_LEN ||
        (uint16_t)offset + namespace_len > table_len ||
        contains_nul(&table[offset], namespace_len)) {
      return false;
    }
    memcpy(module->namespace, &table[offset], namespace_len);
    offset = (uint8_t)(offset + namespace_len);
    if (offset >= table_len) {
      return false;
    }
    const uint8_t contract_len = table[offset++];
    if (contract_len == 0u ||
        contract_len >= BCCAM_UART_APISERVICE_SCHEMA_CONTRACT_ID_MAX_LEN ||
        (uint16_t)offset + contract_len > table_len ||
        contains_nul(&table[offset], contract_len)) {
      return false;
    }
    memcpy(module->contract_id, &table[offset], contract_len);
    offset = (uint8_t)(offset + contract_len);
    if ((uint16_t)offset + 2u > table_len) {
      return false;
    }
    module->major = table[offset++];
    module->minor = table[offset++];
  }
  if (offset != table_len) {
    return false;
  }
  runtime->apiservice_schema_module_count = count;
  return true;
}

static bool handle_schema_response(bccam_uart_runtime_t *runtime,
                                   const uint8_t *payload,
                                   uint16_t payload_len) {
  if (runtime == NULL || payload == NULL || payload_len < 4u ||
      payload[0] != SYSLINK2_APISERVICE_KIND_VALUE ||
      payload[1] != APISERVICE_SCHEMA_REQUEST_ID ||
      payload[2] != SYSLINK2_APISERVICE_TYPE_BYTES ||
      payload_len != (uint16_t)(4u + payload[3])) {
    return false;
  }
  return parse_schema_module_table(runtime, &payload[4], payload[3]);
}

static void refresh_apiservice_binding(bccam_uart_runtime_t *runtime) {
  bccam_uart_service_descriptor_t descriptor;
  if (!bccam_uart_link_apiservice_binding(&runtime->link, &descriptor)) {
    runtime->apiservice_bound = false;
    runtime->apiservice_id = 0u;
    return;
  }
  if (!runtime->apiservice_bound ||
      runtime->apiservice_id != descriptor.handle) {
    const uint16_t malformed = runtime->apiservice_rx_malformed;
    clear_apiservice_state(runtime);
    runtime->apiservice_rx_malformed = malformed;
  }
  runtime->apiservice_bound = true;
  runtime->apiservice_id = descriptor.handle;
}

/** Resolve the compatible Console handle bound in the current Link session. */
static bool console_service_id(const bccam_uart_runtime_t *runtime,
                               uint8_t *service_id)
{
  bccam_uart_service_descriptor_t descriptor;
  if (runtime == NULL ||
      !bccam_uart_link_console_binding(&runtime->link, &descriptor)) {
    return false;
  }
  if (service_id != NULL) {
    *service_id = descriptor.handle;
  }
  return true;
}

void bccam_uart_runtime_init(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }
  memset(runtime, 0, sizeof(*runtime));
  bccam_uart_link_init(&runtime->link);
  bccam_uart_frame_parser_init(&runtime->parser,
                               BCCAM_UART_BOOTSTRAP_PAYLOAD_MTU);
  runtime->last_error = BCCAM_UART_OK;
}

static int begin_new_session(bccam_uart_runtime_t *runtime) {
  reset_runtime_session(runtime);
  runtime->restart_requested = false;
  const int result = bccam_uart_link_start_establishment(&runtime->link);
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_on_firmware_boot(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  bccam_uart_link_init(&runtime->link);
  runtime->last_error = BCCAM_UART_OK;
  return begin_new_session(runtime);
}

int bccam_uart_runtime_reestablish(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  (void)bccam_uart_link_clear_fault(&runtime->link);
  return begin_new_session(runtime);
}

int bccam_uart_runtime_retry_establishment(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (runtime->link.pending != BCCAM_UART_PENDING_ESTABLISH ||
      runtime->link.tx_pending) {
    return BCCAM_UART_OK;
  }
  const int result = bccam_uart_link_start_establishment(&runtime->link);
  set_last_error(runtime, result);
  return result;
}

void bccam_uart_runtime_on_bootloader_enter(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }
  bccam_uart_link_init(&runtime->link);
  reset_runtime_session(runtime);
  runtime->restart_requested = false;
  runtime->last_error = BCCAM_UART_OK;
}

static int receive_frame(bccam_uart_runtime_t *runtime,
                         const bccam_uart_frame_t *frame) {
  const bccam_uart_link_state_t before = runtime->link.state;
  const int result = bccam_uart_link_receive_unit(&runtime->link,
                                                   frame->link_version,
                                                   frame->service,
                                                   frame->payload,
                                                   frame->payload_len);
  if (result == BCCAM_UART_OK) {
    if (bccam_uart_link_boot_notice_received(&runtime->link)) {
      reset_runtime_session(runtime);
      runtime->restart_requested = true;
    } else if (before != BCCAM_UART_LINK_ACTIVE &&
               runtime->link.state == BCCAM_UART_LINK_ACTIVE) {
      bccam_uart_frame_parser_configure(&runtime->parser,
                                        runtime->link.active_link_version,
                                        runtime->link.negotiated_payload);
    }
    refresh_apiservice_binding(runtime);
  }
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_receive_byte(bccam_uart_runtime_t *runtime, uint8_t byte) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  bccam_uart_frame_t frame;
  bool ready = false;
  const int result = bccam_uart_frame_parser_feed(&runtime->parser, byte,
                                                   &frame, &ready);
  runtime->rx_bytes++;
  sync_parser_counters(runtime);
  if (result != BCCAM_UART_OK) {
    if (runtime->link.state == BCCAM_UART_LINK_ACTIVE) {
      (void)bccam_uart_link_enter_fault(&runtime->link);
    }
    set_last_error(runtime, result);
    return result;
  }
  return ready ? receive_frame(runtime, &frame) : BCCAM_UART_OK;
}

int bccam_uart_runtime_on_raw_frame(bccam_uart_runtime_t *runtime,
                                    const uint8_t *raw,
                                    uint16_t raw_len) {
  if (runtime == NULL || raw == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  bccam_uart_frame_parser_t parser;
  bccam_uart_frame_t frame;
  bool ready = false;
  const uint8_t version = runtime->link.state == BCCAM_UART_LINK_ACTIVE ?
    runtime->link.active_link_version : BCCAM_UART_LINK_VERSION_1;
  const uint16_t mtu = runtime->link.state == BCCAM_UART_LINK_ACTIVE ?
    runtime->link.negotiated_payload : BCCAM_UART_BOOTSTRAP_PAYLOAD_MTU;
  bccam_uart_frame_parser_init(&parser, mtu);
  bccam_uart_frame_parser_configure(&parser, version, mtu);
  int result = BCCAM_UART_OK;
  for (uint16_t i = 0u; i < raw_len; i++) {
    result = bccam_uart_frame_parser_feed(&parser, raw[i], &frame, &ready);
    if (result != BCCAM_UART_OK) {
      break;
    }
    if (ready && i != raw_len - 1u) {
      result = BCCAM_UART_ERR_BAD_LENGTH;
      ready = false;
      break;
    }
  }
  runtime->rx_bytes = (uint16_t)(runtime->rx_bytes + raw_len);
  if (result == BCCAM_UART_OK && !ready) {
    result = BCCAM_UART_ERR_BAD_LENGTH;
  }
  if (result != BCCAM_UART_OK) {
    runtime->link.counters.rx_crc_errors += parser.rx_crc_errors;
    runtime->link.counters.rx_length_errors += parser.rx_length_errors;
    runtime->link.counters.rx_version_errors += parser.rx_version_errors;
    runtime->link.counters.rx_resyncs += parser.rx_resyncs;
    if (runtime->link.state == BCCAM_UART_LINK_ACTIVE) {
      (void)bccam_uart_link_enter_fault(&runtime->link);
    }
    set_last_error(runtime, result);
    return result;
  }
  return receive_frame(runtime, &frame);
}

int bccam_uart_runtime_on_rx_fault(bccam_uart_runtime_t *runtime,
                                   bccam_uart_rx_fault_t fault) {
  (void)fault;
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (runtime->link.state != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_OK;
  }
  const int result = bccam_uart_link_enter_fault(&runtime->link);
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_flush_tx(bccam_uart_runtime_t *runtime,
                                bccam_uart_runtime_send_t send,
                                void *send_context) {
  if (runtime == NULL || send == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  const bool had_pending_unit = runtime->link.tx_pending;
  uint8_t version = 0u;
  uint8_t service = 0u;
  uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
  uint16_t payload_len = 0u;
  int result = bccam_uart_link_take_tx_unit(&runtime->link, &version, &service,
                                            payload, sizeof(payload),
                                            &payload_len);
  if (result != BCCAM_UART_OK || !had_pending_unit) {
    set_last_error(runtime, result);
    return result;
  }
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0u;
  result = bccam_uart_frame_encode_version(version, service, payload, payload_len,
                                           frame, sizeof(frame), &frame_len);
  if (result != BCCAM_UART_OK) {
    (void)bccam_uart_link_enter_fault(&runtime->link);
    set_last_error(runtime, result);
    return result;
  }
  if (!send(send_context, frame, (uint32_t)frame_len)) {
    result = bccam_uart_link_note_tx_failure(&runtime->link);
    set_last_error(runtime, result);
    return result;
  }
  runtime->tx_bytes = (uint16_t)(runtime->tx_bytes + frame_len);
  runtime->tx_flushes++;
  return BCCAM_UART_OK;
}

int bccam_uart_runtime_pump_tx(bccam_uart_runtime_t *runtime,
                               bccam_uart_runtime_send_t send,
                               void *send_context) {
  return bccam_uart_runtime_flush_tx(runtime, send, send_context);
}

static bool ordinal_seen(const bccam_uart_link_endpoint_t *link, uint8_t ordinal) {
  return (link->seen_ordinals[ordinal >> 3] &
          (uint8_t)(1u << (ordinal & 7u))) != 0u;
}

static int release_apiservice_rx(bccam_uart_runtime_t *runtime) {
  const int result = bccam_uart_link_release_rx_slot(
    &runtime->link, runtime->apiservice_id);
  if (result == BCCAM_UART_OK) {
    runtime->apiservice_rx_release_pending = false;
    return BCCAM_UART_OK;
  }
  if (temporary_result(result)) {
    return BCCAM_UART_OK;
  }
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_step_apiservice_probe(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (runtime->link.state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_OK;
  }
  if (runtime->link.state == BCCAM_UART_LINK_INACTIVE) {
    if (runtime->restart_requested &&
        runtime->link.pending == BCCAM_UART_PENDING_NONE &&
        !runtime->link.tx_pending) {
      runtime->restart_requested = false;
      bccam_uart_link_clear_boot_notice(&runtime->link);
      const int result = bccam_uart_link_start_establishment(&runtime->link);
      set_last_error(runtime, result);
      return result;
    }
    return BCCAM_UART_OK;
  }
  if (runtime->link.pending != BCCAM_UART_PENDING_NONE ||
      runtime->link.tx_pending) {
    return BCCAM_UART_OK;
  }
  if (!runtime->link.service_count_known) {
    const int result = bccam_uart_link_start_service_count(&runtime->link);
    set_last_error(runtime, result);
    return result;
  }
  while (runtime->next_descriptor_ordinal < runtime->link.service_count &&
         ordinal_seen(&runtime->link, runtime->next_descriptor_ordinal)) {
    runtime->next_descriptor_ordinal++;
  }
  if (runtime->next_descriptor_ordinal < runtime->link.service_count) {
    const int result = bccam_uart_link_start_service_descriptor(
      &runtime->link, runtime->next_descriptor_ordinal);
    set_last_error(runtime, result);
    return result;
  }
  refresh_apiservice_binding(runtime);
  if (!bccam_uart_link_catalog_complete(&runtime->link) ||
      !runtime->apiservice_bound) {
    return BCCAM_UART_OK;
  }
  if (runtime->apiservice_rx_release_pending) {
    const int result = release_apiservice_rx(runtime);
    if (result != BCCAM_UART_OK || runtime->apiservice_rx_release_pending) {
      return result;
    }
  }
  if (runtime->apiservice_probe_done) {
    return BCCAM_UART_OK;
  }
  if (!runtime->apiservice_rx_credit_opened) {
    const int result = bccam_uart_link_send_credit_update(
      &runtime->link, runtime->apiservice_id, 1u);
    if (result == BCCAM_UART_OK) {
      runtime->apiservice_rx_credit_opened = true;
      return BCCAM_UART_OK;
    }
    if (temporary_result(result)) {
      return BCCAM_UART_OK;
    }
    set_last_error(runtime, result);
    return result;
  }
  if (!runtime->apiservice_request_sent &&
      bccam_uart_link_get_tx_credit(&runtime->link,
                                    runtime->apiservice_id) > 0u) {
    const int result = bccam_uart_link_send_normal(
      &runtime->link, runtime->apiservice_id,
      apiservice_schema_modules_request, sizeof(apiservice_schema_modules_request));
    if (result == BCCAM_UART_OK) {
      runtime->apiservice_request_sent = true;
      return BCCAM_UART_OK;
    }
    if (temporary_result(result)) {
      return BCCAM_UART_OK;
    }
    set_last_error(runtime, result);
    return result;
  }
  uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
  uint16_t payload_len = 0u;
  bool unit_present = false;
  const int result = bccam_uart_link_take_rx_for_service(
    &runtime->link, runtime->apiservice_id, payload, sizeof(payload),
    &payload_len, &unit_present);
  if (result != BCCAM_UART_OK) {
    set_last_error(runtime, result);
    return result;
  }
  if (!unit_present) {
    return BCCAM_UART_OK;
  }
  if (runtime->apiservice_request_sent &&
      handle_schema_response(runtime, payload, payload_len)) {
    runtime->apiservice_probe_done = true;
    runtime->apiservice_rx_release_pending = false;
    return BCCAM_UART_OK;
  }
  runtime->apiservice_rx_malformed++;
  runtime->apiservice_rx_release_pending = true;
  return release_apiservice_rx(runtime);
}

bool bccam_uart_runtime_apiservice_probe_done(const bccam_uart_runtime_t *runtime) {
  return runtime != NULL && runtime->apiservice_probe_done;
}

bool bccam_uart_runtime_apiservice_bound(const bccam_uart_runtime_t *runtime) {
  return runtime != NULL && runtime->apiservice_bound;
}

bool bccam_uart_runtime_console_service_bound(const bccam_uart_runtime_t *runtime)
{
  return console_service_id(runtime, NULL);
}

int bccam_uart_runtime_open_console_rx(bccam_uart_runtime_t *runtime)
{
  uint8_t service = 0u;
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (!console_service_id(runtime, &service)) {
    return BCCAM_UART_ERR_UNKNOWN_SERVICE;
  }
  if (bccam_uart_link_get_rx_advertised_credit(&runtime->link, service) > 0u) {
    return BCCAM_UART_OK;
  }
  const int result = bccam_uart_link_send_credit_update(&runtime->link,
                                                         service, 1u);
  if (!temporary_result(result)) {
    set_last_error(runtime, result);
  }
  return result;
}

int bccam_uart_runtime_take_console_rx(bccam_uart_runtime_t *runtime,
                                       uint8_t *payload,
                                       size_t payload_capacity,
                                       uint16_t *payload_len,
                                       bool *unit_present)
{
  uint8_t console_service = 0u;
  if (payload_len == NULL || unit_present == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  *payload_len = 0u;
  *unit_present = false;
  if (runtime == NULL || payload == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (!console_service_id(runtime, &console_service)) {
    return BCCAM_UART_ERR_UNKNOWN_SERVICE;
  }
  return bccam_uart_link_take_rx_for_service(
    &runtime->link, console_service, payload, payload_capacity, payload_len,
    unit_present);
}

int bccam_uart_runtime_release_console_rx(bccam_uart_runtime_t *runtime)
{
  uint8_t service = 0u;
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (!console_service_id(runtime, &service)) {
    return BCCAM_UART_ERR_UNKNOWN_SERVICE;
  }
  const int result = bccam_uart_link_release_rx_slot(&runtime->link, service);
  if (!temporary_result(result)) {
    set_last_error(runtime, result);
  }
  return result;
}

uint8_t bccam_uart_runtime_apiservice_id(const bccam_uart_runtime_t *runtime) {
  return runtime == NULL || !runtime->apiservice_bound ? 0u :
    runtime->apiservice_id;
}

bccam_uart_apiservice_probe_phase_t bccam_uart_runtime_apiservice_probe_phase(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || runtime->link.state != BCCAM_UART_LINK_ACTIVE ||
      !bccam_uart_link_catalog_complete(&runtime->link)) {
    return BCCAM_UART_APISERVICE_PROBE_WAITING_FOR_LINK;
  }
  if (!runtime->apiservice_bound) {
    return BCCAM_UART_APISERVICE_PROBE_WAITING_FOR_APISERVICE;
  }
  if (runtime->apiservice_probe_done) {
    return BCCAM_UART_APISERVICE_PROBE_DONE;
  }
  if (!runtime->apiservice_rx_credit_opened) {
    return BCCAM_UART_APISERVICE_PROBE_WAITING_FOR_LOCAL_RX_CREDIT;
  }
  if (!runtime->apiservice_request_sent) {
    return BCCAM_UART_APISERVICE_PROBE_WAITING_FOR_TARGET_TX_CREDIT;
  }
  return BCCAM_UART_APISERVICE_PROBE_WAITING_FOR_RESPONSE;
}

bccam_uart_firmware_startup_result_t bccam_uart_runtime_firmware_startup_result(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || runtime->link.state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL;
  }
  if (runtime->link.state != BCCAM_UART_LINK_ACTIVE ||
      !bccam_uart_link_catalog_complete(&runtime->link)) {
    return BCCAM_UART_FIRMWARE_STARTUP_WAITING;
  }
  if (!runtime->apiservice_bound) {
    return BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE;
  }
  return runtime->apiservice_probe_done ? BCCAM_UART_FIRMWARE_STARTUP_READY :
    BCCAM_UART_FIRMWARE_STARTUP_WAITING;
}

uint8_t bccam_uart_runtime_apiservice_tx_credit(const bccam_uart_runtime_t *runtime) {
  return runtime == NULL || !runtime->apiservice_bound ? 0u :
    bccam_uart_link_get_tx_credit(&runtime->link, runtime->apiservice_id);
}

uint8_t bccam_uart_runtime_apiservice_rx_slots(const bccam_uart_runtime_t *runtime) {
  return runtime == NULL || !runtime->apiservice_bound ? 0u :
    bccam_uart_link_get_rx_advertised_credit(&runtime->link,
                                             runtime->apiservice_id);
}

uint8_t bccam_uart_runtime_apiservice_schema_module_count(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? 0u : runtime->apiservice_schema_module_count;
}

const bccam_uart_apiservice_schema_module_t *bccam_uart_runtime_apiservice_schema_module(
  const bccam_uart_runtime_t *runtime, uint8_t index) {
  return runtime == NULL || index >= runtime->apiservice_schema_module_count ? NULL :
    &runtime->apiservice_schema_modules[index];
}

uint16_t bccam_uart_runtime_get_apiservice_malformed_count(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? 0u : runtime->apiservice_rx_malformed;
}

bccam_uart_link_state_t bccam_uart_runtime_get_state(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? BCCAM_UART_LINK_FAULT : runtime->link.state;
}

const bccam_uart_link_counters_t *bccam_uart_runtime_get_counters(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? NULL : &runtime->link.counters;
}

/* API Service mailbox: API Service task side. */

bool bccam_uart_runtime_apiservice_ready(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return false;
  }
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  APISERVICE_MAILBOX_LOCK();
  const bool ready = mailbox->ready && !mailbox->ready_lost_unobserved;
  mailbox->ready_lost_unobserved = false;
  APISERVICE_MAILBOX_UNLOCK();
  return ready;
}

void bccam_uart_runtime_grant_apiservice_rx_credit(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  APISERVICE_MAILBOX_LOCK();
  if (mailbox->ready && mailbox->grants_pending < APISERVICE_MAX_GRANTS_PENDING) {
    mailbox->grants_pending++;
  }
  APISERVICE_MAILBOX_UNLOCK();
}

int bccam_uart_runtime_send_apiservice(bccam_uart_runtime_t *runtime,
                                    const syslink2_apiservice_frame_t *frame) {
  if (runtime == NULL || frame == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (frame->len > SYSLINK2_APISERVICE_FRAME_MAX_LEN) {
    return BCCAM_UART_ERR_PAYLOAD_TOO_LONG;
  }
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  int result = BCCAM_UART_OK;
  APISERVICE_MAILBOX_LOCK();
  if (!mailbox->ready) {
    result = BCCAM_UART_ERR_NOT_ACTIVE;
  } else if (frame->len > mailbox->payload_mtu) {
    result = BCCAM_UART_ERR_PAYLOAD_TOO_LONG;
  } else if (mailbox->tx_full) {
    result = BCCAM_UART_ERR_TRANSACTION_BUSY;
  } else {
    mailbox->tx_frame = *frame;
    mailbox->tx_full = true;
  }
  APISERVICE_MAILBOX_UNLOCK();
  return result;
}

bool bccam_uart_runtime_take_apiservice_rx(bccam_uart_runtime_t *runtime,
                                        syslink2_apiservice_frame_t *frame) {
  if (runtime == NULL || frame == NULL) {
    return false;
  }
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  bool taken = false;
  APISERVICE_MAILBOX_LOCK();
  if (mailbox->rx_full) {
    *frame = mailbox->rx_frame;
    mailbox->rx_full = false;
    taken = true;
  }
  APISERVICE_MAILBOX_UNLOCK();
  return taken;
}

/* API Service mailbox: bcCamUartTask side. */

void bccam_uart_runtime_close_apiservice(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }
  clear_apiservice_mailbox(runtime);
  runtime->apiservice_mailbox.closed = true;
}

static bool apiservice_link_ready(const bccam_uart_runtime_t *runtime) {
  return !runtime->apiservice_mailbox.closed &&
         runtime->link.state == BCCAM_UART_LINK_ACTIVE &&
         runtime->apiservice_bound && runtime->apiservice_probe_done;
}

void bccam_uart_runtime_stage_apiservice_rx(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || !apiservice_link_ready(runtime)) {
    return;
  }
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  APISERVICE_MAILBOX_LOCK();
  const bool rx_full = mailbox->rx_full;
  APISERVICE_MAILBOX_UNLOCK();
  if (rx_full) {
    /* Stays staged in the Link until the API Service task has taken the last. */
    return;
  }
  uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
  uint16_t payload_len = 0u;
  bool present = false;
  const int result = bccam_uart_link_take_rx_for_service(
    &runtime->link, runtime->apiservice_id, payload, sizeof(payload),
    &payload_len, &present);
  if (result != BCCAM_UART_OK || !present) {
    set_last_error(runtime, result);
    return;
  }
  if (payload_len > SYSLINK2_APISERVICE_FRAME_MAX_LEN) {
    /* Cannot be an API Service frame in scope. Drop it and return its slot. */
    runtime->apiservice_rx_malformed++;
    APISERVICE_MAILBOX_LOCK();
    if (mailbox->grants_pending < APISERVICE_MAX_GRANTS_PENDING) {
      mailbox->grants_pending++;
    }
    APISERVICE_MAILBOX_UNLOCK();
    return;
  }
  APISERVICE_MAILBOX_LOCK();
  memcpy(mailbox->rx_frame.data, payload, payload_len);
  mailbox->rx_frame.len = (uint8_t)payload_len;
  mailbox->rx_full = true;
  APISERVICE_MAILBOX_UNLOCK();
}

int bccam_uart_runtime_step_apiservice_mailbox(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  bccam_uart_apiservice_mailbox_t *mailbox = &runtime->apiservice_mailbox;
  if (!apiservice_link_ready(runtime)) {
    APISERVICE_MAILBOX_LOCK();
    const bool was_ready = mailbox->ready;
    APISERVICE_MAILBOX_UNLOCK();
    if (was_ready) {
      clear_apiservice_mailbox(runtime);
    }
    return BCCAM_UART_OK;
  }

  APISERVICE_MAILBOX_LOCK();
  mailbox->payload_mtu = runtime->link.negotiated_payload;
  mailbox->ready = true;
  const bool grant_pending = mailbox->grants_pending > 0u;
  APISERVICE_MAILBOX_UNLOCK();

  bccam_uart_runtime_stage_apiservice_rx(runtime);

  if (grant_pending) {
    const int result = bccam_uart_link_release_rx_slot(
      &runtime->link, runtime->apiservice_id);
    if (result == BCCAM_UART_OK) {
      APISERVICE_MAILBOX_LOCK();
      mailbox->grants_pending--;
      APISERVICE_MAILBOX_UNLOCK();
    } else if (!temporary_result(result)) {
      set_last_error(runtime, result);
      return result;
    }
    /* The Link has a single transmit slot: one unit per step. */
    return BCCAM_UART_OK;
  }

  syslink2_apiservice_frame_t frame;
  APISERVICE_MAILBOX_LOCK();
  const bool tx_full = mailbox->tx_full;
  if (tx_full) {
    frame = mailbox->tx_frame;
  }
  APISERVICE_MAILBOX_UNLOCK();
  if (!tx_full) {
    return BCCAM_UART_OK;
  }
  const int result = bccam_uart_link_send_normal(
    &runtime->link, runtime->apiservice_id, frame.data, frame.len);
  if (temporary_result(result)) {
    return BCCAM_UART_OK;
  }
  APISERVICE_MAILBOX_LOCK();
  mailbox->tx_full = false;
  APISERVICE_MAILBOX_UNLOCK();
  set_last_error(runtime, result);
  return result;
}
