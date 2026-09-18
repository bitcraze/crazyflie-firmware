#include "bccam_uart_runtime.h"

#include <stddef.h>
#include <string.h>

#define CONTROL_KIND_GET 0x01u
#define CONTROL_KIND_VALUE 0x81u
#define CONTROL_TYPE_BYTES 0x21u
#define CONTROL_SCHEMA_REQUEST_ID 0u

static const char control_schema_modules_path[] = "meta.schema_modules";
static const uint8_t control_schema_modules_request[] = {
  CONTROL_KIND_GET, CONTROL_SCHEMA_REQUEST_ID,
  sizeof(control_schema_modules_path) - 1u,
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

static void clear_control_state(bccam_uart_runtime_t *runtime) {
  runtime->control_service_bound = false;
  runtime->control_service_id = 0u;
  runtime->control_rx_credit_opened = false;
  runtime->control_request_sent = false;
  runtime->control_probe_done = false;
  runtime->control_rx_release_pending = false;
  runtime->control_rx_malformed = 0u;
  runtime->control_schema_module_count = 0u;
  memset(runtime->control_schema_modules, 0,
         sizeof(runtime->control_schema_modules));
}

static void reset_runtime_session(bccam_uart_runtime_t *runtime) {
  runtime->next_descriptor_ordinal = 0u;
  clear_control_state(runtime);
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
  if (count > BCCAM_UART_CONTROL_MAX_SCHEMA_MODULES) {
    return false;
  }
  memset(runtime->control_schema_modules, 0,
         sizeof(runtime->control_schema_modules));
  for (uint8_t i = 0u; i < count; i++) {
    bccam_uart_control_schema_module_t *module =
      &runtime->control_schema_modules[i];
    if (offset >= table_len) {
      return false;
    }
    const uint8_t namespace_len = table[offset++];
    if (namespace_len == 0u ||
        namespace_len >= BCCAM_UART_CONTROL_SCHEMA_NAMESPACE_MAX_LEN ||
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
        contract_len >= BCCAM_UART_CONTROL_SCHEMA_CONTRACT_ID_MAX_LEN ||
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
  runtime->control_schema_module_count = count;
  return true;
}

static bool handle_schema_response(bccam_uart_runtime_t *runtime,
                                   const uint8_t *payload,
                                   uint16_t payload_len) {
  if (runtime == NULL || payload == NULL || payload_len < 4u ||
      payload[0] != CONTROL_KIND_VALUE ||
      payload[1] != CONTROL_SCHEMA_REQUEST_ID ||
      payload[2] != CONTROL_TYPE_BYTES ||
      payload_len != (uint16_t)(4u + payload[3])) {
    return false;
  }
  return parse_schema_module_table(runtime, &payload[4], payload[3]);
}

static void refresh_control_binding(bccam_uart_runtime_t *runtime) {
  bccam_uart_service_descriptor_t descriptor;
  if (!bccam_uart_link_control_binding(&runtime->link, &descriptor)) {
    runtime->control_service_bound = false;
    runtime->control_service_id = 0u;
    return;
  }
  if (!runtime->control_service_bound ||
      runtime->control_service_id != descriptor.handle) {
    const uint16_t malformed = runtime->control_rx_malformed;
    clear_control_state(runtime);
    runtime->control_rx_malformed = malformed;
  }
  runtime->control_service_bound = true;
  runtime->control_service_id = descriptor.handle;
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

int bccam_uart_runtime_on_firmware_boot(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  bccam_uart_link_init(&runtime->link);
  reset_runtime_session(runtime);
  runtime->restart_requested = false;
  runtime->last_error = BCCAM_UART_OK;
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
    refresh_control_binding(runtime);
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

static int release_control_rx(bccam_uart_runtime_t *runtime) {
  const int result = bccam_uart_link_release_rx_slot(
    &runtime->link, runtime->control_service_id);
  if (result == BCCAM_UART_OK) {
    runtime->control_rx_release_pending = false;
    return BCCAM_UART_OK;
  }
  if (temporary_result(result)) {
    return BCCAM_UART_OK;
  }
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_step_control_probe(bccam_uart_runtime_t *runtime) {
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
  refresh_control_binding(runtime);
  if (!bccam_uart_link_catalog_complete(&runtime->link) ||
      !runtime->control_service_bound) {
    return BCCAM_UART_OK;
  }
  if (runtime->control_rx_release_pending) {
    const int result = release_control_rx(runtime);
    if (result != BCCAM_UART_OK || runtime->control_rx_release_pending) {
      return result;
    }
  }
  if (runtime->control_probe_done) {
    return BCCAM_UART_OK;
  }
  if (!runtime->control_rx_credit_opened) {
    const int result = bccam_uart_link_send_credit_update(
      &runtime->link, runtime->control_service_id, 1u);
    if (result == BCCAM_UART_OK) {
      runtime->control_rx_credit_opened = true;
      return BCCAM_UART_OK;
    }
    if (temporary_result(result)) {
      return BCCAM_UART_OK;
    }
    set_last_error(runtime, result);
    return result;
  }
  if (!runtime->control_request_sent &&
      bccam_uart_link_get_tx_credit(&runtime->link,
                                    runtime->control_service_id) > 0u) {
    const int result = bccam_uart_link_send_normal(
      &runtime->link, runtime->control_service_id,
      control_schema_modules_request, sizeof(control_schema_modules_request));
    if (result == BCCAM_UART_OK) {
      runtime->control_request_sent = true;
      return BCCAM_UART_OK;
    }
    if (temporary_result(result)) {
      return BCCAM_UART_OK;
    }
    set_last_error(runtime, result);
    return result;
  }
  uint8_t service = 0u;
  uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
  uint16_t payload_len = 0u;
  bool unit_present = false;
  const int result = bccam_uart_link_take_rx(&runtime->link, &service, payload,
                                              sizeof(payload), &payload_len,
                                              &unit_present);
  if (result != BCCAM_UART_OK) {
    set_last_error(runtime, result);
    return result;
  }
  if (!unit_present) {
    return BCCAM_UART_OK;
  }
  if (service == runtime->control_service_id && runtime->control_request_sent &&
      handle_schema_response(runtime, payload, payload_len)) {
    runtime->control_probe_done = true;
  } else {
    runtime->control_rx_malformed++;
  }
  runtime->control_rx_release_pending = true;
  return release_control_rx(runtime);
}

bool bccam_uart_runtime_control_probe_done(const bccam_uart_runtime_t *runtime) {
  return runtime != NULL && runtime->control_probe_done;
}

bool bccam_uart_runtime_control_service_bound(const bccam_uart_runtime_t *runtime) {
  return runtime != NULL && runtime->control_service_bound;
}

uint8_t bccam_uart_runtime_control_service_id(const bccam_uart_runtime_t *runtime) {
  return runtime == NULL || !runtime->control_service_bound ? 0u :
    runtime->control_service_id;
}

bccam_uart_control_probe_phase_t bccam_uart_runtime_control_probe_phase(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || runtime->link.state != BCCAM_UART_LINK_ACTIVE ||
      !bccam_uart_link_catalog_complete(&runtime->link)) {
    return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LINK;
  }
  if (!runtime->control_service_bound) {
    return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_CONTROL_SERVICE;
  }
  if (runtime->control_probe_done) {
    return BCCAM_UART_CONTROL_PROBE_DONE;
  }
  if (!runtime->control_rx_credit_opened) {
    return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_LOCAL_RX_CREDIT;
  }
  if (!runtime->control_request_sent) {
    return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_TARGET_TX_CREDIT;
  }
  return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_RESPONSE;
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
  if (!runtime->control_service_bound) {
    return BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE;
  }
  return runtime->control_probe_done ? BCCAM_UART_FIRMWARE_STARTUP_READY :
    BCCAM_UART_FIRMWARE_STARTUP_WAITING;
}

uint8_t bccam_uart_runtime_control_tx_credit(const bccam_uart_runtime_t *runtime) {
  return runtime == NULL || !runtime->control_service_bound ? 0u :
    bccam_uart_link_get_tx_credit(&runtime->link, runtime->control_service_id);
}

uint8_t bccam_uart_runtime_control_rx_slots(const bccam_uart_runtime_t *runtime) {
  return runtime == NULL || !runtime->control_service_bound ? 0u :
    bccam_uart_link_get_rx_advertised_credit(&runtime->link,
                                             runtime->control_service_id);
}

uint8_t bccam_uart_runtime_control_schema_module_count(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? 0u : runtime->control_schema_module_count;
}

const bccam_uart_control_schema_module_t *bccam_uart_runtime_control_schema_module(
  const bccam_uart_runtime_t *runtime, uint8_t index) {
  return runtime == NULL || index >= runtime->control_schema_module_count ? NULL :
    &runtime->control_schema_modules[index];
}

uint16_t bccam_uart_runtime_get_control_malformed_count(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? 0u : runtime->control_rx_malformed;
}

bccam_uart_link_state_t bccam_uart_runtime_get_state(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? BCCAM_UART_LINK_FAULT : runtime->link.state;
}

const bccam_uart_link_counters_t *bccam_uart_runtime_get_counters(
  const bccam_uart_runtime_t *runtime) {
  return runtime == NULL ? NULL : &runtime->link.counters;
}
