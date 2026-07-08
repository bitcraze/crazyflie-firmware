#include "bccam_uart_runtime.h"

#include <stddef.h>
#include <string.h>

#define CONTROL_KIND_GET 0x01u
#define CONTROL_KIND_VALUE 0x81u
#define CONTROL_TYPE_BYTES 0x21u
#define CONTROL_SCHEMA_REQUEST_ID 0u
#define CONTROL_SERVICE_MAJOR 1u
#define CONTROL_SERVICE_MINOR 0u

static const char control_contract_id[] = "bitcraze.control";
static const char control_schema_modules_path[] = "meta.schema_modules";
static const uint8_t control_schema_modules_request[] = {
  CONTROL_KIND_GET,
  CONTROL_SCHEMA_REQUEST_ID,
  sizeof(control_schema_modules_path) - 1u,
  'm', 'e', 't', 'a', '.', 's', 'c', 'h', 'e', 'm', 'a',
  '_', 'm', 'o', 'd', 'u', 'l', 'e', 's'
};

static void set_last_error(bccam_uart_runtime_t *runtime, int result) {
  if (runtime != NULL && result != BCCAM_UART_OK) {
    runtime->last_error = result;
  }
}

static bool is_temporary_link_result(int result) {
  return result == BCCAM_UART_ERR_TRANSACTION_BUSY ||
         result == BCCAM_UART_ERR_NO_CREDIT;
}

static void clear_control_probe_state(bccam_uart_runtime_t *runtime) {
  runtime->control_rx_credit_opened = false;
  runtime->control_request_sent = false;
  runtime->control_probe_done = false;
  runtime->control_rx_release_pending = false;
  runtime->control_rx_malformed = 0;
  runtime->control_schema_module_count = 0;
  memset(runtime->control_schema_modules, 0, sizeof(runtime->control_schema_modules));
}

static bool bind_control_service(bccam_uart_runtime_t *runtime) {
  uint8_t service_id = 0u;
  bool service_found;

  if (runtime == NULL) {
    return false;
  }

  service_found = bccam_uart_link_find_service_by_contract_version(&runtime->link,
                                                                   control_contract_id,
                                                                   CONTROL_SERVICE_MAJOR,
                                                                   CONTROL_SERVICE_MINOR,
                                                                   &service_id);
  if (!service_found) {
    runtime->control_service_bound = false;
    runtime->control_service_id = 0u;
    clear_control_probe_state(runtime);
    return false;
  }

  if (!runtime->control_service_bound || runtime->control_service_id != service_id) {
    clear_control_probe_state(runtime);
  }

  runtime->control_service_bound = true;
  runtime->control_service_id = service_id;
  return true;
}

static bool compatible_control_service_is_advertised(
  const bccam_uart_runtime_t *runtime) {
  uint8_t service_id = 0u;

  if (runtime == NULL) {
    return false;
  }

  return bccam_uart_link_find_service_by_contract_version(&runtime->link,
                                                          control_contract_id,
                                                          CONTROL_SERVICE_MAJOR,
                                                          CONTROL_SERVICE_MINOR,
                                                          &service_id);
}

static void reset_control_probe(bccam_uart_runtime_t *runtime) {
  runtime->control_service_bound = false;
  runtime->control_service_id = 0u;
  clear_control_probe_state(runtime);
}

static bool contains_nul(const uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (data[i] == '\0') {
      return true;
    }
  }
  return false;
}

static bool parse_schema_module_table(bccam_uart_runtime_t *runtime,
                                      const uint8_t *table,
                                      uint8_t table_len) {
  uint8_t offset = 0;
  uint8_t count;

  if (runtime == NULL || table == NULL || table_len < 1u) {
    return false;
  }

  count = table[offset++];
  if (count > BCCAM_UART_CONTROL_MAX_SCHEMA_MODULES) {
    return false;
  }

  memset(runtime->control_schema_modules, 0, sizeof(runtime->control_schema_modules));

  for (uint8_t i = 0; i < count; i++) {
    uint8_t namespace_len;
    uint8_t contract_id_len;
    bccam_uart_control_schema_module_t *module =
      &runtime->control_schema_modules[i];

    if (offset >= table_len) {
      return false;
    }
    namespace_len = table[offset++];
    if (namespace_len == 0u ||
        namespace_len >= BCCAM_UART_CONTROL_SCHEMA_NAMESPACE_MAX_LEN ||
        offset + namespace_len > table_len ||
        contains_nul(&table[offset], namespace_len)) {
      return false;
    }
    memcpy(module->namespace, &table[offset], namespace_len);
    module->namespace[namespace_len] = '\0';
    offset = (uint8_t)(offset + namespace_len);

    if (offset >= table_len) {
      return false;
    }
    contract_id_len = table[offset++];
    if (contract_id_len == 0u ||
        contract_id_len >= BCCAM_UART_CONTROL_SCHEMA_CONTRACT_ID_MAX_LEN ||
        offset + contract_id_len > table_len ||
        contains_nul(&table[offset], contract_id_len)) {
      return false;
    }
    memcpy(module->contract_id, &table[offset], contract_id_len);
    module->contract_id[contract_id_len] = '\0';
    offset = (uint8_t)(offset + contract_id_len);

    if (offset + 2u > table_len) {
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

static bool handle_schema_modules_response(bccam_uart_runtime_t *runtime,
                                           const uint8_t *payload,
                                           uint16_t payload_len) {
  uint8_t value_len;

  if (runtime == NULL || payload == NULL || payload_len < 4u) {
    return false;
  }

  value_len = payload[3];
  if (payload[0] != CONTROL_KIND_VALUE ||
      payload[1] != CONTROL_SCHEMA_REQUEST_ID ||
      payload[2] != CONTROL_TYPE_BYTES ||
      payload_len != (uint16_t)(4u + value_len)) {
    return false;
  }

  return parse_schema_module_table(runtime, &payload[4], value_len);
}

static bool schema_modules_response_is_well_formed(const uint8_t *payload,
                                                   uint16_t payload_len) {
  bccam_uart_runtime_t scratch;

  memset(&scratch, 0, sizeof(scratch));
  return handle_schema_modules_response(&scratch, payload, payload_len);
}

static bool dispatch_to_runtime_service(void *context,
                                        uint8_t service,
                                        const uint8_t *payload,
                                        uint16_t payload_len) {
  bccam_uart_runtime_t *runtime = (bccam_uart_runtime_t *)context;

  if (runtime == NULL) {
    return false;
  }

  if (runtime->control_service_bound &&
      service == runtime->control_service_id &&
      !runtime->control_probe_done) {
    if (!runtime->control_request_sent) {
      runtime->control_rx_malformed++;
      runtime->control_rx_release_pending = true;
      return true;
    }

    if (handle_schema_modules_response(runtime, payload, payload_len)) {
      runtime->control_probe_done = true;
    } else {
      runtime->control_rx_malformed++;
    }
    runtime->control_rx_release_pending = true;
    return true;
  }

  return bccam_uart_link_store_rx_payload(&runtime->link,
                                          service,
                                          payload,
                                          payload_len) == BCCAM_UART_OK;
}

static void force_runtime_fault(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }

  runtime->link.tx_pending = false;
  runtime->link.tx_len = 0;
  runtime->link.transaction_in_flight = false;
  runtime->link.pending_transaction_id = 0u;

  if (runtime->link.state != BCCAM_UART_LINK_FAULT) {
    runtime->link.state = BCCAM_UART_LINK_FAULT;
    runtime->link.counters.link_faults++;
  }
}

static int retry_control_rx_release(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || !runtime->control_service_bound) {
    return BCCAM_UART_OK;
  }

  const int result = bccam_uart_link_release_rx_slot(&runtime->link,
                                                     runtime->control_service_id);
  if (result == BCCAM_UART_OK) {
    runtime->control_rx_release_pending = false;
    return BCCAM_UART_OK;
  }

  if (is_temporary_link_result(result)) {
    return BCCAM_UART_OK;
  }

  set_last_error(runtime, result);
  return result;
}

void bccam_uart_runtime_init(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }

  runtime->rx_bytes = 0;
  runtime->tx_bytes = 0;
  runtime->tx_flushes = 0;
  runtime->last_error = BCCAM_UART_OK;
  reset_control_probe(runtime);
  bccam_uart_link_init(&runtime->link, BCCAM_UART_LINK_ROLE_INITIATOR);
}

int bccam_uart_runtime_on_firmware_boot(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  bccam_uart_link_init(&runtime->link, BCCAM_UART_LINK_ROLE_INITIATOR);
  runtime->last_error = BCCAM_UART_OK;
  reset_control_probe(runtime);

  const int result = bccam_uart_link_start_discovery(&runtime->link);
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_retry_discovery(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  const int result = bccam_uart_link_retry_discovery(&runtime->link);
  set_last_error(runtime, result);
  return result;
}

void bccam_uart_runtime_on_bootloader_enter(bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return;
  }

  bccam_uart_link_init(&runtime->link, BCCAM_UART_LINK_ROLE_INITIATOR);
  runtime->last_error = BCCAM_UART_OK;
  reset_control_probe(runtime);
}

int bccam_uart_runtime_receive_byte(bccam_uart_runtime_t *runtime, uint8_t byte) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  const int result = bccam_uart_link_receive_byte(&runtime->link, byte);
  runtime->rx_bytes++;
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_on_raw_frame(bccam_uart_runtime_t *runtime,
                                    const uint8_t *frame,
                                    uint16_t frame_len) {
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (frame == NULL) {
    set_last_error(runtime, BCCAM_UART_ERR_BAD_ARGUMENT);
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  const int result = bccam_uart_link_receive_raw_frame(&runtime->link,
                                                       frame,
                                                       frame_len,
                                                       dispatch_to_runtime_service,
                                                       runtime);
  runtime->rx_bytes = (uint16_t)(runtime->rx_bytes + frame_len);
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_on_rx_fault(bccam_uart_runtime_t *runtime,
                                   bccam_uart_rx_fault_t fault) {
  (void)fault;
  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  const int result = bccam_uart_link_enter_fault(&runtime->link);
  force_runtime_fault(runtime);
  set_last_error(runtime, result);
  return result;
}

int bccam_uart_runtime_flush_tx(bccam_uart_runtime_t *runtime,
                                bccam_uart_runtime_send_t send,
                                void *send_context) {
  uint8_t frame[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
  size_t frame_len = 0;

  if (runtime == NULL || send == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  const int result = bccam_uart_link_take_tx(&runtime->link,
                                             frame,
                                             sizeof(frame),
                                             &frame_len);
  if (result != BCCAM_UART_OK) {
    set_last_error(runtime, result);
    return result;
  }

  if (frame_len > 0u) {
    send(send_context, frame, (uint32_t)frame_len);
    runtime->tx_bytes = (uint16_t)(runtime->tx_bytes + frame_len);
    runtime->tx_flushes++;
  }

  return BCCAM_UART_OK;
}

int bccam_uart_runtime_pump_tx(bccam_uart_runtime_t *runtime,
                               bccam_uart_runtime_send_t send,
                               void *send_context) {
  return bccam_uart_runtime_flush_tx(runtime, send, send_context);
}

int bccam_uart_runtime_step_control_probe(bccam_uart_runtime_t *runtime) {
  uint8_t service = 0;
  uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
  uint16_t payload_len = 0;
  int result;
  bool request_was_sent_before_step;

  if (runtime == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }

  if (bccam_uart_link_get_state(&runtime->link) != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_OK;
  }

  if (!bind_control_service(runtime)) {
    return BCCAM_UART_OK;
  }

  if (runtime->control_rx_release_pending) {
    result = retry_control_rx_release(runtime);
    if (result != BCCAM_UART_OK || runtime->control_rx_release_pending) {
      return result;
    }
  }

  if (runtime->control_probe_done) {
    return BCCAM_UART_OK;
  }

  request_was_sent_before_step = runtime->control_request_sent;

  if (!runtime->control_rx_credit_opened) {
    result = bccam_uart_link_send_credit_update(&runtime->link,
                                                runtime->control_service_id,
                                                1);
    if (result == BCCAM_UART_OK) {
      runtime->control_rx_credit_opened = true;
    } else if (is_temporary_link_result(result)) {
      return BCCAM_UART_OK;
    } else {
      set_last_error(runtime, result);
      return result;
    }
  }

  if (!runtime->control_request_sent &&
      bccam_uart_link_get_tx_credit(&runtime->link,
                                    runtime->control_service_id) > 0u) {
    result = bccam_uart_link_send_normal(&runtime->link,
                                         runtime->control_service_id,
                                         control_schema_modules_request,
                                         sizeof(control_schema_modules_request));
    if (result == BCCAM_UART_OK) {
      runtime->control_request_sent = true;
      if (!request_was_sent_before_step &&
          runtime->link.rx_pending &&
          runtime->link.rx_service == runtime->control_service_id &&
          schema_modules_response_is_well_formed(runtime->link.rx_payload,
                                                 runtime->link.rx_payload_len)) {
        result = bccam_uart_link_take_rx(&runtime->link,
                                         &service,
                                         payload,
                                         sizeof(payload),
                                         &payload_len);
        if (result != BCCAM_UART_OK) {
          set_last_error(runtime, result);
          return result;
        }
        runtime->control_probe_done = false;
        runtime->control_rx_malformed++;
        runtime->control_rx_release_pending = true;
      }
      return BCCAM_UART_OK;
    } else if (is_temporary_link_result(result)) {
      return BCCAM_UART_OK;
    } else {
      set_last_error(runtime, result);
      return result;
    }
  }

  result = bccam_uart_link_take_rx(&runtime->link,
                                   &service,
                                   payload,
                                   sizeof(payload),
                                   &payload_len);
  if (result != BCCAM_UART_OK) {
    if (!is_temporary_link_result(result)) {
      set_last_error(runtime, result);
    }
    return is_temporary_link_result(result) ? BCCAM_UART_OK : result;
  }

  if (service != runtime->control_service_id) {
    return BCCAM_UART_OK;
  }

  if (request_was_sent_before_step &&
      handle_schema_modules_response(runtime, payload, payload_len)) {
    runtime->control_probe_done = true;
  } else {
    runtime->control_rx_malformed++;
  }

  runtime->control_rx_release_pending = true;
  return retry_control_rx_release(runtime);
}

bool bccam_uart_runtime_control_probe_done(const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return false;
  }

  return runtime->control_probe_done;
}

bool bccam_uart_runtime_control_service_bound(const bccam_uart_runtime_t *runtime) {
  return runtime != NULL && runtime->control_service_bound;
}

uint8_t bccam_uart_runtime_control_service_id(const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || !runtime->control_service_bound) {
    return 0u;
  }

  return runtime->control_service_id;
}

bccam_uart_control_probe_phase_t bccam_uart_runtime_control_probe_phase(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL ||
      bccam_uart_link_get_state(&runtime->link) != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_CONTROL_PROBE_WAITING_FOR_DISCOVERY;
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
  if (runtime == NULL) {
    return BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL;
  }

  const bccam_uart_link_state_t link_state =
    bccam_uart_link_get_state(&runtime->link);

  if (link_state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_FIRMWARE_STARTUP_ABNORMAL;
  }

  if (link_state != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_FIRMWARE_STARTUP_WAITING;
  }

  if (runtime->control_probe_done) {
    return BCCAM_UART_FIRMWARE_STARTUP_READY;
  }

  if (!compatible_control_service_is_advertised(runtime)) {
    return BCCAM_UART_FIRMWARE_STARTUP_INCOMPATIBLE;
  }

  return BCCAM_UART_FIRMWARE_STARTUP_WAITING;
}

uint8_t bccam_uart_runtime_control_tx_credit(const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || !runtime->control_service_bound) {
    return 0u;
  }

  return bccam_uart_link_get_tx_credit(&runtime->link, runtime->control_service_id);
}

uint8_t bccam_uart_runtime_control_rx_slots(const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL || !runtime->control_service_bound) {
    return 0u;
  }

  return bccam_uart_link_get_rx_advertised_credit(&runtime->link,
                                                  runtime->control_service_id);
}

uint8_t bccam_uart_runtime_control_schema_module_count(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return 0;
  }

  return runtime->control_schema_module_count;
}

const bccam_uart_control_schema_module_t *
bccam_uart_runtime_control_schema_module(const bccam_uart_runtime_t *runtime,
                                         uint8_t index) {
  if (runtime == NULL || index >= runtime->control_schema_module_count) {
    return NULL;
  }

  return &runtime->control_schema_modules[index];
}

uint16_t bccam_uart_runtime_get_control_malformed_count(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return 0;
  }

  return runtime->control_rx_malformed;
}

bccam_uart_link_state_t bccam_uart_runtime_get_state(const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return BCCAM_UART_LINK_FAULT;
  }

  return bccam_uart_link_get_state(&runtime->link);
}

bool bccam_uart_runtime_service_is_discovered(const bccam_uart_runtime_t *runtime,
                                              uint8_t service) {
  if (runtime == NULL) {
    return false;
  }

  return bccam_uart_link_service_is_discovered(&runtime->link, service);
}

const bccam_uart_link_counters_t *bccam_uart_runtime_get_counters(
  const bccam_uart_runtime_t *runtime) {
  if (runtime == NULL) {
    return NULL;
  }

  return bccam_uart_link_get_counters(&runtime->link);
}
