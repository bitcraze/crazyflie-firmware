#include "bccam_uart_link.h"

#include <string.h>

static uint16_t read_le16(const uint8_t *in) {
  return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static uint8_t take_transaction_id(bccam_uart_link_endpoint_t *endpoint) {
  const uint8_t result = endpoint->next_transaction_id;
  endpoint->next_transaction_id++;
  if (endpoint->next_transaction_id == 0u) {
    endpoint->next_transaction_id = 1u;
  }
  return result;
}

static bool bit_is_set(const uint8_t bits[32], uint8_t value) {
  return (bits[value >> 3] & (uint8_t)(1u << (value & 7u))) != 0u;
}

static void set_bit(uint8_t bits[32], uint8_t value) {
  bits[value >> 3] |= (uint8_t)(1u << (value & 7u));
}

static void clear_session(bccam_uart_link_endpoint_t *endpoint) {
  endpoint->state = BCCAM_UART_LINK_INACTIVE;
  endpoint->active_link_version = 0u;
  endpoint->negotiated_payload = 0u;
  endpoint->required_link_mtu = 0u;
  endpoint->pending = BCCAM_UART_PENDING_NONE;
  endpoint->pending_transaction_id = 0u;
  endpoint->pending_descriptor_ordinal = 0u;
  endpoint->service_count_known = false;
  endpoint->service_count = 0u;
  endpoint->descriptors_received = 0u;
  memset(endpoint->seen_ordinals, 0, sizeof(endpoint->seen_ordinals));
  memset(endpoint->seen_handles, 0, sizeof(endpoint->seen_handles));
  memset(endpoint->pending_tx_credit, 0, sizeof(endpoint->pending_tx_credit));
  memset(endpoint->observed_credit_handles, 0,
         sizeof(endpoint->observed_credit_handles));
  memset(&endpoint->control_binding, 0, sizeof(endpoint->control_binding));
  endpoint->rx_pending = false;
  endpoint->rx_service = 0u;
  endpoint->rx_payload_len = 0u;
  endpoint->tx_pending = false;
  endpoint->tx_version = 0u;
  endpoint->tx_service = 0u;
  endpoint->tx_payload_len = 0u;
}

static void enter_fault(bccam_uart_link_endpoint_t *endpoint) {
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return;
  }
  clear_session(endpoint);
  endpoint->state = BCCAM_UART_LINK_FAULT;
  endpoint->counters.link_faults++;
}

static int peer_error(bccam_uart_link_endpoint_t *endpoint, int result) {
  if (endpoint->state == BCCAM_UART_LINK_ACTIVE) {
    enter_fault(endpoint);
  }
  return result;
}

static int malformed_management(bccam_uart_link_endpoint_t *endpoint) {
  endpoint->counters.rx_malformed_management_errors++;
  return peer_error(endpoint, BCCAM_UART_ERR_MALFORMED_MANAGEMENT);
}

static int queue_unit(bccam_uart_link_endpoint_t *endpoint,
                      uint8_t version,
                      uint8_t service,
                      const uint8_t *payload,
                      uint16_t payload_len) {
  if (endpoint == NULL || (payload_len > 0u && payload == NULL)) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_ERR_LINK_FAULT;
  }
  if (endpoint->tx_pending) {
    return BCCAM_UART_ERR_TRANSACTION_BUSY;
  }
  const uint16_t limit = endpoint->state == BCCAM_UART_LINK_ACTIVE ?
    endpoint->negotiated_payload : BCCAM_UART_BOOTSTRAP_PAYLOAD_MTU;
  if (payload_len > limit || payload_len > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
    return BCCAM_UART_ERR_PAYLOAD_TOO_LONG;
  }
  endpoint->tx_version = version;
  endpoint->tx_service = service;
  endpoint->tx_payload_len = payload_len;
  if (payload_len > 0u) {
    memcpy(endpoint->tx_payload, payload, payload_len);
  }
  endpoint->tx_pending = true;
  endpoint->counters.tx_frames++;
  return BCCAM_UART_OK;
}

static bool contract_id_valid(const uint8_t *id, uint8_t len) {
  bool segment_start = true;
  if (id == NULL || len == 0u || len > BCCAM_UART_SERVICE_CONTRACT_ID_MAX_LEN) {
    return false;
  }
  for (uint8_t i = 0u; i < len; i++) {
    const uint8_t ch = id[i];
    const bool lower = ch >= (uint8_t)'a' && ch <= (uint8_t)'z';
    const bool digit = ch >= (uint8_t)'0' && ch <= (uint8_t)'9';
    if (segment_start) {
      if (!lower && !digit) {
        return false;
      }
      segment_start = false;
    } else if (ch == (uint8_t)'.') {
      segment_start = true;
    } else if (!lower && !digit && ch != (uint8_t)'_') {
      return false;
    }
  }
  return !segment_start;
}

static bool tlvs_valid(const uint8_t *tlvs, uint16_t len) {
  uint16_t offset = 0u;
  while (offset < len) {
    if ((uint16_t)(len - offset) < 2u) {
      return false;
    }
    const uint8_t value_len = tlvs[offset + 1u];
    offset = (uint16_t)(offset + 2u);
    if (value_len > (uint16_t)(len - offset)) {
      return false;
    }
    offset = (uint16_t)(offset + value_len);
  }
  return true;
}

static bool is_control_contract(const uint8_t *id, uint8_t len) {
  static const uint8_t control[] = "bitcraze.control";
  return len == sizeof(control) - 1u && memcmp(id, control, len) == 0;
}

static bool observed_credit_handles_are_known(
  const bccam_uart_link_endpoint_t *endpoint) {
  for (size_t i = 0u; i < sizeof(endpoint->observed_credit_handles); i++) {
    if ((endpoint->observed_credit_handles[i] &
         (uint8_t)~endpoint->seen_handles[i]) != 0u) {
      return false;
    }
  }
  return true;
}

static int control_index(const bccam_uart_link_endpoint_t *endpoint,
                         uint8_t handle) {
  return endpoint != NULL && endpoint->control_binding.valid &&
         endpoint->control_binding.descriptor.handle == handle ? 0 : -1;
}

void bccam_uart_link_init(bccam_uart_link_endpoint_t *endpoint) {
  if (endpoint == NULL) {
    return;
  }
  memset(endpoint, 0, sizeof(*endpoint));
  endpoint->state = BCCAM_UART_LINK_INACTIVE;
  endpoint->next_transaction_id = 1u;
}

bccam_uart_link_state_t bccam_uart_link_get_state(
  const bccam_uart_link_endpoint_t *endpoint) {
  return endpoint == NULL ? BCCAM_UART_LINK_FAULT : endpoint->state;
}

uint16_t bccam_uart_link_get_negotiated_payload(
  const bccam_uart_link_endpoint_t *endpoint) {
  return endpoint == NULL ? 0u : endpoint->negotiated_payload;
}

const bccam_uart_link_counters_t *bccam_uart_link_get_counters(
  const bccam_uart_link_endpoint_t *endpoint) {
  return endpoint == NULL ? NULL : &endpoint->counters;
}

static int start_establishment_offer(bccam_uart_link_endpoint_t *endpoint,
                                     uint8_t min_version,
                                     uint8_t max_version,
                                     uint16_t max_mtu,
                                     uint16_t required_mtu) {
  if (endpoint == NULL || min_version == 0u || min_version > max_version) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_ERR_LINK_FAULT;
  }
  const uint8_t transaction_id = endpoint->next_transaction_id;
  const uint8_t payload[] = {
    BCCAM_UART_LINK_OP_ESTABLISH, transaction_id,
    min_version, max_version,
    (uint8_t)(max_mtu & 0xffu), (uint8_t)(max_mtu >> 8)
  };
  clear_session(endpoint);
  endpoint->establishment_rejected = false;
  endpoint->establishment_status = 0u;
  endpoint->boot_notice_received = false;
  endpoint->offered_min_version = min_version;
  endpoint->offered_max_version = max_version;
  endpoint->offered_max_mtu = max_mtu;
  endpoint->required_link_mtu = required_mtu;
  const int result = queue_unit(endpoint, BCCAM_UART_LINK_VERSION_1,
                                BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                payload, sizeof(payload));
  if (result != BCCAM_UART_OK) {
    return result;
  }
  endpoint->pending = BCCAM_UART_PENDING_ESTABLISH;
  endpoint->pending_transaction_id = take_transaction_id(endpoint);
  return BCCAM_UART_OK;
}

int bccam_uart_link_start_establishment_with(bccam_uart_link_endpoint_t *endpoint,
                                             uint8_t min_version,
                                             uint8_t max_version,
                                             uint16_t max_mtu) {
  if (min_version != BCCAM_UART_LINK_VERSION_1 ||
      max_version != BCCAM_UART_LINK_VERSION_1 ||
      max_mtu < BCCAM_UART_LINK_V1_MIN_MTU ||
      max_mtu > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  return start_establishment_offer(endpoint, min_version, max_version, max_mtu,
                                   BCCAM_UART_LINK_V1_MIN_MTU);
}

int bccam_uart_link_start_establishment(bccam_uart_link_endpoint_t *endpoint) {
  return start_establishment_offer(
    endpoint, BCCAM_UART_LINK_VERSION_1, BCCAM_UART_LINK_VERSION_1,
    BCCAM_UART_CAMERA_PROFILE_MTU, BCCAM_UART_CAMERA_PROFILE_MTU);
}

static int start_active_request(bccam_uart_link_endpoint_t *endpoint,
                                bccam_uart_pending_t pending,
                                uint8_t ordinal,
                                const uint8_t *payload,
                                uint16_t payload_len) {
  if (endpoint == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_ERR_LINK_FAULT;
  }
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_ERR_NOT_ACTIVE;
  }
  if (endpoint->pending != BCCAM_UART_PENDING_NONE) {
    return BCCAM_UART_ERR_TRANSACTION_BUSY;
  }
  const int result = queue_unit(endpoint, endpoint->active_link_version,
                                BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                payload, payload_len);
  if (result == BCCAM_UART_OK) {
    endpoint->pending = pending;
    endpoint->pending_transaction_id = payload[1];
    endpoint->pending_descriptor_ordinal = ordinal;
    (void)take_transaction_id(endpoint);
  }
  return result;
}

int bccam_uart_link_start_service_count(bccam_uart_link_endpoint_t *endpoint) {
  if (endpoint == NULL || endpoint->service_count_known) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  const uint8_t payload[] = {
    BCCAM_UART_LINK_OP_GET_SERVICE_COUNT, endpoint->next_transaction_id
  };
  return start_active_request(endpoint, BCCAM_UART_PENDING_SERVICE_COUNT, 0u,
                              payload, sizeof(payload));
}

int bccam_uart_link_start_service_descriptor(bccam_uart_link_endpoint_t *endpoint,
                                              uint8_t ordinal) {
  if (endpoint == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (!endpoint->service_count_known || ordinal >= endpoint->service_count ||
      bit_is_set(endpoint->seen_ordinals, ordinal)) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  const uint8_t payload[] = {
    BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR,
    endpoint->next_transaction_id, ordinal
  };
  return start_active_request(endpoint, BCCAM_UART_PENDING_SERVICE_DESCRIPTOR,
                              ordinal, payload, sizeof(payload));
}

int bccam_uart_link_send_normal(bccam_uart_link_endpoint_t *endpoint,
                                uint8_t service,
                                const uint8_t *payload,
                                uint16_t payload_len) {
  if (endpoint == NULL || (payload_len > 0u && payload == NULL)) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_ERR_LINK_FAULT;
  }
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_ERR_NOT_ACTIVE;
  }
  if (control_index(endpoint, service) < 0) {
    return BCCAM_UART_ERR_UNKNOWN_SERVICE;
  }
  if (payload_len > endpoint->negotiated_payload) {
    return BCCAM_UART_ERR_PAYLOAD_TOO_LONG;
  }
  if (endpoint->control_binding.tx_credit == 0u) {
    return BCCAM_UART_ERR_NO_CREDIT;
  }
  const int result = queue_unit(endpoint, endpoint->active_link_version,
                                service, payload, payload_len);
  if (result == BCCAM_UART_OK) {
    endpoint->control_binding.tx_credit--;
  }
  return result;
}

int bccam_uart_link_send_credit_update(bccam_uart_link_endpoint_t *endpoint,
                                       uint8_t service,
                                       uint8_t available_credit) {
  if (endpoint == NULL || available_credit > 127u) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_ERR_LINK_FAULT;
  }
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_ERR_NOT_ACTIVE;
  }
  if (control_index(endpoint, service) < 0) {
    return BCCAM_UART_ERR_UNKNOWN_SERVICE;
  }
  if (available_credit < endpoint->control_binding.rx_advertised_credit) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  const uint8_t payload[] = {
    BCCAM_UART_LINK_OP_CREDIT_UPDATE, 0u, service, available_credit
  };
  const int result = queue_unit(endpoint, endpoint->active_link_version,
                                BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                payload, sizeof(payload));
  if (result == BCCAM_UART_OK) {
    endpoint->control_binding.rx_advertised_credit = available_credit;
    endpoint->counters.credit_updates_tx++;
  }
  return result;
}

int bccam_uart_link_release_rx_slot(bccam_uart_link_endpoint_t *endpoint,
                                    uint8_t service) {
  if (endpoint == NULL || control_index(endpoint, service) < 0) {
    return BCCAM_UART_ERR_UNKNOWN_SERVICE;
  }
  if (endpoint->control_binding.rx_advertised_credit >= 127u) {
    return BCCAM_UART_OK;
  }
  return bccam_uart_link_send_credit_update(
    endpoint, service,
    (uint8_t)(endpoint->control_binding.rx_advertised_credit + 1u));
}

static int handle_establish_reply(bccam_uart_link_endpoint_t *endpoint,
                                  const uint8_t *payload,
                                  uint16_t payload_len) {
  if (payload_len != 6u || endpoint->pending != BCCAM_UART_PENDING_ESTABLISH ||
      payload[1] != endpoint->pending_transaction_id) {
    return malformed_management(endpoint);
  }
  const uint8_t status = payload[2];
  const uint8_t version = payload[3];
  const uint16_t mtu = read_le16(&payload[4]);
  endpoint->pending = BCCAM_UART_PENDING_NONE;
  endpoint->pending_transaction_id = 0u;
  endpoint->establishment_status = status;
  if (status != BCCAM_UART_ESTABLISH_ACCEPTED) {
    endpoint->establishment_rejected = true;
    endpoint->state = BCCAM_UART_LINK_INACTIVE;
    return BCCAM_UART_OK;
  }
  if (version < endpoint->offered_min_version ||
      version > endpoint->offered_max_version ||
      version != BCCAM_UART_LINK_VERSION_1 ||
      mtu < endpoint->required_link_mtu || mtu > endpoint->offered_max_mtu ||
      mtu > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
    return malformed_management(endpoint);
  }
  endpoint->state = BCCAM_UART_LINK_ACTIVE;
  endpoint->active_link_version = version;
  endpoint->negotiated_payload = mtu;
  return BCCAM_UART_OK;
}

static int handle_service_count(bccam_uart_link_endpoint_t *endpoint,
                                const uint8_t *payload,
                                uint16_t payload_len) {
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE || payload_len < 3u ||
      endpoint->pending != BCCAM_UART_PENDING_SERVICE_COUNT ||
      payload[1] != endpoint->pending_transaction_id ||
      !tlvs_valid(&payload[3], (uint16_t)(payload_len - 3u))) {
    return malformed_management(endpoint);
  }
  if (endpoint->service_count_known &&
      endpoint->service_count != payload[2]) {
    return malformed_management(endpoint);
  }
  endpoint->service_count_known = true;
  endpoint->service_count = payload[2];
  endpoint->pending = BCCAM_UART_PENDING_NONE;
  endpoint->pending_transaction_id = 0u;
  if (endpoint->service_count == 0u &&
      !observed_credit_handles_are_known(endpoint)) {
    return malformed_management(endpoint);
  }
  return BCCAM_UART_OK;
}

static int handle_service_descriptor(bccam_uart_link_endpoint_t *endpoint,
                                     const uint8_t *payload,
                                     uint16_t payload_len) {
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE || payload_len < 8u ||
      endpoint->pending != BCCAM_UART_PENDING_SERVICE_DESCRIPTOR ||
      payload[1] != endpoint->pending_transaction_id ||
      payload[2] != endpoint->pending_descriptor_ordinal ||
      !endpoint->service_count_known || payload[2] >= endpoint->service_count ||
      payload[3] == 0u || payload_len != (uint16_t)(7u + payload[6]) ||
      !contract_id_valid(&payload[7], payload[6]) ||
      (payload[4] == 0u && payload[5] != 0u)) {
    return malformed_management(endpoint);
  }
  const uint8_t ordinal = payload[2];
  const uint8_t handle = payload[3];
  const bool control_contract = is_control_contract(&payload[7], payload[6]);
  if (bit_is_set(endpoint->seen_ordinals, ordinal) ||
      bit_is_set(endpoint->seen_handles, handle)) {
    return malformed_management(endpoint);
  }
  set_bit(endpoint->seen_ordinals, ordinal);
  set_bit(endpoint->seen_handles, handle);
  endpoint->descriptors_received++;
  if (endpoint->descriptors_received == endpoint->service_count &&
      !observed_credit_handles_are_known(endpoint)) {
    return malformed_management(endpoint);
  }
  if (control_contract) {
    if (payload[4] == BCCAM_UART_CONTROL_CONTRACT_MAJOR &&
        !endpoint->control_binding.valid) {
      endpoint->control_binding.valid = true;
      endpoint->control_binding.descriptor.handle = handle;
      endpoint->control_binding.descriptor.major = payload[4];
      endpoint->control_binding.descriptor.minor = payload[5];
      endpoint->control_binding.descriptor.contract_id_len = payload[6];
      memcpy(endpoint->control_binding.descriptor.contract_id,
             &payload[7], payload[6]);
      endpoint->control_binding.tx_credit = endpoint->pending_tx_credit[handle];
      endpoint->pending_tx_credit[handle] = 0u;
    }
  }
  endpoint->pending = BCCAM_UART_PENDING_NONE;
  endpoint->pending_transaction_id = 0u;
  return BCCAM_UART_OK;
}

static int handle_credit_update(bccam_uart_link_endpoint_t *endpoint,
                                const uint8_t *payload,
                                uint16_t payload_len) {
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE || payload_len != 4u ||
      payload[1] != 0u || payload[2] == 0u || payload[3] > 127u) {
    return malformed_management(endpoint);
  }
  const uint8_t handle = payload[2];
  if (bccam_uart_link_catalog_complete(endpoint) &&
      !bit_is_set(endpoint->seen_handles, handle)) {
    return malformed_management(endpoint);
  }
  set_bit(endpoint->observed_credit_handles, handle);
  if (control_index(endpoint, handle) >= 0) {
    if (payload[3] < endpoint->control_binding.tx_credit) {
      return malformed_management(endpoint);
    }
    endpoint->control_binding.tx_credit = payload[3];
  } else {
    if (payload[3] < endpoint->pending_tx_credit[handle]) {
      return malformed_management(endpoint);
    }
    endpoint->pending_tx_credit[handle] = payload[3];
  }
  endpoint->counters.credit_updates_rx++;
  return BCCAM_UART_OK;
}

static int handle_management(bccam_uart_link_endpoint_t *endpoint,
                             const uint8_t *payload,
                             uint16_t payload_len) {
  if (payload_len < 2u) {
    return malformed_management(endpoint);
  }
  switch ((bccam_uart_link_op_t)payload[0]) {
    case BCCAM_UART_LINK_OP_ESTABLISH_REPLY:
      return handle_establish_reply(endpoint, payload, payload_len);
    case BCCAM_UART_LINK_OP_SERVICE_COUNT:
      return handle_service_count(endpoint, payload, payload_len);
    case BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR:
      return handle_service_descriptor(endpoint, payload, payload_len);
    case BCCAM_UART_LINK_OP_BOOT_NOTICE:
      if (payload_len != 2u || payload[1] != 0u) {
        return malformed_management(endpoint);
      }
      clear_session(endpoint);
      endpoint->boot_notice_received = true;
      return BCCAM_UART_OK;
    case BCCAM_UART_LINK_OP_CREDIT_UPDATE:
      return handle_credit_update(endpoint, payload, payload_len);
    default:
      return malformed_management(endpoint);
  }
}

int bccam_uart_link_receive_unit(bccam_uart_link_endpoint_t *endpoint,
                                 uint8_t link_version,
                                 uint8_t service,
                                 const uint8_t *payload,
                                 uint16_t payload_len) {
  if (endpoint == NULL || (payload_len > 0u && payload == NULL)) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (endpoint->state == BCCAM_UART_LINK_FAULT) {
    return BCCAM_UART_ERR_LINK_FAULT;
  }
  const uint8_t expected_version = endpoint->state == BCCAM_UART_LINK_ACTIVE ?
    endpoint->active_link_version : BCCAM_UART_LINK_VERSION_1;
  const uint16_t limit = endpoint->state == BCCAM_UART_LINK_ACTIVE ?
    endpoint->negotiated_payload : BCCAM_UART_BOOTSTRAP_PAYLOAD_MTU;
  if (link_version != expected_version) {
    endpoint->counters.rx_version_errors++;
    return peer_error(endpoint, BCCAM_UART_ERR_BAD_VERSION);
  }
  if (payload_len > limit) {
    endpoint->counters.rx_length_errors++;
    return peer_error(endpoint, BCCAM_UART_ERR_BAD_LENGTH);
  }
  endpoint->counters.rx_frames++;
  if (service == BCCAM_UART_SERVICE_LINK_MANAGEMENT) {
    return handle_management(endpoint, payload, payload_len);
  }
  if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
    return BCCAM_UART_ERR_NOT_ACTIVE;
  }
  if (control_index(endpoint, service) < 0) {
    endpoint->counters.rx_unknown_service_errors++;
    return peer_error(endpoint, BCCAM_UART_ERR_UNKNOWN_SERVICE);
  }
  if (endpoint->control_binding.rx_advertised_credit == 0u ||
      endpoint->rx_pending) {
    return peer_error(endpoint, BCCAM_UART_ERR_NO_CREDIT);
  }
  endpoint->control_binding.rx_advertised_credit--;
  endpoint->rx_pending = true;
  endpoint->rx_service = service;
  endpoint->rx_payload_len = payload_len;
  if (payload_len > 0u) {
    memcpy(endpoint->rx_payload, payload, payload_len);
  }
  return BCCAM_UART_OK;
}

int bccam_uart_link_take_tx_unit(bccam_uart_link_endpoint_t *endpoint,
                                 uint8_t *link_version,
                                 uint8_t *service,
                                 uint8_t *payload,
                                 size_t payload_capacity,
                                 uint16_t *payload_len) {
  if (payload_len == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  *payload_len = 0u;
  if (endpoint == NULL || link_version == NULL || service == NULL || payload == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (!endpoint->tx_pending) {
    return BCCAM_UART_OK;
  }
  if (payload_capacity < endpoint->tx_payload_len) {
    return BCCAM_UART_ERR_BUFFER_TOO_SMALL;
  }
  *link_version = endpoint->tx_version;
  *service = endpoint->tx_service;
  *payload_len = endpoint->tx_payload_len;
  memcpy(payload, endpoint->tx_payload, endpoint->tx_payload_len);
  endpoint->tx_pending = false;
  endpoint->tx_payload_len = 0u;
  return BCCAM_UART_OK;
}

int bccam_uart_link_take_rx(bccam_uart_link_endpoint_t *endpoint,
                            uint8_t *service,
                            uint8_t *out,
                            size_t out_capacity,
                            uint16_t *out_len,
                            bool *unit_present) {
  if (out_len == NULL || unit_present == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  *out_len = 0u;
  *unit_present = false;
  if (endpoint == NULL || service == NULL || out == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  if (!endpoint->rx_pending) {
    return BCCAM_UART_OK;
  }
  if (out_capacity < endpoint->rx_payload_len) {
    return BCCAM_UART_ERR_BUFFER_TOO_SMALL;
  }
  *service = endpoint->rx_service;
  *out_len = endpoint->rx_payload_len;
  *unit_present = true;
  memcpy(out, endpoint->rx_payload, endpoint->rx_payload_len);
  endpoint->rx_pending = false;
  endpoint->rx_payload_len = 0u;
  return BCCAM_UART_OK;
}

int bccam_uart_link_enter_fault(bccam_uart_link_endpoint_t *endpoint) {
  if (endpoint == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  enter_fault(endpoint);
  return BCCAM_UART_ERR_LINK_FAULT;
}

int bccam_uart_link_note_tx_failure(bccam_uart_link_endpoint_t *endpoint) {
  if (endpoint == NULL) {
    return BCCAM_UART_ERR_BAD_ARGUMENT;
  }
  endpoint->counters.tx_failures++;
  enter_fault(endpoint);
  return BCCAM_UART_ERR_LINK_FAULT;
}

bool bccam_uart_link_catalog_complete(const bccam_uart_link_endpoint_t *endpoint) {
  return endpoint != NULL && endpoint->state == BCCAM_UART_LINK_ACTIVE &&
         endpoint->service_count_known &&
         endpoint->descriptors_received == endpoint->service_count;
}

bool bccam_uart_link_control_binding(const bccam_uart_link_endpoint_t *endpoint,
                                     bccam_uart_service_descriptor_t *descriptor) {
  if (endpoint == NULL || !endpoint->control_binding.valid) {
    return false;
  }
  if (descriptor != NULL) {
    *descriptor = endpoint->control_binding.descriptor;
  }
  return true;
}

bool bccam_uart_link_boot_notice_received(
  const bccam_uart_link_endpoint_t *endpoint) {
  return endpoint != NULL && endpoint->boot_notice_received;
}

void bccam_uart_link_clear_boot_notice(bccam_uart_link_endpoint_t *endpoint) {
  if (endpoint != NULL) {
    endpoint->boot_notice_received = false;
  }
}

uint8_t bccam_uart_link_get_tx_credit(
  const bccam_uart_link_endpoint_t *endpoint, uint8_t service) {
  return control_index(endpoint, service) < 0 ? 0u :
    endpoint->control_binding.tx_credit;
}

uint8_t bccam_uart_link_get_rx_advertised_credit(
  const bccam_uart_link_endpoint_t *endpoint, uint8_t service) {
  return control_index(endpoint, service) < 0 ? 0u :
    endpoint->control_binding.rx_advertised_credit;
}
