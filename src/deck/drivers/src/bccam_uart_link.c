#include "bccam_uart_link.h"

#include <string.h>

static const bccam_uart_service_entry_t default_services[] = {
    {
        .service_id = BCCAM_UART_SERVICE_CONTROL,
        .major = 1u,
        .minor = 0u,
        .contract_id = {
            'b', 'i', 't', 'c', 'r', 'a', 'z', 'e',
            '.', 'c', 'o', 'n', 't', 'r', 'o', 'l',
        },
    },
};

static void write_le16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t read_le16(const uint8_t *in) {
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static int begin_encode(uint8_t *out, size_t *out_len) {
    if (out_len == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    *out_len = 0;

    if (out == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    return BCCAM_UART_OK;
}

static int require_capacity(size_t required, size_t capacity) {
    if (capacity < required) {
        return BCCAM_UART_ERR_BUFFER_TOO_SMALL;
    }

    return BCCAM_UART_OK;
}

static void sync_parser_counters(bccam_uart_link_endpoint_t *endpoint) {
    endpoint->counters.rx_crc_errors = endpoint->parser.rx_crc_errors;
    endpoint->counters.rx_length_errors = endpoint->parser.rx_length_errors;
    endpoint->counters.rx_version_errors = endpoint->parser.rx_version_errors;
    endpoint->counters.rx_resyncs = endpoint->parser.rx_resyncs;
}

static void add_parser_counters(bccam_uart_link_endpoint_t *endpoint,
                                const bccam_uart_frame_parser_t *parser) {
    endpoint->parser.rx_crc_errors += parser->rx_crc_errors;
    endpoint->parser.rx_length_errors += parser->rx_length_errors;
    endpoint->parser.rx_version_errors += parser->rx_version_errors;
    endpoint->parser.rx_resyncs += parser->rx_resyncs;
    sync_parser_counters(endpoint);
}

static int raw_length_error(bccam_uart_link_endpoint_t *endpoint) {
    endpoint->parser.rx_length_errors++;
    sync_parser_counters(endpoint);
    return BCCAM_UART_ERR_BAD_LENGTH;
}

static void enter_fault(bccam_uart_link_endpoint_t *endpoint) {
    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return;
    }
    if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
        return;
    }

    endpoint->state = BCCAM_UART_LINK_FAULT;
    endpoint->counters.link_faults++;
}

static void apply_services(bccam_uart_link_endpoint_t *endpoint,
                           const bccam_uart_service_entry_t *services,
                           uint8_t service_count) {
    endpoint->service_count = service_count;
    if (service_count > 0u) {
        memcpy(endpoint->services,
               services,
               (size_t)service_count * sizeof(endpoint->services[0]));
    }
    memset(endpoint->tx_credit, 0, sizeof(endpoint->tx_credit));
    memset(endpoint->rx_advertised_credit, 0, sizeof(endpoint->rx_advertised_credit));
}

static int find_service_index(const bccam_uart_link_endpoint_t *endpoint, uint8_t service) {
    if (endpoint == NULL) {
        return -1;
    }

    const uint8_t service_count =
        (endpoint->service_count > BCCAM_UART_MAX_DISCOVERED_SERVICES) ?
        BCCAM_UART_MAX_DISCOVERED_SERVICES :
        endpoint->service_count;

    for (uint8_t i = 0; i < service_count; i++) {
        if (endpoint->services[i].service_id == service) {
            return (int)i;
        }
    }

    return -1;
}

static size_t contract_id_len(
    const uint8_t contract_id[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
    bool *valid_padding) {
    size_t len = BCCAM_UART_SERVICE_CONTRACT_ID_LEN;

    *valid_padding = true;
    for (size_t i = 0; i < BCCAM_UART_SERVICE_CONTRACT_ID_LEN; i++) {
        if (contract_id[i] == 0u) {
            len = i;
            for (size_t j = i + 1u; j < BCCAM_UART_SERVICE_CONTRACT_ID_LEN; j++) {
                if (contract_id[j] != 0u) {
                    *valid_padding = false;
                    break;
                }
            }
            break;
        }
    }

    return len;
}

static bool contract_id_is_valid(
    const uint8_t contract_id[BCCAM_UART_SERVICE_CONTRACT_ID_LEN]) {
    bool valid_padding = false;
    const size_t len = contract_id_len(contract_id, &valid_padding);
    bool segment_start = true;

    if (!valid_padding || len == 0u) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        const uint8_t ch = contract_id[i];
        const bool lower = (ch >= (uint8_t)'a' && ch <= (uint8_t)'z');
        const bool digit = (ch >= (uint8_t)'0' && ch <= (uint8_t)'9');
        const bool underscore = (ch == (uint8_t)'_');

        if (segment_start) {
            if (!lower && !digit) {
                return false;
            }
            segment_start = false;
            continue;
        }

        if (ch == (uint8_t)'.') {
            segment_start = true;
        } else if (!lower && !digit && !underscore) {
            return false;
        }
    }

    return !segment_start;
}

static bool same_contract_id(
    const uint8_t left[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
    const uint8_t right[BCCAM_UART_SERVICE_CONTRACT_ID_LEN]) {
    bool left_padding = false;
    bool right_padding = false;
    const size_t left_len = contract_id_len(left, &left_padding);
    const size_t right_len = contract_id_len(right, &right_padding);

    return left_padding &&
           right_padding &&
           left_len == right_len &&
           memcmp(left, right, left_len) == 0;
}

static bool same_contract_id_string(
    const uint8_t left[BCCAM_UART_SERVICE_CONTRACT_ID_LEN],
    const char *right) {
    bool left_padding = false;
    const size_t left_len = contract_id_len(left, &left_padding);
    size_t right_len = 0u;

    if (!left_padding || right == NULL) {
        return false;
    }

    while (right[right_len] != '\0') {
        right_len++;
        if (right_len > BCCAM_UART_SERVICE_CONTRACT_ID_LEN) {
            return false;
        }
    }

    return left_len == right_len && memcmp(left, right, left_len) == 0;
}

static bool service_table_is_valid(const bccam_uart_service_entry_t *services,
                                   uint8_t service_count) {
    if (service_count > BCCAM_UART_MAX_DISCOVERED_SERVICES ||
        (service_count > 0u && services == NULL)) {
        return false;
    }

    for (uint8_t i = 0; i < service_count; i++) {
        if (services[i].service_id == BCCAM_UART_SERVICE_LINK_MANAGEMENT ||
            !contract_id_is_valid(services[i].contract_id) ||
            (services[i].major == 0u && services[i].minor != 0u)) {
            return false;
        }

        for (uint8_t j = (uint8_t)(i + 1u); j < service_count; j++) {
            if (services[i].service_id == services[j].service_id ||
                same_contract_id(services[i].contract_id, services[j].contract_id)) {
                return false;
            }
        }
    }

    return true;
}

static int management_fault(bccam_uart_link_endpoint_t *endpoint) {
    endpoint->counters.rx_malformed_management_errors++;
    enter_fault(endpoint);
    return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
}

static int endpoint_queue_management_payload(bccam_uart_link_endpoint_t *endpoint,
                                             const uint8_t *payload,
                                             uint16_t payload_len) {
    size_t tx_len = 0;
    int result;

    if (endpoint == NULL || (payload_len > 0u && payload == NULL)) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->tx_pending) {
        return BCCAM_UART_ERR_TRANSACTION_BUSY;
    }

    result = bccam_uart_frame_encode(BCCAM_UART_SERVICE_LINK_MANAGEMENT,
                                  payload,
                                  payload_len,
                                  endpoint->tx_buffer,
                                  sizeof(endpoint->tx_buffer),
                                  &tx_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    endpoint->tx_len = tx_len;
    endpoint->tx_pending = true;
    endpoint->counters.tx_frames++;
    return BCCAM_UART_OK;
}

static int endpoint_queue_discover(bccam_uart_link_endpoint_t *endpoint,
                                   uint8_t transaction_id) {
    uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
    size_t payload_len = 0;
    int result;

    result = bccam_uart_link_encode_discover(transaction_id,
                                             payload,
                                             sizeof(payload),
                                             &payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    return endpoint_queue_management_payload(endpoint,
                                             payload,
                                             (uint16_t)payload_len);
}

static void clear_session(bccam_uart_link_endpoint_t *endpoint) {
    const uint16_t rx_crc_errors = endpoint->parser.rx_crc_errors;
    const uint16_t rx_length_errors = endpoint->parser.rx_length_errors;
    const uint16_t rx_version_errors = endpoint->parser.rx_version_errors;
    const uint16_t rx_resyncs = endpoint->parser.rx_resyncs;

    endpoint->state = BCCAM_UART_LINK_UNINITIALIZED;
    endpoint->negotiated_payload = BCCAM_UART_NORMAL_MAX_PAYLOAD;
    endpoint->pending_transaction_id = 0u;
    endpoint->transaction_in_flight = false;
    endpoint->service_count = 0u;
    memset(endpoint->services, 0, sizeof(endpoint->services));
    memset(endpoint->tx_credit, 0, sizeof(endpoint->tx_credit));
    memset(endpoint->rx_advertised_credit, 0, sizeof(endpoint->rx_advertised_credit));
    endpoint->rx_pending = false;
    endpoint->rx_service = 0u;
    endpoint->rx_payload_len = 0u;
    endpoint->tx_pending = false;
    endpoint->tx_len = 0u;

    bccam_uart_frame_parser_init(&endpoint->parser, BCCAM_UART_NORMAL_MAX_PAYLOAD);
    endpoint->parser.rx_crc_errors = rx_crc_errors;
    endpoint->parser.rx_length_errors = rx_length_errors;
    endpoint->parser.rx_version_errors = rx_version_errors;
    endpoint->parser.rx_resyncs = rx_resyncs;
}

static void clear_session_preserving_tx(bccam_uart_link_endpoint_t *endpoint) {
    const bool tx_pending = endpoint->tx_pending;
    const size_t tx_len = endpoint->tx_len;
    uint8_t tx_buffer[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];

    if (tx_pending && tx_len > 0u) {
        memcpy(tx_buffer, endpoint->tx_buffer, tx_len);
    }

    clear_session(endpoint);

    if (tx_pending && tx_len > 0u) {
        memcpy(endpoint->tx_buffer, tx_buffer, tx_len);
    }
    endpoint->tx_pending = tx_pending;
    endpoint->tx_len = tx_len;
}

void bccam_uart_link_init(bccam_uart_link_endpoint_t *endpoint, bccam_uart_link_role_t role) {
    if (endpoint == NULL) {
        return;
    }

    memset(endpoint, 0, sizeof(*endpoint));
    endpoint->role = role;
    endpoint->state = BCCAM_UART_LINK_UNINITIALIZED;
    endpoint->negotiated_payload = BCCAM_UART_NORMAL_MAX_PAYLOAD;
    endpoint->next_transaction_id = 1u;
    bccam_uart_frame_parser_init(&endpoint->parser, BCCAM_UART_NORMAL_MAX_PAYLOAD);
}

bccam_uart_link_state_t bccam_uart_link_get_state(const bccam_uart_link_endpoint_t *endpoint) {
    if (endpoint == NULL) {
        return BCCAM_UART_LINK_FAULT;
    }

    return endpoint->state;
}

uint16_t bccam_uart_link_get_negotiated_payload(const bccam_uart_link_endpoint_t *endpoint) {
    if (endpoint == NULL) {
        return 0u;
    }

    return endpoint->negotiated_payload;
}

const bccam_uart_link_counters_t *bccam_uart_link_get_counters(
    const bccam_uart_link_endpoint_t *endpoint) {
    if (endpoint == NULL) {
        return NULL;
    }

    return &endpoint->counters;
}

bool bccam_uart_link_service_is_discovered(const bccam_uart_link_endpoint_t *endpoint,
                                        uint8_t service) {
    return find_service_index(endpoint, service) >= 0;
}

bool bccam_uart_link_find_service_by_contract_version(
    const bccam_uart_link_endpoint_t *endpoint,
    const char *contract_id,
    uint8_t major,
    uint8_t minor,
    uint8_t *service_id) {
    if (service_id != NULL) {
        *service_id = 0u;
    }

    if (endpoint == NULL || contract_id == NULL || service_id == NULL) {
        return false;
    }

    const uint8_t service_count =
        (endpoint->service_count > BCCAM_UART_MAX_DISCOVERED_SERVICES) ?
        BCCAM_UART_MAX_DISCOVERED_SERVICES :
        endpoint->service_count;

    for (uint8_t i = 0; i < service_count; i++) {
        const bccam_uart_service_entry_t *service = &endpoint->services[i];
        if (service->major == major &&
            service->minor == minor &&
            same_contract_id_string(service->contract_id, contract_id)) {
            *service_id = service->service_id;
            return true;
        }
    }

    return false;
}

uint8_t bccam_uart_link_get_tx_credit(const bccam_uart_link_endpoint_t *endpoint,
                                   uint8_t service) {
    const int service_index = find_service_index(endpoint, service);
    if (service_index < 0) {
        return 0u;
    }

    return endpoint->tx_credit[service_index];
}

uint8_t bccam_uart_link_get_rx_advertised_credit(
    const bccam_uart_link_endpoint_t *endpoint,
    uint8_t service) {
    const int service_index = find_service_index(endpoint, service);
    if (service_index < 0) {
        return 0u;
    }

    return endpoint->rx_advertised_credit[service_index];
}

int bccam_uart_link_start_discovery(bccam_uart_link_endpoint_t *endpoint) {
    uint8_t transaction_id;
    int result;

    if (endpoint == NULL || endpoint->role != BCCAM_UART_LINK_ROLE_INITIATOR) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (endpoint->state != BCCAM_UART_LINK_UNINITIALIZED) {
        return BCCAM_UART_ERR_TRANSACTION_BUSY;
    }

    if (endpoint->transaction_in_flight) {
        return BCCAM_UART_ERR_TRANSACTION_BUSY;
    }

    transaction_id = endpoint->next_transaction_id;
    result = endpoint_queue_discover(endpoint, transaction_id);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    endpoint->pending_transaction_id = transaction_id;
    endpoint->transaction_in_flight = true;
    endpoint->next_transaction_id++;
    if (endpoint->next_transaction_id == 0u) {
        endpoint->next_transaction_id = 1u;
    }
    return BCCAM_UART_OK;
}

int bccam_uart_link_retry_discovery(bccam_uart_link_endpoint_t *endpoint) {
    if (endpoint == NULL || endpoint->role != BCCAM_UART_LINK_ROLE_INITIATOR) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (endpoint->state != BCCAM_UART_LINK_UNINITIALIZED) {
        return BCCAM_UART_ERR_TRANSACTION_BUSY;
    }

    if (!endpoint->transaction_in_flight || endpoint->pending_transaction_id == 0u) {
        return bccam_uart_link_start_discovery(endpoint);
    }

    return endpoint_queue_discover(endpoint, endpoint->pending_transaction_id);
}

int bccam_uart_link_send_normal(bccam_uart_link_endpoint_t *endpoint,
                             uint8_t service,
                             const uint8_t *payload,
                             uint16_t payload_len) {
    size_t tx_len = 0u;
    int result;

    if (endpoint == NULL || (payload_len > 0u && payload == NULL)) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
        return BCCAM_UART_ERR_NOT_ACTIVE;
    }

    if (service == BCCAM_UART_SERVICE_LINK_MANAGEMENT ||
        !bccam_uart_link_service_is_discovered(endpoint, service)) {
        return BCCAM_UART_ERR_UNKNOWN_SERVICE;
    }

    const int service_index = find_service_index(endpoint, service);
    if (service_index < 0) {
        return BCCAM_UART_ERR_UNKNOWN_SERVICE;
    }

    if (payload_len > endpoint->negotiated_payload) {
        return BCCAM_UART_ERR_PAYLOAD_TOO_LONG;
    }

    if (endpoint->tx_credit[service_index] == 0u) {
        return BCCAM_UART_ERR_NO_CREDIT;
    }

    if (endpoint->tx_pending) {
        return BCCAM_UART_ERR_TRANSACTION_BUSY;
    }

    result = bccam_uart_frame_encode(service,
                                  payload,
                                  payload_len,
                                  endpoint->tx_buffer,
                                  sizeof(endpoint->tx_buffer),
                                  &tx_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    endpoint->tx_len = tx_len;
    endpoint->tx_pending = true;
    endpoint->tx_credit[service_index]--;
    endpoint->counters.tx_frames++;
    return BCCAM_UART_OK;
}

int bccam_uart_link_send_credit_update(bccam_uart_link_endpoint_t *endpoint,
                                    uint8_t service,
                                    uint8_t available_credit) {
    uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
    size_t payload_len = 0u;
    int result;

    if (endpoint == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
        return BCCAM_UART_ERR_NOT_ACTIVE;
    }

    const int service_index = find_service_index(endpoint, service);
    if (service_index < 0 || service == BCCAM_UART_SERVICE_LINK_MANAGEMENT ||
        available_credit > 127u ||
        available_credit < endpoint->rx_advertised_credit[service_index]) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    result = bccam_uart_link_encode_credit_update(service,
                                               available_credit,
                                               payload,
                                               sizeof(payload),
                                               &payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    result = endpoint_queue_management_payload(endpoint,
                                               payload,
                                               (uint16_t)payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    endpoint->rx_advertised_credit[service_index] = available_credit;
    endpoint->counters.credit_updates_tx++;
    return BCCAM_UART_OK;
}

int bccam_uart_link_release_rx_slot(bccam_uart_link_endpoint_t *endpoint,
                                 uint8_t service) {
    if (endpoint == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
        return BCCAM_UART_ERR_NOT_ACTIVE;
    }

    const int service_index = find_service_index(endpoint, service);
    if (service_index < 0) {
        return BCCAM_UART_ERR_UNKNOWN_SERVICE;
    }

    if (endpoint->rx_advertised_credit[service_index] >= 127u) {
        return BCCAM_UART_OK;
    }

    return bccam_uart_link_send_credit_update(
        endpoint,
        service,
        (uint8_t)(endpoint->rx_advertised_credit[service_index] + 1u));
}

int bccam_uart_link_send_boot_notice(bccam_uart_link_endpoint_t *endpoint) {
    uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
    size_t payload_len = 0u;
    int result;

    if (endpoint == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    result = bccam_uart_link_encode_empty_management(BCCAM_UART_LINK_OP_BOOT_NOTICE,
                                                  0u,
                                                  payload,
                                                  sizeof(payload),
                                                  &payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    result = endpoint_queue_management_payload(endpoint,
                                               payload,
                                               (uint16_t)payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    clear_session_preserving_tx(endpoint);
    return BCCAM_UART_OK;
}

int bccam_uart_link_start_reinit(bccam_uart_link_endpoint_t *endpoint) {
    uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
    size_t payload_len = 0u;
    uint8_t transaction_id;
    int result;

    if (endpoint == NULL || endpoint->role != BCCAM_UART_LINK_ROLE_INITIATOR) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
        return BCCAM_UART_ERR_NOT_ACTIVE;
    }

    if (endpoint->transaction_in_flight) {
        return BCCAM_UART_ERR_TRANSACTION_BUSY;
    }

    transaction_id = endpoint->next_transaction_id;
    result = bccam_uart_link_encode_empty_management(BCCAM_UART_LINK_OP_REINIT_REQUEST,
                                                  transaction_id,
                                                  payload,
                                                  sizeof(payload),
                                                  &payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    result = endpoint_queue_management_payload(endpoint,
                                               payload,
                                               (uint16_t)payload_len);
    if (result != BCCAM_UART_OK) {
        return result;
    }

    endpoint->pending_transaction_id = transaction_id;
    endpoint->transaction_in_flight = true;
    endpoint->next_transaction_id++;
    if (endpoint->next_transaction_id == 0u) {
        endpoint->next_transaction_id = 1u;
    }
    return BCCAM_UART_OK;
}

int bccam_uart_link_take_tx(bccam_uart_link_endpoint_t *endpoint,
                         uint8_t *out,
                         size_t out_capacity,
                         size_t *out_len) {
    if (out_len == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    *out_len = 0;

    if (endpoint == NULL || out == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (!endpoint->tx_pending) {
        return BCCAM_UART_OK;
    }

    if (out_capacity < endpoint->tx_len) {
        return BCCAM_UART_ERR_BUFFER_TOO_SMALL;
    }

    memcpy(out, endpoint->tx_buffer, endpoint->tx_len);
    *out_len = endpoint->tx_len;
    endpoint->tx_pending = false;
    endpoint->tx_len = 0u;
    return BCCAM_UART_OK;
}

int bccam_uart_link_take_rx(bccam_uart_link_endpoint_t *endpoint,
                         uint8_t *service,
                         uint8_t *out,
                         size_t out_capacity,
                         uint16_t *out_len) {
    if (out_len == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    *out_len = 0u;

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
    memcpy(out, endpoint->rx_payload, endpoint->rx_payload_len);
    *out_len = endpoint->rx_payload_len;
    endpoint->rx_pending = false;
    endpoint->rx_service = 0u;
    endpoint->rx_payload_len = 0u;
    return BCCAM_UART_OK;
}

static int handle_management(bccam_uart_link_endpoint_t *endpoint,
                             const bccam_uart_frame_t *frame) {
    bccam_uart_link_management_message_t message;
    uint8_t payload[BCCAM_UART_MANAGEMENT_MAX_PAYLOAD];
    size_t payload_len = 0;
    uint16_t negotiated_payload;
    int result;

    if (endpoint->state == BCCAM_UART_LINK_FAULT) {
        return BCCAM_UART_ERR_LINK_FAULT;
    }

    if (frame->payload_len > BCCAM_UART_MANAGEMENT_MAX_PAYLOAD) {
        return management_fault(endpoint);
    }

    result = bccam_uart_link_decode_management(frame->payload,
                                            frame->payload_len,
                                            &message);
    if (result != BCCAM_UART_OK) {
        (void)management_fault(endpoint);
        return result;
    }

    switch (message.op) {
    case BCCAM_UART_LINK_OP_DISCOVER:
        if (endpoint->role != BCCAM_UART_LINK_ROLE_TARGET ||
            (endpoint->state != BCCAM_UART_LINK_UNINITIALIZED &&
             endpoint->state != BCCAM_UART_LINK_ACTIVE)) {
            return management_fault(endpoint);
        }

        if (message.body.discover.min_frame_version > BCCAM_UART_FRAME_VERSION ||
            message.body.discover.max_frame_version < BCCAM_UART_FRAME_VERSION ||
            message.body.discover.requested_max_payload == 0u) {
            return management_fault(endpoint);
        }

        negotiated_payload = message.body.discover.requested_max_payload;
        if (negotiated_payload > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
            negotiated_payload = BCCAM_UART_NORMAL_MAX_PAYLOAD;
        }

        result = bccam_uart_link_encode_discover_reply(
            message.transaction_id,
            default_services,
            (uint8_t)(sizeof(default_services) / sizeof(default_services[0])),
            payload,
            sizeof(payload),
            &payload_len);
        if (result != BCCAM_UART_OK) {
            return result;
        }
        write_le16(&payload[3], negotiated_payload);

        if (endpoint->state == BCCAM_UART_LINK_ACTIVE) {
            clear_session(endpoint);
        }

        result = endpoint_queue_management_payload(endpoint,
                                                   payload,
                                                   (uint16_t)payload_len);
        if (result != BCCAM_UART_OK) {
            return result;
        }

        endpoint->state = BCCAM_UART_LINK_ACTIVE;
        endpoint->negotiated_payload = negotiated_payload;
        endpoint->parser.max_payload_len = negotiated_payload;
        apply_services(endpoint,
                       default_services,
                       (uint8_t)(sizeof(default_services) / sizeof(default_services[0])));
        return BCCAM_UART_OK;

    case BCCAM_UART_LINK_OP_DISCOVER_REPLY:
        if (endpoint->role != BCCAM_UART_LINK_ROLE_INITIATOR ||
            endpoint->state != BCCAM_UART_LINK_UNINITIALIZED ||
            !endpoint->transaction_in_flight ||
            message.transaction_id != endpoint->pending_transaction_id) {
            return management_fault(endpoint);
        }

        if (message.body.discover_reply.selected_frame_version != BCCAM_UART_FRAME_VERSION ||
            message.body.discover_reply.max_payload == 0u ||
            message.body.discover_reply.max_payload > BCCAM_UART_NORMAL_MAX_PAYLOAD ||
            !service_table_is_valid(message.body.discover_reply.services,
                                    message.body.discover_reply.service_count)) {
            return management_fault(endpoint);
        }

        endpoint->transaction_in_flight = false;
        endpoint->pending_transaction_id = 0u;
        endpoint->tx_pending = false;
        endpoint->tx_len = 0u;
        endpoint->state = BCCAM_UART_LINK_ACTIVE;
        endpoint->negotiated_payload = message.body.discover_reply.max_payload;
        endpoint->parser.max_payload_len = message.body.discover_reply.max_payload;
        apply_services(endpoint,
                       message.body.discover_reply.services,
                       message.body.discover_reply.service_count);
        return BCCAM_UART_OK;

    case BCCAM_UART_LINK_OP_CREDIT_UPDATE: {
        if (endpoint->role == BCCAM_UART_LINK_ROLE_INITIATOR &&
            endpoint->state == BCCAM_UART_LINK_UNINITIALIZED &&
            endpoint->transaction_in_flight) {
            return BCCAM_UART_OK;
        }

        const int service_index =
            find_service_index(endpoint, message.body.credit_update.service_id);

        if (endpoint->state != BCCAM_UART_LINK_ACTIVE ||
            service_index < 0 ||
            message.body.credit_update.service_id == BCCAM_UART_SERVICE_LINK_MANAGEMENT ||
            message.body.credit_update.available_credit > 127u) {
            return management_fault(endpoint);
        }

        endpoint->tx_credit[service_index] = message.body.credit_update.available_credit;
        endpoint->counters.credit_updates_rx++;
        return BCCAM_UART_OK;
    }

    case BCCAM_UART_LINK_OP_BOOT_NOTICE:
        clear_session(endpoint);
        return BCCAM_UART_OK;

    case BCCAM_UART_LINK_OP_REINIT_REQUEST:
        if (endpoint->role != BCCAM_UART_LINK_ROLE_TARGET ||
            endpoint->state != BCCAM_UART_LINK_ACTIVE) {
            return management_fault(endpoint);
        }

        result = bccam_uart_link_encode_empty_management(BCCAM_UART_LINK_OP_REINIT_ACK,
                                                      message.transaction_id,
                                                      payload,
                                                      sizeof(payload),
                                                      &payload_len);
        if (result != BCCAM_UART_OK) {
            return result;
        }

        result = endpoint_queue_management_payload(endpoint,
                                                   payload,
                                                   (uint16_t)payload_len);
        if (result != BCCAM_UART_OK) {
            enter_fault(endpoint);
            return result;
        }

        clear_session_preserving_tx(endpoint);
        return BCCAM_UART_OK;

    case BCCAM_UART_LINK_OP_REINIT_ACK:
        if (endpoint->role != BCCAM_UART_LINK_ROLE_INITIATOR ||
            endpoint->state != BCCAM_UART_LINK_ACTIVE ||
            !endpoint->transaction_in_flight ||
            message.transaction_id != endpoint->pending_transaction_id) {
            return management_fault(endpoint);
        }

        clear_session(endpoint);
        return BCCAM_UART_OK;

    default:
        return management_fault(endpoint);
    }
}

static int decode_raw_frame(bccam_uart_link_endpoint_t *endpoint,
                            const uint8_t *raw,
                            uint16_t raw_len,
                            bccam_uart_frame_t *frame) {
    bccam_uart_frame_parser_t parser;
    bool ready = false;
    int result = BCCAM_UART_OK;

    if (endpoint == NULL || raw == NULL || frame == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (raw_len < BCCAM_UART_FRAME_OVERHEAD) {
        return raw_length_error(endpoint);
    }

    if (raw[0] != BCCAM_UART_MAGIC0 || raw[1] != BCCAM_UART_MAGIC1) {
        return raw_length_error(endpoint);
    }

    bccam_uart_frame_parser_init(&parser, endpoint->parser.max_payload_len);
    for (uint16_t i = 0; i < raw_len; i++) {
        result = bccam_uart_frame_parser_feed(&parser, raw[i], frame, &ready);
        if (result != BCCAM_UART_OK) {
            add_parser_counters(endpoint, &parser);
            return result;
        }
        if (ready && i != (uint16_t)(raw_len - 1u)) {
            add_parser_counters(endpoint, &parser);
            return raw_length_error(endpoint);
        }
    }

    add_parser_counters(endpoint, &parser);

    if (!ready) {
        return raw_length_error(endpoint);
    }

    return BCCAM_UART_OK;
}

int bccam_uart_link_enter_fault(bccam_uart_link_endpoint_t *endpoint) {
    if (endpoint == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }
    enter_fault(endpoint);
    return BCCAM_UART_ERR_LINK_FAULT;
}

int bccam_uart_link_store_rx_payload(bccam_uart_link_endpoint_t *endpoint,
                                     uint8_t service,
                                     const uint8_t *payload,
                                     uint16_t payload_len) {
    if (endpoint == NULL || (payload_len > 0u && payload == NULL)) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (payload_len > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
        return BCCAM_UART_ERR_BAD_LENGTH;
    }

    if (endpoint->rx_pending) {
        enter_fault(endpoint);
        return BCCAM_UART_ERR_NO_CREDIT;
    }

    endpoint->rx_service = service;
    endpoint->rx_payload_len = payload_len;
    if (payload_len > 0u) {
        memcpy(endpoint->rx_payload, payload, payload_len);
    }
    endpoint->rx_pending = true;
    return BCCAM_UART_OK;
}

static int handle_normal_frame(bccam_uart_link_endpoint_t *endpoint,
                               const bccam_uart_frame_t *frame,
                               bccam_uart_link_dispatch_t dispatch,
                               void *dispatch_context) {
    if (endpoint->state != BCCAM_UART_LINK_ACTIVE) {
        enter_fault(endpoint);
        return BCCAM_UART_ERR_NOT_ACTIVE;
    }

    if (!bccam_uart_link_service_is_discovered(endpoint, frame->service)) {
        endpoint->counters.rx_unknown_service_errors++;
        enter_fault(endpoint);
        return BCCAM_UART_ERR_UNKNOWN_SERVICE;
    }

    const int service_index = find_service_index(endpoint, frame->service);
    if (service_index < 0 || endpoint->rx_advertised_credit[service_index] == 0u) {
        enter_fault(endpoint);
        return BCCAM_UART_ERR_NO_CREDIT;
    }

    if (dispatch != NULL) {
        if (!dispatch(dispatch_context,
                      frame->service,
                      frame->payload,
                      frame->payload_len)) {
            enter_fault(endpoint);
            return BCCAM_UART_ERR_NO_CREDIT;
        }
        endpoint->rx_advertised_credit[service_index]--;
        return BCCAM_UART_OK;
    }

    if (endpoint->rx_pending) {
        enter_fault(endpoint);
        return BCCAM_UART_ERR_NO_CREDIT;
    }

    const int result = bccam_uart_link_store_rx_payload(endpoint,
                                                       frame->service,
                                                       frame->payload,
                                                       frame->payload_len);
    if (result == BCCAM_UART_OK) {
        endpoint->rx_advertised_credit[service_index]--;
    }
    return result;
}

static int handle_received_frame(bccam_uart_link_endpoint_t *endpoint,
                                 const bccam_uart_frame_t *frame,
                                 bccam_uart_link_dispatch_t dispatch,
                                 void *dispatch_context) {
    endpoint->counters.rx_frames++;

    if (frame->service == BCCAM_UART_SERVICE_LINK_MANAGEMENT) {
        return handle_management(endpoint, frame);
    }

    return handle_normal_frame(endpoint, frame, dispatch, dispatch_context);
}

int bccam_uart_link_receive_byte(bccam_uart_link_endpoint_t *endpoint, uint8_t byte) {
    bccam_uart_frame_t frame;
    bool frame_ready = false;
    int result;

    if (endpoint == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    result = bccam_uart_frame_parser_feed(&endpoint->parser, byte, &frame, &frame_ready);
    sync_parser_counters(endpoint);

    if (result == BCCAM_UART_ERR_BAD_CRC ||
        result == BCCAM_UART_ERR_BAD_VERSION ||
        result == BCCAM_UART_ERR_BAD_LENGTH) {
        enter_fault(endpoint);
        return result;
    }

    if (result != BCCAM_UART_OK || !frame_ready) {
        return result;
    }

    return handle_received_frame(endpoint, &frame, NULL, NULL);
}

int bccam_uart_link_receive_raw_frame(bccam_uart_link_endpoint_t *endpoint,
                                      const uint8_t *raw,
                                      uint16_t raw_len,
                                      bccam_uart_link_dispatch_t dispatch,
                                      void *dispatch_context) {
    bccam_uart_frame_t frame;
    int result;

    if (endpoint == NULL || raw == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    result = decode_raw_frame(endpoint, raw, raw_len, &frame);
    if (result == BCCAM_UART_ERR_BAD_CRC ||
        result == BCCAM_UART_ERR_BAD_VERSION ||
        result == BCCAM_UART_ERR_BAD_LENGTH) {
        enter_fault(endpoint);
        return result;
    }
    if (result != BCCAM_UART_OK) {
        return result;
    }

    return handle_received_frame(endpoint, &frame, dispatch, dispatch_context);
}

int bccam_uart_link_encode_discover(uint8_t transaction_id,
                                 uint8_t *out,
                                 size_t out_capacity,
                                 size_t *out_len) {
    const size_t required = 6u;
    const int args_result = begin_encode(out, out_len);
    if (args_result != BCCAM_UART_OK) {
        return args_result;
    }

    const int capacity_result = require_capacity(required, out_capacity);
    if (capacity_result != BCCAM_UART_OK) {
        return capacity_result;
    }

    out[0] = (uint8_t)BCCAM_UART_LINK_OP_DISCOVER;
    out[1] = transaction_id;
    out[2] = BCCAM_UART_FRAME_VERSION;
    out[3] = BCCAM_UART_FRAME_VERSION;
    write_le16(&out[4], BCCAM_UART_NORMAL_MAX_PAYLOAD);

    *out_len = required;
    return BCCAM_UART_OK;
}

int bccam_uart_link_encode_discover_reply(uint8_t transaction_id,
                                       const bccam_uart_service_entry_t *services,
                                       uint8_t service_count,
                                       uint8_t *out,
                                       size_t out_capacity,
                                       size_t *out_len) {
    const size_t entry_size = 3u + BCCAM_UART_SERVICE_CONTRACT_ID_LEN;
    const size_t required = 6u + ((size_t)service_count * entry_size);
    const int args_result = begin_encode(out, out_len);
    if (args_result != BCCAM_UART_OK) {
        return args_result;
    }

    if (!service_table_is_valid(services, service_count)) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    const int capacity_result = require_capacity(required, out_capacity);
    if (capacity_result != BCCAM_UART_OK) {
        return capacity_result;
    }

    out[0] = (uint8_t)BCCAM_UART_LINK_OP_DISCOVER_REPLY;
    out[1] = transaction_id;
    out[2] = BCCAM_UART_FRAME_VERSION;
    write_le16(&out[3], BCCAM_UART_NORMAL_MAX_PAYLOAD);
    out[5] = service_count;

    for (uint8_t i = 0; i < service_count; i++) {
        const size_t offset = 6u + ((size_t)i * entry_size);
        out[offset] = services[i].service_id;
        out[offset + 1u] = services[i].major;
        out[offset + 2u] = services[i].minor;
        memcpy(&out[offset + 3u],
               services[i].contract_id,
               BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
    }

    *out_len = required;
    return BCCAM_UART_OK;
}

int bccam_uart_link_encode_credit_update(uint8_t service_id,
                                      uint8_t available_credit,
                                      uint8_t *out,
                                      size_t out_capacity,
                                      size_t *out_len) {
    const size_t required = 4u;
    const int args_result = begin_encode(out, out_len);
    if (args_result != BCCAM_UART_OK) {
        return args_result;
    }

    if (service_id == BCCAM_UART_SERVICE_LINK_MANAGEMENT || available_credit > 127u) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    const int capacity_result = require_capacity(required, out_capacity);
    if (capacity_result != BCCAM_UART_OK) {
        return capacity_result;
    }

    out[0] = (uint8_t)BCCAM_UART_LINK_OP_CREDIT_UPDATE;
    out[1] = 0u;
    out[2] = service_id;
    out[3] = available_credit;

    *out_len = required;
    return BCCAM_UART_OK;
}

int bccam_uart_link_encode_empty_management(bccam_uart_link_op_t op,
                                         uint8_t transaction_id,
                                         uint8_t *out,
                                         size_t out_capacity,
                                         size_t *out_len) {
    const size_t required = 2u;
    const int args_result = begin_encode(out, out_len);
    if (args_result != BCCAM_UART_OK) {
        return args_result;
    }

    if (op != BCCAM_UART_LINK_OP_BOOT_NOTICE &&
        op != BCCAM_UART_LINK_OP_REINIT_REQUEST &&
        op != BCCAM_UART_LINK_OP_REINIT_ACK) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    const int capacity_result = require_capacity(required, out_capacity);
    if (capacity_result != BCCAM_UART_OK) {
        return capacity_result;
    }

    out[0] = (uint8_t)op;
    out[1] = transaction_id;

    *out_len = required;
    return BCCAM_UART_OK;
}

int bccam_uart_link_decode_management(const uint8_t *payload,
                                   uint16_t payload_len,
                                   bccam_uart_link_management_message_t *out) {
    bccam_uart_link_management_message_t message;

    if (payload == NULL || out == NULL || payload_len < 2u) {
        return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
    }

    memset(&message, 0, sizeof(message));
    message.op = (bccam_uart_link_op_t)payload[0];
    message.transaction_id = payload[1];

    switch (message.op) {
    case BCCAM_UART_LINK_OP_DISCOVER:
        if (payload_len != 6u) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }
        message.body.discover.min_frame_version = payload[2];
        message.body.discover.max_frame_version = payload[3];
        message.body.discover.requested_max_payload = read_le16(&payload[4]);
        *out = message;
        return BCCAM_UART_OK;

    case BCCAM_UART_LINK_OP_DISCOVER_REPLY: {
        const size_t entry_size = 3u + BCCAM_UART_SERVICE_CONTRACT_ID_LEN;

        if (payload_len < 6u) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }

        const uint8_t service_count = payload[5];
        if (service_count > BCCAM_UART_MAX_DISCOVERED_SERVICES ||
            payload_len != (uint16_t)(6u + ((size_t)service_count * entry_size))) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }

        message.body.discover_reply.selected_frame_version = payload[2];
        message.body.discover_reply.max_payload = read_le16(&payload[3]);
        message.body.discover_reply.service_count = service_count;

        for (uint8_t i = 0; i < service_count; i++) {
            const size_t offset = 6u + ((size_t)i * entry_size);
            message.body.discover_reply.services[i].service_id = payload[offset];
            message.body.discover_reply.services[i].major = payload[offset + 1u];
            message.body.discover_reply.services[i].minor = payload[offset + 2u];
            memcpy(message.body.discover_reply.services[i].contract_id,
                   &payload[offset + 3u],
                   BCCAM_UART_SERVICE_CONTRACT_ID_LEN);
        }

        if (!service_table_is_valid(message.body.discover_reply.services, service_count)) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }
        *out = message;
        return BCCAM_UART_OK;
    }

    case BCCAM_UART_LINK_OP_BOOT_NOTICE:
    case BCCAM_UART_LINK_OP_REINIT_REQUEST:
    case BCCAM_UART_LINK_OP_REINIT_ACK:
        if (payload_len != 2u) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }
        *out = message;
        return BCCAM_UART_OK;

    case BCCAM_UART_LINK_OP_CREDIT_UPDATE:
        if (payload_len != 4u) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }
        message.body.credit_update.service_id = payload[2];
        message.body.credit_update.available_credit = payload[3];
        if (message.transaction_id != 0u ||
            message.body.credit_update.service_id == BCCAM_UART_SERVICE_LINK_MANAGEMENT ||
            message.body.credit_update.available_credit > 127u) {
            return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
        }
        *out = message;
        return BCCAM_UART_OK;

    default:
        return BCCAM_UART_ERR_MALFORMED_MANAGEMENT;
    }
}
