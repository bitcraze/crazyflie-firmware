#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bccam_uart_frame.h"

#define BCCAM_UART_SERVICE_CONTROL 1u
#define BCCAM_UART_SERVICE_CONSOLE 2u
#define BCCAM_UART_LINK_MANAGEMENT_RESERVE 2u
#define BCCAM_UART_MAX_DISCOVERED_SERVICES 3u
#define BCCAM_UART_SERVICE_CONTRACT_ID_LEN 32u

typedef enum bccam_uart_link_role_t {
    BCCAM_UART_LINK_ROLE_INITIATOR,
    BCCAM_UART_LINK_ROLE_TARGET
} bccam_uart_link_role_t;

typedef enum bccam_uart_link_state_t {
    BCCAM_UART_LINK_UNINITIALIZED,
    BCCAM_UART_LINK_ACTIVE,
    BCCAM_UART_LINK_FAULT
} bccam_uart_link_state_t;

typedef enum bccam_uart_link_op_t {
    BCCAM_UART_LINK_OP_DISCOVER = 0x01,
    BCCAM_UART_LINK_OP_DISCOVER_REPLY = 0x02,
    BCCAM_UART_LINK_OP_BOOT_NOTICE = 0x03,
    BCCAM_UART_LINK_OP_REINIT_REQUEST = 0x04,
    BCCAM_UART_LINK_OP_REINIT_ACK = 0x05,
    BCCAM_UART_LINK_OP_CREDIT_UPDATE = 0x06
} bccam_uart_link_op_t;

typedef struct bccam_uart_discover_t {
    uint8_t min_frame_version;
    uint8_t max_frame_version;
    uint16_t requested_max_payload;
} bccam_uart_discover_t;

typedef struct bccam_uart_service_entry_t {
    uint8_t service_id;
    uint8_t major;
    uint8_t minor;
    uint8_t contract_id[BCCAM_UART_SERVICE_CONTRACT_ID_LEN];
} bccam_uart_service_entry_t;

typedef struct bccam_uart_discover_reply_t {
    uint8_t selected_frame_version;
    uint16_t max_payload;
    uint8_t service_count;
    bccam_uart_service_entry_t services[BCCAM_UART_MAX_DISCOVERED_SERVICES];
} bccam_uart_discover_reply_t;

typedef struct bccam_uart_credit_update_t {
    uint8_t service_id;
    uint8_t available_credit;
} bccam_uart_credit_update_t;

typedef struct bccam_uart_link_management_message_t {
    bccam_uart_link_op_t op;
    uint8_t transaction_id;
    union {
        bccam_uart_discover_t discover;
        bccam_uart_discover_reply_t discover_reply;
        bccam_uart_credit_update_t credit_update;
    } body;
} bccam_uart_link_management_message_t;

typedef struct bccam_uart_link_counters_t {
    uint16_t rx_frames;
    uint16_t tx_frames;
    uint16_t rx_crc_errors;
    uint16_t rx_length_errors;
    uint16_t rx_version_errors;
    uint16_t rx_unknown_service_errors;
    uint16_t rx_malformed_management_errors;
    uint16_t rx_resyncs;
    uint16_t link_faults;
    uint16_t credit_updates_tx;
    uint16_t credit_updates_rx;
} bccam_uart_link_counters_t;

typedef struct bccam_uart_link_endpoint_t {
    bccam_uart_link_role_t role;
    bccam_uart_link_state_t state;
    bccam_uart_frame_parser_t parser;
    bccam_uart_link_counters_t counters;
    uint16_t negotiated_payload;
    uint8_t next_transaction_id;
    uint8_t pending_transaction_id;
    bool transaction_in_flight;
    uint8_t service_count;
    bccam_uart_service_entry_t services[BCCAM_UART_MAX_DISCOVERED_SERVICES];
    uint8_t tx_credit[BCCAM_UART_MAX_DISCOVERED_SERVICES];
    uint8_t rx_advertised_credit[BCCAM_UART_MAX_DISCOVERED_SERVICES];
    bool rx_pending;
    uint8_t rx_service;
    uint16_t rx_payload_len;
    uint8_t rx_payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
    uint8_t tx_buffer[BCCAM_UART_FRAME_MAX_ENCODED_SIZE];
    size_t tx_len;
    bool tx_pending;
} bccam_uart_link_endpoint_t;

typedef bool (*bccam_uart_link_dispatch_t)(void *context,
                                           uint8_t service,
                                           const uint8_t *payload,
                                           uint16_t payload_len);

void bccam_uart_link_init(bccam_uart_link_endpoint_t *endpoint, bccam_uart_link_role_t role);

bccam_uart_link_state_t bccam_uart_link_get_state(const bccam_uart_link_endpoint_t *endpoint);

uint16_t bccam_uart_link_get_negotiated_payload(const bccam_uart_link_endpoint_t *endpoint);

const bccam_uart_link_counters_t *bccam_uart_link_get_counters(const bccam_uart_link_endpoint_t *endpoint);

bool bccam_uart_link_service_is_discovered(const bccam_uart_link_endpoint_t *endpoint,
                                        uint8_t service);

bool bccam_uart_link_find_service_by_contract_version(
    const bccam_uart_link_endpoint_t *endpoint,
    const char *contract_id,
    uint8_t major,
    uint8_t minor,
    uint8_t *service_id);

int bccam_uart_link_start_discovery(bccam_uart_link_endpoint_t *endpoint);

int bccam_uart_link_retry_discovery(bccam_uart_link_endpoint_t *endpoint);

int bccam_uart_link_send_normal(bccam_uart_link_endpoint_t *endpoint,
                             uint8_t service,
                             const uint8_t *payload,
                             uint16_t payload_len);

int bccam_uart_link_send_credit_update(bccam_uart_link_endpoint_t *endpoint,
                                    uint8_t service,
                                    uint8_t available_credit);

int bccam_uart_link_release_rx_slot(bccam_uart_link_endpoint_t *endpoint,
                                 uint8_t service);

int bccam_uart_link_send_boot_notice(bccam_uart_link_endpoint_t *endpoint);

uint8_t bccam_uart_link_get_tx_credit(const bccam_uart_link_endpoint_t *endpoint,
                                   uint8_t service);

uint8_t bccam_uart_link_get_rx_advertised_credit(const bccam_uart_link_endpoint_t *endpoint,
                                              uint8_t service);

int bccam_uart_link_take_rx(bccam_uart_link_endpoint_t *endpoint,
                         uint8_t *service,
                         uint8_t *out,
                         size_t out_capacity,
                         uint16_t *out_len);

int bccam_uart_link_start_reinit(bccam_uart_link_endpoint_t *endpoint);

int bccam_uart_link_receive_byte(bccam_uart_link_endpoint_t *endpoint, uint8_t byte);

int bccam_uart_link_receive_raw_frame(bccam_uart_link_endpoint_t *endpoint,
                                      const uint8_t *frame,
                                      uint16_t frame_len,
                                      bccam_uart_link_dispatch_t dispatch,
                                      void *dispatch_context);

int bccam_uart_link_enter_fault(bccam_uart_link_endpoint_t *endpoint);

int bccam_uart_link_store_rx_payload(bccam_uart_link_endpoint_t *endpoint,
                                     uint8_t service,
                                     const uint8_t *payload,
                                     uint16_t payload_len);

int bccam_uart_link_take_tx(bccam_uart_link_endpoint_t *endpoint,
                         uint8_t *out,
                         size_t out_capacity,
                         size_t *out_len);

int bccam_uart_link_encode_discover(uint8_t transaction_id,
                                 uint8_t *out,
                                 size_t out_capacity,
                                 size_t *out_len);

int bccam_uart_link_encode_discover_reply(uint8_t transaction_id,
                                       const bccam_uart_service_entry_t *services,
                                       uint8_t service_count,
                                       uint8_t *out,
                                       size_t out_capacity,
                                       size_t *out_len);

int bccam_uart_link_encode_credit_update(uint8_t service_id,
                                      uint8_t available_credit,
                                      uint8_t *out,
                                      size_t out_capacity,
                                      size_t *out_len);

int bccam_uart_link_encode_empty_management(bccam_uart_link_op_t op,
                                         uint8_t transaction_id,
                                         uint8_t *out,
                                         size_t out_capacity,
                                         size_t *out_len);

int bccam_uart_link_decode_management(const uint8_t *payload,
                                   uint16_t payload_len,
                                   bccam_uart_link_management_message_t *out);
