#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BCCAM_UART_LINK_VERSION_1 1u
#define BCCAM_UART_BOOTSTRAP_PAYLOAD_MTU 8u
#define BCCAM_UART_LINK_V1_MIN_MTU 31u
#define BCCAM_UART_CAMERA_PROFILE_MTU 64u
#define BCCAM_UART_NORMAL_MAX_PAYLOAD 256u
#define BCCAM_UART_SERVICE_LINK_MANAGEMENT 0u
#define BCCAM_UART_SERVICE_CONTRACT_ID_MAX_LEN 24u
#define BCCAM_UART_CONTROL_CONTRACT_MAJOR 1u

typedef enum bccam_uart_result_t {
  BCCAM_UART_OK = 0,
  BCCAM_UART_ERR_BUFFER_TOO_SMALL = -1,
  BCCAM_UART_ERR_PAYLOAD_TOO_LONG = -2,
  BCCAM_UART_ERR_BAD_ARGUMENT = -3,
  BCCAM_UART_ERR_BAD_CRC = -4,
  BCCAM_UART_ERR_BAD_VERSION = -5,
  BCCAM_UART_ERR_BAD_LENGTH = -6,
  BCCAM_UART_ERR_NO_CREDIT = -7,
  BCCAM_UART_ERR_NOT_ACTIVE = -8,
  BCCAM_UART_ERR_LINK_FAULT = -9,
  BCCAM_UART_ERR_TRANSACTION_BUSY = -10,
  BCCAM_UART_ERR_UNKNOWN_SERVICE = -11,
  BCCAM_UART_ERR_MALFORMED_MANAGEMENT = -12,
} bccam_uart_result_t;

typedef enum bccam_uart_link_state_t {
  BCCAM_UART_LINK_INACTIVE = 0,
  BCCAM_UART_LINK_ACTIVE,
  BCCAM_UART_LINK_FAULT,
} bccam_uart_link_state_t;

typedef enum bccam_uart_link_op_t {
  BCCAM_UART_LINK_OP_ESTABLISH = 0x01,
  BCCAM_UART_LINK_OP_ESTABLISH_REPLY = 0x02,
  BCCAM_UART_LINK_OP_GET_SERVICE_COUNT = 0x03,
  BCCAM_UART_LINK_OP_SERVICE_COUNT = 0x04,
  BCCAM_UART_LINK_OP_GET_SERVICE_DESCRIPTOR = 0x05,
  BCCAM_UART_LINK_OP_SERVICE_DESCRIPTOR = 0x06,
  BCCAM_UART_LINK_OP_BOOT_NOTICE = 0x07,
  BCCAM_UART_LINK_OP_CREDIT_UPDATE = 0x08,
} bccam_uart_link_op_t;

typedef enum bccam_uart_establish_status_t {
  BCCAM_UART_ESTABLISH_ACCEPTED = 0x00,
  BCCAM_UART_ESTABLISH_MTU_TOO_SMALL = 0x01,
  BCCAM_UART_ESTABLISH_NO_COMMON_LINK_VERSION = 0x02,
} bccam_uart_establish_status_t;

typedef enum bccam_uart_pending_t {
  BCCAM_UART_PENDING_NONE = 0,
  BCCAM_UART_PENDING_ESTABLISH,
  BCCAM_UART_PENDING_SERVICE_COUNT,
  BCCAM_UART_PENDING_SERVICE_DESCRIPTOR,
} bccam_uart_pending_t;

typedef struct bccam_uart_service_descriptor_t {
  uint8_t handle;
  uint8_t major;
  uint8_t minor;
  uint8_t contract_id_len;
  uint8_t contract_id[BCCAM_UART_SERVICE_CONTRACT_ID_MAX_LEN];
} bccam_uart_service_descriptor_t;

typedef struct bccam_uart_service_binding_t {
  bool valid;
  bccam_uart_service_descriptor_t descriptor;
  uint8_t tx_credit;
  uint8_t rx_advertised_credit;
} bccam_uart_service_binding_t;

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
  uint16_t tx_failures;
  uint16_t credit_updates_tx;
  uint16_t credit_updates_rx;
} bccam_uart_link_counters_t;

typedef struct bccam_uart_link_endpoint_t {
  bccam_uart_link_state_t state;
  bccam_uart_link_counters_t counters;
  uint8_t active_link_version;
  uint16_t negotiated_payload;
  uint8_t next_transaction_id;
  uint8_t pending_transaction_id;
  bccam_uart_pending_t pending;
  uint8_t pending_descriptor_ordinal;
  uint8_t offered_min_version;
  uint8_t offered_max_version;
  uint16_t offered_max_mtu;
  uint16_t required_link_mtu;
  bool establishment_rejected;
  uint8_t establishment_status;
  bool boot_notice_received;
  bool service_count_known;
  uint8_t service_count;
  uint8_t descriptors_received;
  uint8_t seen_ordinals[32];
  uint8_t seen_handles[32];
  uint8_t pending_tx_credit[256];
  uint8_t observed_credit_handles[32];
  bccam_uart_service_binding_t control_binding;
  bool rx_pending;
  uint8_t rx_service;
  uint16_t rx_payload_len;
  uint8_t rx_payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
  bool tx_pending;
  uint8_t tx_version;
  uint8_t tx_service;
  uint16_t tx_payload_len;
  uint8_t tx_payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
} bccam_uart_link_endpoint_t;

void bccam_uart_link_init(bccam_uart_link_endpoint_t *endpoint);
bccam_uart_link_state_t bccam_uart_link_get_state(const bccam_uart_link_endpoint_t *endpoint);
uint16_t bccam_uart_link_get_negotiated_payload(const bccam_uart_link_endpoint_t *endpoint);
const bccam_uart_link_counters_t *bccam_uart_link_get_counters(const bccam_uart_link_endpoint_t *endpoint);

int bccam_uart_link_start_establishment(bccam_uart_link_endpoint_t *endpoint);
int bccam_uart_link_start_establishment_with(bccam_uart_link_endpoint_t *endpoint,
                                             uint8_t min_version,
                                             uint8_t max_version,
                                             uint16_t max_mtu);
int bccam_uart_link_start_service_count(bccam_uart_link_endpoint_t *endpoint);
int bccam_uart_link_start_service_descriptor(bccam_uart_link_endpoint_t *endpoint,
                                              uint8_t ordinal);
int bccam_uart_link_send_normal(bccam_uart_link_endpoint_t *endpoint,
                                uint8_t service,
                                const uint8_t *payload,
                                uint16_t payload_len);
int bccam_uart_link_send_credit_update(bccam_uart_link_endpoint_t *endpoint,
                                       uint8_t service,
                                       uint8_t available_credit);
int bccam_uart_link_release_rx_slot(bccam_uart_link_endpoint_t *endpoint,
                                    uint8_t service);
int bccam_uart_link_receive_unit(bccam_uart_link_endpoint_t *endpoint,
                                 uint8_t link_version,
                                 uint8_t service,
                                 const uint8_t *payload,
                                 uint16_t payload_len);
int bccam_uart_link_take_tx_unit(bccam_uart_link_endpoint_t *endpoint,
                                 uint8_t *link_version,
                                 uint8_t *service,
                                 uint8_t *payload,
                                 size_t payload_capacity,
                                 uint16_t *payload_len);
int bccam_uart_link_take_rx(bccam_uart_link_endpoint_t *endpoint,
                            uint8_t *service,
                            uint8_t *out,
                            size_t out_capacity,
                            uint16_t *out_len,
                            bool *unit_present);
int bccam_uart_link_enter_fault(bccam_uart_link_endpoint_t *endpoint);
int bccam_uart_link_note_tx_failure(bccam_uart_link_endpoint_t *endpoint);

bool bccam_uart_link_catalog_complete(const bccam_uart_link_endpoint_t *endpoint);
bool bccam_uart_link_control_binding(const bccam_uart_link_endpoint_t *endpoint,
                                     bccam_uart_service_descriptor_t *descriptor);
bool bccam_uart_link_boot_notice_received(const bccam_uart_link_endpoint_t *endpoint);
void bccam_uart_link_clear_boot_notice(bccam_uart_link_endpoint_t *endpoint);
uint8_t bccam_uart_link_get_tx_credit(const bccam_uart_link_endpoint_t *endpoint,
                                      uint8_t service);
uint8_t bccam_uart_link_get_rx_advertised_credit(const bccam_uart_link_endpoint_t *endpoint,
                                                  uint8_t service);
