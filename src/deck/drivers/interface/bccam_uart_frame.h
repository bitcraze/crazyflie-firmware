#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BCCAM_UART_MAGIC0 0xBCu
#define BCCAM_UART_MAGIC1 0xCDu
#define BCCAM_UART_FRAME_VERSION 1u
#define BCCAM_UART_SERVICE_LINK_MANAGEMENT 0u
#define BCCAM_UART_NORMAL_MAX_PAYLOAD 256u
#define BCCAM_UART_MANAGEMENT_MAX_PAYLOAD 128u
#define BCCAM_UART_FRAME_HEADER_SIZE 6u
#define BCCAM_UART_FRAME_CRC_SIZE 2u
#define BCCAM_UART_FRAME_OVERHEAD (BCCAM_UART_FRAME_HEADER_SIZE + BCCAM_UART_FRAME_CRC_SIZE)
#define BCCAM_UART_FRAME_MAX_ENCODED_SIZE (BCCAM_UART_FRAME_OVERHEAD + BCCAM_UART_NORMAL_MAX_PAYLOAD)

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

typedef struct bccam_uart_frame_t {
    uint8_t service;
    uint16_t payload_len;
    uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
} bccam_uart_frame_t;

typedef enum bccam_uart_parse_state_t {
    BCCAM_UART_PARSE_WAIT_MAGIC0,
    BCCAM_UART_PARSE_WAIT_MAGIC1,
    BCCAM_UART_PARSE_VERSION,
    BCCAM_UART_PARSE_SERVICE,
    BCCAM_UART_PARSE_LEN0,
    BCCAM_UART_PARSE_LEN1,
    BCCAM_UART_PARSE_PAYLOAD,
    BCCAM_UART_PARSE_CRC0,
    BCCAM_UART_PARSE_CRC1
} bccam_uart_parse_state_t;

typedef struct bccam_uart_frame_parser_t {
    bccam_uart_parse_state_t state;
    uint16_t max_payload_len;
    uint8_t header_after_magic[4];
    uint8_t header_index;
    uint8_t service;
    uint16_t payload_len;
    uint16_t payload_index;
    uint8_t payload[BCCAM_UART_NORMAL_MAX_PAYLOAD];
    uint8_t crc_bytes[2];
    uint16_t rx_crc_errors;
    uint16_t rx_length_errors;
    uint16_t rx_version_errors;
    uint16_t rx_resyncs;
} bccam_uart_frame_parser_t;

int bccam_uart_frame_encode(uint8_t service,
                         const uint8_t *payload,
                         uint16_t payload_len,
                         uint8_t *out,
                         size_t out_capacity,
                         size_t *out_len);

void bccam_uart_frame_parser_init(bccam_uart_frame_parser_t *parser, uint16_t max_payload_len);

int bccam_uart_frame_parser_feed(bccam_uart_frame_parser_t *parser,
                              uint8_t byte,
                              bccam_uart_frame_t *out_frame,
                              bool *out_frame_ready);
