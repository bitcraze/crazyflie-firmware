#include "bccam_uart_frame.h"

#include <string.h>

#include "bccam_uart_crc.h"

static void write_le16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)(value & 0xFFu);
    out[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t read_le16(const uint8_t *in) {
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static void parser_reset(bccam_uart_frame_parser_t *parser) {
    parser->state = BCCAM_UART_PARSE_WAIT_MAGIC0;
    parser->service = 0;
    parser->payload_len = 0;
    parser->payload_index = 0;
    parser->crc_bytes[0] = 0;
    parser->crc_bytes[1] = 0;
}

static void parser_reset_after_error(bccam_uart_frame_parser_t *parser,
                                     const uint8_t *suffix,
                                     size_t suffix_len) {
    parser_reset(parser);
    if (suffix_len >= 2u &&
        suffix[suffix_len - 2u] == BCCAM_UART_MAGIC0 &&
        suffix[suffix_len - 1u] == BCCAM_UART_MAGIC1) {
        parser->state = BCCAM_UART_PARSE_VERSION;
    } else if (suffix_len >= 1u && suffix[suffix_len - 1u] == BCCAM_UART_MAGIC0) {
        parser->state = BCCAM_UART_PARSE_WAIT_MAGIC1;
    }
}

static int parser_finish_frame(bccam_uart_frame_parser_t *parser,
                               bccam_uart_frame_t *out_frame,
                               bool *out_frame_ready) {
    const uint16_t expected_crc = read_le16(parser->crc_bytes);
    uint16_t actual_crc = bccam_uart_crc16_begin();
    actual_crc = bccam_uart_crc16_update(actual_crc,
                                      parser->header_after_magic,
                                      sizeof(parser->header_after_magic));
    actual_crc = bccam_uart_crc16_update(actual_crc,
                                      parser->payload,
                                      parser->payload_len);

    if (actual_crc != expected_crc) {
        const uint8_t crc_suffix[] = {
            parser->crc_bytes[0],
            parser->crc_bytes[1]
        };
        parser->rx_crc_errors++;
        parser_reset_after_error(parser, crc_suffix, sizeof(crc_suffix));
        return BCCAM_UART_ERR_BAD_CRC;
    }

    out_frame->link_version = parser->header_after_magic[0];
    out_frame->service = parser->service;
    out_frame->payload_len = parser->payload_len;
    if (parser->payload_len > 0u) {
        memcpy(out_frame->payload, parser->payload, parser->payload_len);
    }

    *out_frame_ready = true;
    parser_reset(parser);
    return BCCAM_UART_OK;
}

int bccam_uart_frame_encode_version(uint8_t link_version,
                                    uint8_t service,
                                    const uint8_t *payload,
                                    uint16_t payload_len,
                                    uint8_t *out,
                                    size_t out_capacity,
                                    size_t *out_len) {
    if (out_len == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    *out_len = 0;

    if (out == NULL || (payload_len > 0u && payload == NULL)) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    if (payload_len > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
        return BCCAM_UART_ERR_PAYLOAD_TOO_LONG;
    }

    const size_t required = BCCAM_UART_FRAME_OVERHEAD + (size_t)payload_len;
    if (out_capacity < required) {
        return BCCAM_UART_ERR_BUFFER_TOO_SMALL;
    }

    out[0] = BCCAM_UART_MAGIC0;
    out[1] = BCCAM_UART_MAGIC1;
    out[2] = link_version;
    out[3] = service;
    write_le16(&out[4], payload_len);

    if (payload_len > 0u) {
        memcpy(&out[BCCAM_UART_FRAME_HEADER_SIZE], payload, payload_len);
    }

    const uint16_t crc = bccam_uart_crc16(&out[2], 4u + (size_t)payload_len);
    write_le16(&out[BCCAM_UART_FRAME_HEADER_SIZE + payload_len], crc);

    *out_len = required;
    return BCCAM_UART_OK;
}

void bccam_uart_frame_parser_init(bccam_uart_frame_parser_t *parser, uint16_t max_payload_len) {
    if (parser == NULL) {
        return;
    }

    parser->expected_link_version = BCCAM_UART_FRAME_VERSION;
    parser->max_payload_len = max_payload_len;
    parser->rx_crc_errors = 0;
    parser->rx_length_errors = 0;
    parser->rx_version_errors = 0;
    parser->rx_resyncs = 0;
    parser_reset(parser);
}

void bccam_uart_frame_parser_configure(bccam_uart_frame_parser_t *parser,
                                       uint8_t link_version,
                                       uint16_t max_payload_len) {
    if (parser == NULL) {
        return;
    }
    parser->expected_link_version = link_version;
    parser->max_payload_len = max_payload_len;
    parser_reset(parser);
}

int bccam_uart_frame_parser_feed(bccam_uart_frame_parser_t *parser,
                              uint8_t byte,
                              bccam_uart_frame_t *out_frame,
                              bool *out_frame_ready) {
    if (parser == NULL || out_frame == NULL || out_frame_ready == NULL) {
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }

    *out_frame_ready = false;

    switch (parser->state) {
    case BCCAM_UART_PARSE_WAIT_MAGIC0:
        if (byte == BCCAM_UART_MAGIC0) {
            parser->state = BCCAM_UART_PARSE_WAIT_MAGIC1;
        } else {
            parser->rx_resyncs++;
        }
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_WAIT_MAGIC1:
        if (byte == BCCAM_UART_MAGIC1) {
            parser->state = BCCAM_UART_PARSE_VERSION;
        } else if (byte == BCCAM_UART_MAGIC0) {
            parser->rx_resyncs++;
            parser->state = BCCAM_UART_PARSE_WAIT_MAGIC1;
        } else {
            parser->rx_resyncs++;
            parser_reset(parser);
        }
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_VERSION:
        parser->header_after_magic[0] = byte;
        if (byte != parser->expected_link_version) {
            parser->rx_version_errors++;
            parser_reset_after_error(parser, &byte, 1u);
            return BCCAM_UART_ERR_BAD_VERSION;
        }
        parser->state = BCCAM_UART_PARSE_SERVICE;
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_SERVICE:
        parser->service = byte;
        parser->header_after_magic[1] = byte;
        parser->state = BCCAM_UART_PARSE_LEN0;
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_LEN0:
        parser->header_after_magic[2] = byte;
        parser->payload_len = byte;
        parser->state = BCCAM_UART_PARSE_LEN1;
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_LEN1:
        parser->header_after_magic[3] = byte;
        parser->payload_len |= (uint16_t)byte << 8;
        if (parser->payload_len > parser->max_payload_len ||
            parser->payload_len > BCCAM_UART_NORMAL_MAX_PAYLOAD) {
            const uint8_t length_suffix[] = {
                parser->header_after_magic[2],
                parser->header_after_magic[3]
            };
            parser->rx_length_errors++;
            parser_reset_after_error(parser, length_suffix, sizeof(length_suffix));
            return BCCAM_UART_ERR_BAD_LENGTH;
        }
        parser->payload_index = 0;
        parser->state = (parser->payload_len == 0u) ?
                        BCCAM_UART_PARSE_CRC0 :
                        BCCAM_UART_PARSE_PAYLOAD;
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_PAYLOAD:
        parser->payload[parser->payload_index++] = byte;
        if (parser->payload_index >= parser->payload_len) {
            parser->state = BCCAM_UART_PARSE_CRC0;
        }
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_CRC0:
        parser->crc_bytes[0] = byte;
        parser->state = BCCAM_UART_PARSE_CRC1;
        return BCCAM_UART_OK;

    case BCCAM_UART_PARSE_CRC1:
        parser->crc_bytes[1] = byte;
        return parser_finish_frame(parser, out_frame, out_frame_ready);

    default:
        parser_reset(parser);
        return BCCAM_UART_ERR_BAD_ARGUMENT;
    }
}
