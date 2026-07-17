#pragma once

#include <stddef.h>
#include <stdint.h>

#define BCCAM_UART_CRC16_INITIAL 0xFFFFu

uint16_t bccam_uart_crc16_begin(void);
uint16_t bccam_uart_crc16_update(uint16_t crc, const uint8_t *data, size_t length);
uint16_t bccam_uart_crc16(const uint8_t *data, size_t length);
