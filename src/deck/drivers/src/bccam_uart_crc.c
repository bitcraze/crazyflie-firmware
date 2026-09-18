#include "bccam_uart_crc.h"

#define BCCAM_UART_CRC16_POLYNOMIAL 0x1021u

uint16_t bccam_uart_crc16_begin(void) {
    return BCCAM_UART_CRC16_INITIAL;
}

uint16_t bccam_uart_crc16_update(uint16_t crc, const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t)((crc << 1) ^ BCCAM_UART_CRC16_POLYNOMIAL);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }

    return crc;
}

uint16_t bccam_uart_crc16(const uint8_t *data, size_t length) {
    return bccam_uart_crc16_update(bccam_uart_crc16_begin(), data, length);
}
