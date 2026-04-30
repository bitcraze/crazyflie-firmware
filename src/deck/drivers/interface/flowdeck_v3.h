#pragma once

#include <stdint.h>

/*
 * Template values for the UART framing.
 * Update these constants to match the final Flow deck V3 protocol.
 */
#define FLOWDECK_V3_UART_BAUDRATE    230400
#define FLOWDECK_V3_UART_SYNC_HEADER   0xFFFF
#define FLOWDECK_V3_UART_SYNC_LENGTH 4

typedef struct {
	uint16_t motion;
	uint16_t deltaX;
	uint16_t deltaY;
    uint16_t shutter;
	uint16_t rangeMm;
} __attribute__((packed)) flowdeckV3UartFrame_t;
