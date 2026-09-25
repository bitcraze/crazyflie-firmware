#pragma once

#include <stdint.h>

/*
 * Template values for the UART framing.
 * Update these constants to match the final Flow deck V3 protocol.
 */
#define FLOWDECK_V3_UART_BAUDRATE    230400
#define FLOWDECK_V3_UART_SYNC_HEADER   0xFFFF
// Text messages from the deck, printed on the Crazyflie console
#define FLOWDECK_V3_UART_TEXT_HEADER   0xFFFD
#define FLOWDECK_V3_UART_TEXT_MAX_LENGTH 96
// Give up on a partially received frame and look for a header again
#define FLOWDECK_V3_UART_FRAME_TIMEOUT_MS 100
#define FLOWDECK_V3_UART_SYNC_LENGTH 4

typedef struct {
	uint16_t motion;
	uint16_t deltaX;
	uint16_t deltaY;
    uint16_t shutter;
	uint16_t rangeMm;
} __attribute__((packed)) flowdeckV3UartFrame_t;

// rangeMm value sent by the deck when it has no valid range
#define FLOWDECK_V3_RANGE_INVALID 0xFFFE
