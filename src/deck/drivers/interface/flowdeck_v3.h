#pragma once

#include <stdint.h>

/*
 * Template values for the UART framing.
 * Update these constants to match the final Flow deck V3 protocol.
 */
#define FLOWDECK_V3_UART_BAUDRATE    230400
// Flow and ToF measurements come in separate frames, each sent by the deck as
// soon as the sensor delivers it
#define FLOWDECK_V3_UART_FLOW_HEADER   0xFFFF
#define FLOWDECK_V3_UART_TOF_HEADER    0xFFFE
// Text messages from the deck, printed on the Crazyflie console
#define FLOWDECK_V3_UART_TEXT_HEADER   0xFFFD
#define FLOWDECK_V3_UART_TEXT_MAX_LENGTH 96
// Give up on a partially received frame and look for a header again
#define FLOWDECK_V3_UART_FRAME_TIMEOUT_MS 100

typedef struct {
	uint16_t motion;
	uint16_t deltaX;
	uint16_t deltaY;
	uint16_t shutter;
} __attribute__((packed)) flowdeckV3UartFlowFrame_t;

typedef struct {
	uint16_t rangeMm;
} __attribute__((packed)) flowdeckV3UartTofFrame_t;

// rangeMm value sent by the deck when no ToF zone has a valid measurement,
// for example when everything is out of range
#define FLOWDECK_V3_RANGE_INVALID 0xFFFE
