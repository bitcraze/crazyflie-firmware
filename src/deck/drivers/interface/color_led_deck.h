#pragma once

#include <stdint.h>

// Protocol commands
#define CMD_GET_VERSION         0x00
#define CMD_SET_COLOR           0x01  // Legacy, already corrected [CMD, W, R, G, B]
#define CMD_GET_THERMAL_STATUS  0x02
#define CMD_GET_LED_POSITION    0x03
#define CMD_GET_LED_CURRENT     0x04
#define CMD_GET_I2C_ADDR_PIN    0x05
#define CMD_SET_BRIGHTNESS_CORR 0x06  // Enable/disable brightness correction [CMD, enable, 0, ...]

#define CMD_SET_COLOR_FADE      0x07  // Raw color + fade duration (float32 seconds)

// Expected protocol version
#define COLORLED_PROTOCOL_VERSION_REQUIRED 4

// LED physical position on the deck PCB (hardware-configured)
#define COLORLED_LED_POS_NONE           0x00  // No LED / floating
#define COLORLED_LED_POS_BOTTOM         0x01  // Bottom-mounted
#define COLORLED_LED_POS_TOP            0x02  // Top-mounted

#define COLORLED_REQUEST_SIZE 5  // Stable legacy commands, including GET_VERSION
#define COLORLED_FADE_REQUEST_SIZE 9
#define RXBUFFERSIZE  9
