#pragma once

#include <stdint.h>

// Protocol commands
#define CMD_GET_VERSION         0x00
#define CMD_SET_COLOR           0x01  // Set LED color (raw, pre-correction) [CMD, W, R, G, B]
#define CMD_GET_THERMAL_STATUS  0x02
#define CMD_GET_LED_POSITION    0x03
#define CMD_GET_LED_CURRENT     0x04
#define CMD_GET_I2C_ADDR_PIN    0x05
#define CMD_SET_BRIGHTNESS_CORR 0x06  // Enable/disable brightness correction [CMD, enable, 0, ...]

// Expected protocol version
#define COLORLED_PROTOCOL_VERSION_REQUIRED 3

// LED physical position on the deck PCB (hardware-configured)
#define COLORLED_LED_POS_NONE           0x00  // No LED / floating
#define COLORLED_LED_POS_BOTTOM         0x01  // Bottom-mounted
#define COLORLED_LED_POS_TOP            0x02  // Top-mounted

#define TXBUFFERSIZE  5
#define RXBUFFERSIZE  9
