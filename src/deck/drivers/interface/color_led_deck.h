#pragma once

#include <stdint.h>

#define COLORLED_DECK_I2C_ADDRESS 0x10

// Protocol commands
#define CMD_GET_VERSION         0x00
#define CMD_SET_COLOR           0x01
#define CMD_GET_THERMAL_STATUS  0x02

// Expected protocol version
#define COLORLED_PROTOCOL_VERSION_REQUIRED 1

#define TXBUFFERSIZE  5
#define RXBUFFERSIZE  3

typedef struct {
    uint8_t r, g, b, w;
} rgbw_t;

typedef struct {
    uint8_t r_lumens;
    uint8_t g_lumens;
    uint8_t b_lumens;
    uint8_t w_lumens;
} ledLuminance_t;

// LED luminance values from datasheet
static const ledLuminance_t LED_LUMINANCE = {
    .r_lumens = 90,
    .g_lumens = 210,
    .b_lumens = 50,
    .w_lumens = 250
};
