#pragma once

#include <stdint.h>

#define HPRGBW_DECK_I2C_ADDRESS 0x10

// Protocol commands
#define CMD_GET_VERSION         0x00
#define CMD_SET_COLOR           0x01
#define CMD_GET_THERMAL_STATUS  0x02

// Expected protocol version
#define HP_LED_PROTOCOL_VERSION_REQUIRED 1

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
} led_luminance_t;

// LED luminance values from datasheet
static const led_luminance_t LED_LUMINANCE = {
    .r_lumens = 90,
    .g_lumens = 210,
    .b_lumens = 50,
    .w_lumens = 250
};

static rgbw_t apply_brightness_correction(const rgbw_t *input_rgbw);
