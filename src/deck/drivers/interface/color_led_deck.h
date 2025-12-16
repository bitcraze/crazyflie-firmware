#pragma once

#include <stdint.h>

// Protocol commands
#define CMD_GET_VERSION         0x00
#define CMD_SET_COLOR           0x01
#define CMD_GET_THERMAL_STATUS  0x02
#define CMD_GET_LED_POSITION    0x03
#define CMD_GET_LED_CURRENT     0x04
#define CMD_GET_I2C_ADDR_PIN    0x05

// Expected protocol version
#define COLORLED_PROTOCOL_VERSION_REQUIRED 3

// LED physical position on the deck PCB (hardware-configured)
#define COLORLED_LED_POS_NONE           0x00  // No LED / floating
#define COLORLED_LED_POS_BOTTOM         0x01  // Bottom-mounted
#define COLORLED_LED_POS_TOP            0x02  // Top-mounted

#define TXBUFFERSIZE  5
#define RXBUFFERSIZE  9

typedef struct {
    uint8_t w, r, g, b;
} wrgb_t;

typedef struct {
    float w;
    float r;
    float g;
    float b;
} ledPerceptualScale_t;

// Perceptual balance factors from user survey
// These scale brightness values to achieve perceptually balanced colors
// Blue is observed as weakest, others are scaled relative to it
static const ledPerceptualScale_t LED_PERCEPTUAL_SCALE = {
    .w = 0.99f,
    .r = 0.78f,
    .g = 0.51f,
    .b = 1.0f
};
