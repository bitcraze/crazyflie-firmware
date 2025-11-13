#pragma once

#include <stdint.h>

// Protocol commands
#define CMD_GET_VERSION         0x00
#define CMD_SET_COLOR           0x01
#define CMD_GET_THERMAL_STATUS  0x02
#define CMD_GET_LED_POSITION    0x03
#define CMD_GET_LED_CURRENT     0x04

// Expected protocol version
#define COLORLED_PROTOCOL_VERSION_REQUIRED 2

#define TXBUFFERSIZE  5
#define RXBUFFERSIZE  9

typedef struct {
    uint8_t w, r, g, b;
} wrgb_t;

typedef struct {
    uint8_t r_lumens;
    uint8_t g_lumens;
    uint8_t b_lumens;
    uint8_t w_lumens;
} ledLuminance_t;

// LED luminance values from datasheet, adjusted for actual circuit current
// Datasheet values assume equal current, but actual current depends on both
// sense resistor AND LED forward voltage (V_F):
//
// Circuit: I = (Vsupply - V_F) / (R_mosfet + R_sense)
// - Red:   I = (3.1V - 2.1V) / (0.070Ω + 3.4Ω) = 288 mA → 1.54× datasheet current
// - G/B/W: I = (3.1V - 2.9V) / (0.070Ω + 1.0Ω) = 187 mA → 1.0× datasheet current
//
// Red's lower V_F gives more voltage headroom, overcoming higher sense resistance
// LED brightness scales approximately linearly with current
static const ledLuminance_t LED_LUMINANCE = {
    .r_lumens = 139,  // 90 lumens @ datasheet current × 1.54 current ratio = 139 lumens actual
    .g_lumens = 210,
    .b_lumens = 50,
    .w_lumens = 250
};
