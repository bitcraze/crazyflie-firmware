#pragma once

#include <stdint.h>

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
