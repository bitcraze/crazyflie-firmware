#ifndef __NEOPIXELRING_H__
#define __NEOPIXELRING_H__

#include <stdint.h>
#include <stdbool.h>

#define NBR_LEDS  12

extern uint8_t ledringmem[NBR_LEDS * 2];

typedef void (*NeopixelRingEffect)(uint8_t buffer[][3], bool reset);

void neopixelringInit(void);

#endif //__NEOPIXELRING_H__
