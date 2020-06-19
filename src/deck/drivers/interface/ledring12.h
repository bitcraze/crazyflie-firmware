#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef LED_RING_NBR_LEDS
#define NBR_LEDS  LED_RING_NBR_LEDS
#else
#define NBR_LEDS  12
#endif

typedef struct timings
{
  uint8_t duration;       // How long this color should be show in parts of 1/25s. So 25 will show the color 1s, before going to the next color.
  uint8_t color[2];     // What color should be shown as index in the ledringmem memory
} ledtimings;

extern uint8_t ledringmem[NBR_LEDS * 2];
extern ledtimings ledringtimingsmem[3000];

#endif //__LEDRING12_H__
