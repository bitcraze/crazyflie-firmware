#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef LED_RING_NBR_LEDS
#define NBR_LEDS  LED_RING_NBR_LEDS
#else
#define NBR_LEDS  12
#endif

#ifndef LEDRING_TIME_MEM_SIZE
#define LEDRING_TIME_MEM_SIZE 50
#endif

typedef struct timings
{
  uint8_t duration;       // How long this color should be show in parts of 1/25s. So 25 will show the color 1s, before going to the next color.
  uint8_t color[2];       // What color should be shown in RGB565 format.
  unsigned leds:5;        // What led should be show, 0 equals all leds.
  bool fade;              // Fade from previous colour to this colour during the full duration.
  unsigned rotate:2;      // Speed of the rotation, number of seconds per revolution
} ledtimings;

extern uint8_t ledringmem[NBR_LEDS * 2];
extern ledtimings ledringtimingsmem[LEDRING_TIME_MEM_SIZE];

#endif //__LEDRING12_H__
