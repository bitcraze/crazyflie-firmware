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
#define LEDRING_TIME_MEM_SIZE 10
#endif

extern uint8_t ledringmem[NBR_LEDS * 2];

typedef struct __attribute__((packed)) timing
{
  uint8_t duration;       // How long this color should be show in parts of 1/25s. So 25 will show the color 1s, before going to the next color.
  uint8_t color[2];       // What color should be shown in RGB565 format.
  unsigned leds:4;        // What led should be show, 0 equals all leds.
  bool fade:1;            // Fade from previous colour to this colour during the full duration.
  unsigned rotate:3;      // Speed of the rotation, number of seconds per revolution
} ledtiming;

typedef struct timings
{
  uint32_t hash;          // Hash will later be used to check if the contents is already uploaded, so we don't reupload
  ledtiming timings[LEDRING_TIME_MEM_SIZE];
} ledtimings;

extern ledtimings ledringtimingsmem;

#endif //__LEDRING12_H__
