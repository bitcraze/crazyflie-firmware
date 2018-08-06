#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef LED_RING_NBR_LEDS
#define NBR_LEDS  LED_RING_NBR_LEDS
#else
#define NBR_LEDS  12
#endif

extern uint8_t ledringmem[NBR_LEDS * 2];

#endif //__LEDRING12_H__
