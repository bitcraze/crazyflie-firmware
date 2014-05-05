#ifndef __NEOPIXELRING_H__
#define __NEOPIXELRING_H__

#include <stdbool.h>

typedef void (*NeopixelRingEffect)(uint8_t buffer[][3], bool reset);

void neopixelringInit(void);

#endif //__NEOPIXELRING_H__
