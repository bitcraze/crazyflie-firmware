#ifndef __LEDRING12_H__
#define __LEDRING12_H__

#include <stdint.h>
#include <stdbool.h>

#define NBR_LEDS  12

extern uint8_t ledringmem[NBR_LEDS * 2];

typedef void (*Ledring12Effect)(uint8_t buffer[][3], bool reset);

void ledring12Init(void);

#endif //__LEDRING12_H__
