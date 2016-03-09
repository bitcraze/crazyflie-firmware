#ifndef __WS2812_H__
#define __WS2812_H__

void ws2812Init(void);
void ws2812Send(uint8_t (*color)[3], uint16_t len);
void ws2812DmaIsr(void);

#endif
