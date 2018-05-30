#ifndef __USDDECK_H__
#define __USDDECK_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct usdLogDataPtr_s {
  uint32_t* tick;
  uint8_t* data;
} usdLogQueuePtr_t;

typedef struct usdLogConfig_s {
  char filename[13];
  uint8_t items;
  uint16_t frequency;
  uint8_t bufferSize;
  uint16_t numSlots;
  uint16_t numBytes;
  int* varIds; // dynamically allocated
} usdLogConfig_t;

#define USD_WRITE(FILE, MESSAGE, BYTES, BYTES_WRITTEN, CRC_VALUE, CRC_FINALXOR, CRC_TABLE) \
  f_write(FILE, MESSAGE, BYTES, BYTES_WRITTEN); \
  CRC_VALUE = crcByByte(MESSAGE, BYTES, CRC_VALUE, CRC_FINALXOR, CRC_TABLE);

#endif //__USDDECK_H__
