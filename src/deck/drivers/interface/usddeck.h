#ifndef __USDDECK_H__
#define __USDDECK_H__

#include <stdint.h>
#include <stdbool.h>

#define USDLOG_ACC              (0x01)
#define USDLOG_GYRO             (0x02)
#define USDLOG_BARO             (0x04)
#define USDLOG_MAG              (0x08)
#define USDLOG_STABILIZER       (0x10)
#define USDLOG_CONTROL          (0x20)
#define USDLOG_RANGE            (0x40)

typedef struct usdLogDataPtr_s {
  uint32_t* tick;
  float* floats;
  int* ints;
} usdLogQueuePtr_t;

typedef struct usdLogConfig_s {
  char filename[13];
  uint8_t items;
  uint16_t frequency;
  uint8_t bufferSize;
  uint8_t floatSlots;
  uint8_t intSlots;
} usdLogConfig_t;

#define USD_WRITE(FILE, MESSAGE, BYTES, BYTES_WRITTEN, CRC_VALUE, CRC_FINALXOR, CRC_TABLE) \
  f_write(FILE, MESSAGE, BYTES, BYTES_WRITTEN); \
  CRC_VALUE = crcByByte(MESSAGE, BYTES, CRC_VALUE, CRC_FINALXOR, CRC_TABLE);

#endif //__USDDECK_H__
