#ifndef __USBDECK_H__
#define __USBDECK_H__

#include <stdint.h>
#include <stdbool.h>
#include "queue.h"

typedef struct
{
  uint32_t timestamp;
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float mx;
  float my;
  float mz;
}  __attribute__((packed)) UsdLogStruct;

extern xQueueHandle usdDataQueue;

bool usdQueueLogData(UsdLogStruct* logData);


#endif //__USBDECK_H__
