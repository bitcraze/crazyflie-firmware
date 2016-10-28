#ifndef __LPS_TWR_TAG_H__
#define __LPS_TWR_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

extern uwbAlgorithm_t uwbTwrTagAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsTwrTagReportPayload_t;


#endif // __LPS_TWR_TAG_H__