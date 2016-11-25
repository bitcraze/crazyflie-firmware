#ifndef __LPS_TDOA_TAG_H__
#define __LPS_TDOA_TAG_H__

#include "locodeck.h"
#include "libdw1000.h"

#include "mac.h"

extern uwbAlgorithm_t uwbTdoaTagAlgorithm;

typedef struct rangePacket_s {
  uint8_t type;
  uint8_t txMaster[5];
  uint8_t timestamps[LOCODECK_NR_OF_ANCHORS][5];
} __attribute__((packed)) rangePacket_t;

#endif // __LPS_TDOA_TAG_H__
